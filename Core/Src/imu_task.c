#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "imu_math.h"
#include "portmacro.h"
#include "stm32g031xx.h"
#include "stm32g0xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#define FLAG_DMA_COMPLETE    0x0001
#define IMU_DEV_ADDR 0b11010000
#define DEG2RAD 0.01745329252f

#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC

extern osMessageQueueId_t eulerQueue;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern uint8_t sensor_bytes[16];
extern osThreadId_t displayTaskHandle;

volatile UBaseType_t g_imu_hwm;
volatile UBaseType_t g_disp_hwm;   // words free, all-time minimum

int adjusted_sensor_data[14];
int16_t sensor_data[7] = {0,0,0,0,0,0,0};
uint8_t local_sensor_bytes[16];

uint8_t pwr_mgmt_1 = 0x00;
uint8_t config = 0x03;
uint8_t sample_div = 9;
uint8_t accel_config = 0b00000000;
uint8_t gyro_config = 0b00010000;
uint8_t int_config = 0b00000000;
uint8_t int_enable = 0b00000001;
float accel_adjustment = 16384.0f;
float gyro_adjustment = 32.8f;


static uint16_t prev_ts = 0;
static uint8_t has_prev;
float time[2];
vec3 accel;
vec3 gyro;
quat rot;

vec3 gyro_bias;
vec3 accel_bias;
vec3 euler_angles;
quat q_cal;
quat q_gyro_sum;


void load_sensor_data(){
  memcpy(local_sensor_bytes, sensor_bytes, 16);
  
  //load raw bytes
  sensor_data[0] = (int16_t) (local_sensor_bytes[0] << 8) | local_sensor_bytes[1];
  sensor_data[1] = (int16_t) (local_sensor_bytes[2] << 8) | local_sensor_bytes[3];
  sensor_data[2] = (int16_t) (local_sensor_bytes[4] << 8) | local_sensor_bytes[5];
  sensor_data[3] = (int16_t) (local_sensor_bytes[6] << 8) | local_sensor_bytes[7];
  sensor_data[4] = (int16_t) (local_sensor_bytes[10] << 8) | local_sensor_bytes[11];
  sensor_data[5] = (int16_t) (local_sensor_bytes[12] << 8) | local_sensor_bytes[13];
  sensor_data[6] = (int16_t) (local_sensor_bytes[14] << 8) | local_sensor_bytes[15];

  //clear interrupt bit
  uint8_t stat;
  HAL_I2C_Mem_Read(&hi2c1, IMU_DEV_ADDR, 0x3A, I2C_MEMADD_SIZE_8BIT, &stat, 1, 100);
}

void callibrate_sensor_data(){
  int32_t avg_sensor_data[7] = {0};

  for(int i = 0; i < 500; i++){
    uint32_t flags = osThreadFlagsWait(FLAG_DMA_COMPLETE, osFlagsWaitAny, osWaitForever);
    if(!(flags & 0x80000000U) && (flags & FLAG_DMA_COMPLETE)){
        load_sensor_data();

        for(int j = 0; j < 7; j++){
            avg_sensor_data[j] += sensor_data[j];
        }
    }
  }

  for(int i = 0; i < 7; i++){
    avg_sensor_data[i] /= 500;
  }

  gyro_bias = construct_vec(avg_sensor_data[4], avg_sensor_data[5], avg_sensor_data[6], gyro_adjustment);
  accel_bias = construct_vec(avg_sensor_data[1], avg_sensor_data[2], avg_sensor_data[3], accel_adjustment);

  float grav_mag = vector_mag(&accel_bias);

  vector_norm(&accel_bias);
  vec3 north = (vec3) {0.0f, 0.0f, 1.0f};
  vectors_to_quat(&accel_bias, &north, &q_cal);
  rot = (quat) {1,0,0,0};

  accel_bias.x = 0;
  accel_bias.y = 0;
  accel_bias.z = grav_mag;

  euler_angles = construct_vec(0, 0, 0, 1);
  osThreadFlagsSet(displayTaskHandle, 0x0002);
}

void adjust_readings(){
  uint16_t time_stamp = (uint16_t)sensor_data[0];
  float dt = has_prev ? (uint16_t)(time_stamp - prev_ts) / 1000.0f : 0.01f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
  prev_ts = time_stamp;
  has_prev = 1;

  time[0] = 0.0f;
  time[1] = dt;
  accel = construct_vec(sensor_data[1], sensor_data[2], sensor_data[3], accel_adjustment);
  gyro = construct_vec(sensor_data[4], sensor_data[5], sensor_data[6], gyro_adjustment);

  // accel roatate->subtract accel bias
  vec3 accel_rotated;
  quat_rotate(&q_cal, &accel, &accel_rotated);
  accel = accel_rotated;

  //gyro subtract gyro_bias -> rotate
  vec3 gyro_adjusted;
  gyro_adjusted.x = gyro.x - gyro_bias.x;
  gyro_adjusted.y = gyro.y - gyro_bias.y;
  gyro_adjusted.z = gyro.z - gyro_bias.z;
  quat_rotate(&q_cal, &gyro_adjusted, &gyro);

  //reload
  float_to_ints((float) time[1], &adjusted_sensor_data[0], &adjusted_sensor_data[1]);
  float_to_ints((float) accel.x, &adjusted_sensor_data[2], &adjusted_sensor_data[3]);
  float_to_ints((float) accel.y, &adjusted_sensor_data[4], &adjusted_sensor_data[5]);
  float_to_ints((float) accel.z, &adjusted_sensor_data[6], &adjusted_sensor_data[7]);
  float_to_ints((float) gyro.x, &adjusted_sensor_data[8], &adjusted_sensor_data[9]);
  float_to_ints((float) gyro.y, &adjusted_sensor_data[10], &adjusted_sensor_data[11]);
  float_to_ints((float) gyro.z, &adjusted_sensor_data[12], &adjusted_sensor_data[13]);
}

void madgwick_step(){
  float dt = time[1] - time[0];

  float a = vector_mag(&accel);
  if (a < 1e-6f){
    return;
  }
  vector_norm(&accel);
  
  vec3 gyro_rad = { gyro.x * DEG2RAD, gyro.y * DEG2RAD, gyro.z * DEG2RAD };
  quat gyro_q = {0, gyro_rad.x, gyro_rad.y, gyro_rad.z};
  quat q_omega;
  quat_mult(&rot, &gyro_q, &q_omega);
  quat_scale(&q_omega, 2);

  quat q_grad;
  q_grad.w = 4 * rot.w * (rot.x * rot.x + rot.y * rot.y) + 2 * rot.y * accel.x - 2 * rot.x * accel.y;
  q_grad.x = 4 * rot.x * (rot.z * rot.z + rot.w * rot.w + 2 * rot.x * rot.x + 2 * rot.y * rot.y - 1 + accel.z) - 2 * rot.z * accel.x - 2 * rot.w * accel.y;
  q_grad.y = 4 * rot.y * (rot.w * rot.w + rot.z * rot.z + 2 * rot.x * rot.x + 2 * rot.y * rot.y - 1 + accel.z) + 2 * rot.w * accel.x - 2 * rot.z * accel.y;
  q_grad.z = 4 * rot.z * (rot.x * rot.x + rot.y * rot.y) - 2 * rot.x * accel.x - 2 * rot.y * accel.y;

  quat_norm(&q_grad);
  quat_scale(&q_grad, 200);

  quat q_integ;
  q_integ.w = q_omega.w - q_grad.w;
  q_integ.x = q_omega.x - q_grad.x;
  q_integ.y = q_omega.y - q_grad.y;
  q_integ.z = q_omega.z - q_grad.z;
  quat_scale(&q_integ, 1/dt);

  rot.w += q_integ.w;
  rot.x += q_integ.x;
  rot.y += q_integ.y;
  rot.z += q_integ.z;

  quat_norm(&rot);
}

void thread_func_imu(void *argument){
  // imu configs
  HAL_I2C_Mem_Write(&hi2c1, IMU_DEV_ADDR, 0x37, I2C_MEMADD_SIZE_8BIT, &int_config, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, IMU_DEV_ADDR, 0x38, I2C_MEMADD_SIZE_8BIT, &int_enable, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, IMU_DEV_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &pwr_mgmt_1, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, IMU_DEV_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &accel_config, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, IMU_DEV_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &gyro_config, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, IMU_DEV_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &config, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, IMU_DEV_ADDR, 0x19, I2C_MEMADD_SIZE_8BIT, &sample_div, 1, 100);

  char sucess_message[] = "SUCCESS";
  char fail_message[] = "FAIL";
  HAL_StatusTypeDef stat = HAL_I2C_IsDeviceReady(&hi2c1, IMU_DEV_ADDR, 10, 100);
  if(stat != 0x00) {
    HAL_UART_Transmit(&huart2, (uint8_t *) fail_message, strlen(fail_message), 100);
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t*) sucess_message, strlen(sucess_message), 100);
  }

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  callibrate_sensor_data();

  while(1){
    uint32_t flags = osThreadFlagsWait(FLAG_DMA_COMPLETE, osFlagsWaitAny, osWaitForever);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    if(!(flags & 0x80000000U) && (flags & FLAG_DMA_COMPLETE)){
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        load_sensor_data();
        adjust_readings();
        madgwick_step();
        quat_to_euler(&rot, &euler_angles);

        osMessageQueueReset(eulerQueue);
        osMessageQueuePut(eulerQueue, &euler_angles, 0, 0);
    }   
  }
}

