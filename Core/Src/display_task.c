#include "cmsis_os2.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "imu_math.h"
#include "lcd_driver.h"
#include "projdefs.h"
#include "stm32g031xx.h"
#include "stm32g0xx_hal.h"
#include "cmsis_os2.h"
#include "stm32g0xx_hal_spi.h"
#include "stm32g0xx_hal_uart.h"
#include "display_task.h"

#define CALIBRATION_FLAG 0x0002

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern osMessageQueueId_t eulerQueue;

lcd_t display;
vec3 angles;
vec3 prev_angles;

void draw_gui(vec3* euler_angles, int isOn){
  float y = 120 + 120 * tan((3.1415f/180.0f) * euler_angles->x);
  float x = 160;
  lcd_draw_line_point_angle(&display, x, y, euler_angles->y, isOn);
}

void thread_func_display(void* argument){
  construct_lcd(&hspi1, GPIOA, GPIO_PIN_12, GPIOB, GPIO_PIN_1, GPIOA, GPIO_PIN_4, &display);
  lcd_init(&display);
  clear_display(&display);
  uint8_t display_status[5];
  read_display_status(&display, display_status);
  char msg[40];
  int len = sprintf(msg, "%02X %02X %02X %02X %02X\r\n",
                    display_status[0], display_status[1], display_status[2],
                    display_status[3], display_status[4]);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 20);
  lcd_draw_string(&display, 20, 20, "CALLIBRATING...");
  osThreadFlagsWait(CALIBRATION_FLAG, osFlagsWaitAny, osWaitForever);
  prev_angles = construct_vec(0, 0, 0, 1);
  lcd_draw_vec(&display, 20, 20, &angles, "PITCH: ", "ROLL: ", "YAW: ");
  draw_gui(&prev_angles, 1);

  while(1){
    osStatus_t status = osMessageQueueGet(eulerQueue, &angles, NULL, 20);
    if(status == osOK && (abs((int) angles.x - (int)prev_angles.x) > 3 || abs((int) angles.y - (int)prev_angles.y) > 3 || abs((int) angles.z - (int)prev_angles.z) > 3)){
      draw_gui(&prev_angles, 0);
      draw_gui(&angles, 1);
      lcd_draw_vec(&display, 20, 20, &angles, "PITCH: ", "ROLL: ", "YAW: ");
      prev_angles = angles;
    }
  }
}