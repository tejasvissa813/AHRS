#include "imu_math.h"
#include "math.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>

void float_to_ints(float input, int* whole, int* part){
    int neg = (input < 0.0f);
    
    int adj = (fabsf(input) * 1000.0f + 0.5f);
    int w = adj / 1000;
    int p = adj % 1000; 

    *whole = (neg) ? -w : w;
    *part = (neg && w == 0) ? -p : p;
}

vec3 construct_vec(float x_raw, float y_raw, float z_raw, float scale){
    return (vec3) { x_raw/scale, y_raw/scale, z_raw/scale};
}

quat construct_quat(float w_raw, float x_raw, float y_raw, float z_raw, float scale){
    return (quat) { w_raw/scale, x_raw/scale, y_raw/scale, z_raw/scale};
}

void load_vec(float* arr, const vec3* v){
    arr[0] = v->x;
    arr[1] = v->y;
    arr[2] = v->z;
}

void load_quat(float* arr, const quat* q){
    arr[0] = q->w;
    arr[1] = q->x;
    arr[2] = q->y;
    arr[3] = q->z;
}

void vector_norm(vec3 *vec){
    float mag = sqrt((vec->x * vec->x) + (vec->y * vec->y) + (vec->z * vec->z));
    vec->x /= mag;
    vec->y /= mag;
    vec->z /= mag;
}

void quat_norm(quat *q){
    float mag = sqrt((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
    q->w /= mag;
    q->x /= mag;
    q->y /= mag;
    q->z /= mag;
}

float vector_mag(vec3* vec){
    return sqrt((vec->x * vec->x) + (vec->y * vec->y) + (vec->z * vec->z));
}

float quat_mag(quat* q){
    return sqrt((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
}

void vector_scale(vec3* vec, float scale){
    vec->x /= scale;
    vec->y /= scale;
    vec->z /= scale;
}

void quat_scale(quat* q, float scale){
    q->w /= scale;
    q->x /= scale;
    q->y /= scale;
    q->z /= scale;
}

void vector_cross(const vec3 *v1, const vec3 *v2, vec3 *out){
    out->x = (v1->y * v2->z) - (v1->z * v2->y);
    out->y = (v1->z * v2->x) - (v1->x * v2->z);
    out->z = (v1->x * v2->y) - (v1->y * v2->x);
}

void quat_mult(const quat *q1, const quat *q2, quat *out){
    out->w = (q1->w * q2->w) - (q1->x * q2->x) - (q1->y * q2->y) - (q1->z * q2->z);
    out->x = (q1->w * q2->x) + (q1->x * q2->w) + (q1->y * q2->z) - (q1->z * q2->y);
    out->y = (q1->w * q2->y) - (q1->x * q2->z) + (q1->y * q2->w) + (q1->z * q2->x);
    out->z = (q1->w * q2->z) + (q1->x * q2->y) - (q1->y * q2->x) + (q1->z * q2->w);
}

void quat_rotate(const quat *q, const vec3 *vec, vec3 *out){
    quat q_inv = (quat) {q->w, -1*q->x, -1*q->y, -1*q->z};
    quat q_vec = (quat) {0, vec->x, vec->y, vec->z};
    quat temp;
    quat q_out;

    quat_mult(q, &q_vec, &temp);
    quat_mult(&temp, &q_inv, &q_out);

    out->x = q_out.x;
    out->y = q_out.y;
    out->z = q_out.z;
}

void vectors_to_quat(const vec3 *v1, const vec3 *v2, quat *q){
    vec3 axis;
    float dot;

    vector_cross(v1, v2, &axis);
    dot = (v1->x * v2->x) + (v1->y * v2->y) + (v1->z * v2->z);
    q->w = 1 + dot;
    q->x = axis.x;
    q->y = axis.y;
    q->z = axis.z;

    float mag = quat_mag(q);
    if(mag < 1e-12f){
        vec3 basis = (vec3) {0.0f, 0.0f, 0.0f};
        float ax = fabsf(v1->x), ay = fabsf(v1->y), az = fabsf(v1->z);
        if (ax <= ay && ax <= az) basis.x = 1.0f;
        else if (ay <= az) basis.y = 1.0f;
        else basis.z = 1.0f;

        vec3 ortho;
        vector_cross(v1, &basis, &ortho);
        float len = vector_mag(&ortho);

        if (len < 1e-12f) {
            q->w = 1.0f; 
            q->x = q->y = q->z = 0.0f; 
        } else {
            q->w = 0.0f;
            q->x = ortho.x/len;
            q->y = ortho.y/len;
            q->z = ortho.z/len;
        }

        return;
    }

    quat_norm(q);
}

void quat_to_euler(const quat *q, vec3 *out){
    out->x = atan2(2 * (q->y * q->z + q->w * q->x), 1 - 2 * (q->x * q->x + q->y * q->y)) * (180.0f / 3.1415);
    out->y = asin(2 * ((q->x * q->z) - (q->w * q->y))) * (180.0f / 3.1415);
    out->z = atan2(2 * ((q->x * q->y) + (q->w * q->z)), ((q->w * q->w) + (q->x * q->x)) - ((q->y * q->y) + (q->z * q->z))) * (180.0f / 3.1414);
}

void axis_angle_to_quat(const vec3 *axis, const float angle, quat *out){
    out->w = cos(angle/2);
    out->x = axis->x * sin(angle/2);
    out->y = axis->y * sin(angle/2);
    out->z = axis->z * sin(angle/2);
}

void print_vec(const vec3* vec, UART_HandleTypeDef* huart, const char* name){
    char msg[64];

    int x[2];
    int y[2];
    int z[2];
    float_to_ints(vec->x, &x[0], &x[1]);
    float_to_ints(vec->y, &y[0], &y[1]);
    float_to_ints(vec->z, &z[0], &z[1]);
    
    int len = sprintf(
        msg,
        "%s: <%d.%03d, %d.%03d, %d.%03d>\r\n",
        name,
        x[0],
        x[1],
        y[0],
        y[1],
        z[0],
        z[1]
    );

    HAL_UART_Transmit(huart, (uint8_t *) msg, len, 100);
}

void print_quat(const quat* q, UART_HandleTypeDef* huart, const char* name){
    char msg[64];

    int w[2];
    int x[2];
    int y[2];
    int z[2];
    float_to_ints(q->w, &w[0], &w[1]);
    float_to_ints(q->x, &x[0], &x[1]);
    float_to_ints(q->y, &y[0], &y[1]);
    float_to_ints(q->z, &z[0], &z[1]);
    
    int len = sprintf(
        msg,
        "%s: <%d.%03d, %d.%03d, %d.%03d, %d.%03d>\r\n",
        name,
        w[0],
        w[1],
        x[0],
        x[1],
        y[0],
        y[1],
        z[0],
        z[1]
    );

    HAL_UART_Transmit(huart, (uint8_t *) msg, len, 100);
}