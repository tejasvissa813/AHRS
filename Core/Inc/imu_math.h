#ifndef IMU_MATH_H
#define IMU_MATH_H

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;
typedef struct {
    float x;
    float y;
    float z;
} vec3;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat;

vec3 construct_vec(float x_raw, float y_raw, float z_raw, float scale);
quat construct_quat(float w_raw, float x_raw, float y_raw, float z_raw, float scale);
void load_vec(float* arr, const vec3* v);
void loat_quat(float* arr, const quat* q);
void float_to_ints(float input, int* whole, int* part);

void vector_norm(vec3* vec);
void vector_cross(const vec3* v1, const vec3* v2, vec3* out);
void quat_norm(quat* q);

float vector_mag(vec3* vec);
float quat_mag(quat* q);
void vector_scale(vec3* vec, float scale);
void quat_scale(quat* q, float scale);

void quat_mult(const quat* q1, const quat* q2, quat* out);
void quat_rotate(const quat* q, const vec3* vec, vec3* out);
void vectors_to_quat(const vec3* v1, const vec3* v2, quat* q);

void quat_to_euler(const quat* q, vec3* out);
void axis_angle_to_quat(const vec3* axis, const float angle, quat* out);

void print_vec(const vec3* vec, UART_HandleTypeDef* huart, const char* name);
void print_quat(const quat* q, UART_HandleTypeDef* huart, const char* name);

#endif