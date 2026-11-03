#include "lcd_driver.h"
#include "imu_math.h"
#include "stm32g031xx.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_gpio.h"
#include "stm32g0xx_hal_spi.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"

void split_int(uint16_t whole, uint8_t* high, uint8_t* low){
    *high = whole >> 8;
    *low = whole & 0xFF;
}

void lcd_cmd(lcd_t *lcd, uint8_t cmd){
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->dc_port, lcd->dc_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(lcd->spi, &cmd, 1, 15);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_SET);
}

void lcd_data_chunk(lcd_t *lcd, uint8_t *data, uint16_t len){
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->dc_port, lcd->dc_pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(lcd->spi, data, len, 100);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_SET);
}

void lcd_data(lcd_t* lcd, uint8_t data){
    lcd_data_chunk(lcd, &data, 1);
}

void lcd_init(lcd_t* lcd){
    //hardware reset
    HAL_GPIO_WritePin(lcd->reset_port, lcd->reset_pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(lcd->reset_port, lcd->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(lcd->reset_port, lcd->reset_pin, GPIO_PIN_SET);
    HAL_Delay(120);

    //init commands
    lcd_cmd(lcd, 0x11);
    lcd_cmd(lcd, 0x29);
    lcd_cmd(lcd, 0x3A);
    lcd_data(lcd, 0x55);
    lcd_cmd(lcd, 0x36);
    lcd_data(lcd, 0xE8);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_SET);
}

void read_display_status(lcd_t* lcd, uint8_t* out){
    lcd_cmd(lcd, 0x09);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Receive(lcd->spi, out, 5, 10);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_SET);
}

void construct_lcd(SPI_HandleTypeDef* spi, GPIO_TypeDef* dc_port, uint16_t dc_pin, GPIO_TypeDef* reset_port, uint16_t reset_pin, GPIO_TypeDef* cs_port, uint16_t cs_pin, lcd_t* out){
    out->spi = spi;
    out->dc_port = dc_port;
    out->dc_pin = dc_pin;
    out->reset_port = reset_port;
    out->reset_pin = reset_pin;
    out->cs_port = cs_port;
    out->cs_pin = cs_pin;
}

void clear_display(lcd_t* lcd){
    uint8_t buf[64];
    for (int i = 0; i < 64; i += 2) { buf[i] = 0x00; buf[i+1] = 0x00; }

    uint8_t ca[4] = {0, 0, 0x01, 0x3F};
    uint8_t pa[4] = {0, 0, 0, 239};   // rows 0..319
    lcd_cmd(lcd, 0x2A); lcd_data_chunk(lcd, ca, 4);
    lcd_cmd(lcd, 0x2B); lcd_data_chunk(lcd, pa, 4);
    lcd_cmd(lcd, 0x2C);
    for (int i = 0; i < 240 * 320 / 32; i++)
        lcd_data_chunk(lcd, buf, 64);
}

void lcd_set_up_drawing_region(lcd_t* lcd, uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1){
    uint8_t column_address_data[4];
    uint8_t page_address_data[4];
    split_int(x_0, &column_address_data[0], &column_address_data[1]);
    split_int(x_1, &column_address_data[2], &column_address_data[3]);
    split_int(y_0, &page_address_data[0], &page_address_data[1]);
    split_int(y_1, &page_address_data[2], &page_address_data[3]);

    lcd_cmd(lcd, 0x2A);
    lcd_data_chunk(lcd, column_address_data, 4);
    lcd_cmd(lcd, 0x2B);
    lcd_data_chunk(lcd, page_address_data, 4);
}

void lcd_draw_char(lcd_t *lcd, uint16_t x, uint16_t y, char ca){
    const uint8_t color_on[2] = {0xF8, 0x00};
    const uint8_t color_off[2] = {0x00, 0x00};

    lcd_set_up_drawing_region(lcd, x, y, x + OUT_W - 1, y + OUT_H - 1);
    lcd_cmd(lcd, 0x2C);

    for(int r = 0; r < OUT_H; r++){
        int row = r / SCALE;
        uint8_t row_data[OUT_W * 2] = {0};
        int n = 0;
        for(int c = 0; c < OUT_W; c++){
            int col = c / SCALE;
            uint8_t curr_byte = font_data[(uint8_t)ca*5 + col];
            uint8_t mask = 1 << row;
            if(curr_byte & mask){
                row_data[n++] = color_on[0];
                row_data[n++] = color_on[1];
            } else {
                row_data[n++] = color_off[0];
                row_data[n++] = color_off[1];
            }
        }
        lcd_data_chunk(lcd, row_data, sizeof(row_data));
    }
}

int lcd_draw_int(lcd_t* lcd, uint16_t x, uint16_t y, int value){
    int curr_x = x;

    if(value < 0){
        value = abs(value);
        lcd_draw_char(lcd, curr_x, y, '-');
        curr_x += OUT_W + 2 * SCALE;
    }

    char val_str[4] = {' ',' ',' ', '\0'};
    int n = 2;
    if(value == 0){
        val_str[0] = ' ';
        val_str[1] = ' ';
        val_str[2] = '0';
    } else {
        while(value > 0 && n >= 0){
            char curr = '0' + (value%10);
            val_str[n] = curr;
            n--;
            value /= 10;
        }
    }

    curr_x = lcd_draw_string(lcd, curr_x, y, val_str);

    return curr_x;
}

int lcd_draw_string(lcd_t* lcd, uint16_t x, uint16_t y, char* str){
    int curr_x = x;
    int n = strlen(str);

    for(int i = 0; i < n; i++){
        lcd_draw_char(lcd, curr_x, y, str[i]);
        curr_x += OUT_W + 2 * SCALE;
    }

    return curr_x;
}

void lcd_draw_vec(lcd_t* lcd, uint16_t x, uint16_t y, vec3* vec, char* x_name, char* y_name, char* z_name){
    int x_whole, x_part, y_whole, y_part, z_whole, z_part;
    float_to_ints(vec->x, &x_whole, &x_part);
    float_to_ints(vec->y, &y_whole, &y_part);
    float_to_ints(vec->z, &z_whole, &z_part);

    int curr_x = x;
    curr_x = lcd_draw_string(lcd, curr_x, y, x_name);
    if(x_part <= 0){
        curr_x = lcd_draw_string(lcd, curr_x, y, "-");
        x_part = abs(x_part);
    }
    curr_x = lcd_draw_int(lcd, curr_x, y, x_whole);
    curr_x = lcd_draw_string(lcd, curr_x, y, ".");
    lcd_draw_int(lcd, curr_x, y, x_part);

    y += OUT_H + 2 * SCALE;
    curr_x = x;
    curr_x = lcd_draw_string(lcd, curr_x, y, y_name);
    if(y_part <= 0){
        curr_x = lcd_draw_string(lcd, curr_x, y, "-");
        y_part = abs(y_part);
    }
    curr_x = lcd_draw_int(lcd, curr_x, y, y_whole);
    curr_x = lcd_draw_string(lcd, curr_x, y, ".");
    lcd_draw_int(lcd, curr_x, y, y_part);

    y += OUT_H + 2 * SCALE;
    curr_x = x;
    curr_x = lcd_draw_string(lcd, curr_x, y, z_name);
    if(z_part <= 0){
        curr_x = lcd_draw_string(lcd, curr_x, y, "-");
        z_part = abs(z_part);
    }
    curr_x = lcd_draw_int(lcd, curr_x, y, z_whole);
    curr_x = lcd_draw_string(lcd, curr_x, y, ".");
    lcd_draw_int(lcd, curr_x, y, z_part);
}

void lcd_draw_line_point_angle(lcd_t* lcd, uint16_t x, uint16_t y, float slope, int isOn){
    const float W = 319.0f, H = 239.0f;

    float d = fmodf(fabsf(slope), 180.0f);
    if (fabsf(d - 90.0f) < 0.01f) {
        lcd_draw_line(lcd, x, 0, x, 239, isOn);
        return;
    }
    
    slope = tanf(slope * (3.1415f / 180.0f));
    float x0 = 0.0f;
    float x1 = W;
    float y0 = y + (0.0f - x) * slope;
    float y1 = y + (W - x) * slope;

    if (y0 < 0.0f)      { x0 = x + (0.0f - y) / slope; y0 = 0.0f; }
    else if (y0 > H)    { x0 = x + (H    - y) / slope; y0 = H;    }
    if (y1 < 0.0f)      { x1 = x + (0.0f - y) / slope; y1 = 0.0f; }
    else if (y1 > H)    { x1 = x + (H    - y) / slope; y1 = H;    }

    lcd_draw_line(lcd, (int)lroundf(x0), (int)lroundf(y0), (int)lroundf(x1), (int)lroundf(y1), isOn);
}

void lcd_draw_line(lcd_t* lcd, uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1, int isOn){
    int dx = abs(x_1 - x_0);
    int dy = abs(y_1 - y_0);
    if(abs(dx) >= abs(dy)){
        if(x_0 > x_1){
            int temp = x_0;
            x_0 = x_1;
            x_1 = temp;

            temp = y_0;
            y_0 = y_1;
            y_1 = temp;
        }

        int sy = (y_0 < y_1) ? 1 : -1;

        int error = -1 * dx;
        int prev_x = x_0;
        int curr_x = x_0;
        int curr_y = y_0;
        for(int i = 1; i < abs(dx); i++){
            if(error + dy < 0){
                error = error + dy;
            } else {
                error = (error + dy) - dx;
                lcd_draw_rect(lcd, prev_x, curr_x, curr_y, isOn);
                prev_x = curr_x+1;
                curr_y+=sy;
            }
            curr_x++;
        }
        lcd_draw_rect(lcd, prev_x, curr_x, curr_y, isOn);
    } else {
        if(y_0 > y_1){
            int temp = y_0;
            y_0 = y_1;
            y_1 = temp;

            temp = x_0;
            x_0 = x_1;
            x_1 = temp;
        }
        
        int sx = (x_0 < x_1) ? 1 : -1;

        int error = -1 * dy;
        int prev_y = y_0;
        int curr_y = y_0;
        int curr_x = x_0;
        for(int i = 1; i < abs(dy); i++){
            if(error + dx < 0){
                error = error + dx;
            } else {
                error = (error + dx) - dy;
                lcd_draw_rect_vert(lcd, prev_y, curr_y, curr_x, isOn);
                prev_y = curr_y+1;
                curr_x+=sx;
            }
            curr_y++;
        }
        lcd_draw_rect_vert(lcd, prev_y, curr_y, curr_x, isOn);
    }
}

// void lcd_draw_rect(lcd_t* lcd, uint16_t x_0, uint16_t x_1, uint16_t y, int isOn){
//     const uint8_t color_on[2] = {0xF8, 0x00};
//     const uint8_t color_off[2] = {0x00, 0x00};
//     lcd_set_up_drawing_region(lcd, x_0, y, x_1, y);
//     uint8_t pixel_data[(x_1-x_0+1) * 2];
//     int n = 0;
//     while(n < sizeof(pixel_data)){
//         pixel_data[n++] = (isOn == 1) ? color_on[0] : color_off[0];
//         pixel_data[n++] = (isOn == 1) ? color_on[1] : color_off[1];
//     }
//     lcd_cmd(lcd, 0x2C);
//     lcd_data_chunk(lcd, pixel_data, sizeof(pixel_data));
// }

void lcd_draw_rect(lcd_t* lcd, uint16_t x_0, uint16_t x_1, uint16_t y, int isOn){
    if (x_1 > 319) x_1 = 319;
    if (y > 239 || x_1 < x_0) return;
    lcd_set_up_drawing_region(lcd, x_0, y, x_1, y);

    uint8_t pixel_data[64] = {0};                 // fixed size
    if (isOn == 1) {
        for (int i = 0; i < 64; i += 2) pixel_data[i] = 0xF8;   // second byte stays 0x00
    }

    lcd_cmd(lcd, 0x2C);
    int remaining = x_1 - x_0 + 1;                // pixels
    while (remaining > 0) {
        int n = (remaining > 32) ? 32 : remaining;
        lcd_data_chunk(lcd, pixel_data, n * 2);
        remaining -= n;
    }
}

// void lcd_draw_rect_vert(lcd_t* lcd, uint16_t y_0, uint16_t y_1, uint16_t x, int isOn){
//     const uint8_t color_on[2] = {0xF8, 0x00};
//     const uint8_t color_off[2] = {0x00, 0x00};
//     lcd_set_up_drawing_region(lcd, x, y_0, x, y_1);
//     uint8_t pixel_data[(y_1-y_0+1) * 2];
//     int n = 0;
//     while(n < sizeof(pixel_data)){
//         pixel_data[n++] = (isOn == 1) ? color_on[0] : color_off[0];
//         pixel_data[n++] = (isOn == 1) ? color_on[1] : color_off[1];
//     }
//     lcd_cmd(lcd, 0x2C);
//     lcd_data_chunk(lcd, pixel_data, sizeof(pixel_data));
// }

void lcd_draw_rect_vert(lcd_t* lcd, uint16_t y_0, uint16_t y_1, uint16_t x, int isOn){
    if (y_1 > 239) y_1 = 239;
    if (x > 319 || y_1 < y_0) return;
    lcd_set_up_drawing_region(lcd, x, y_0, x, y_1);

    uint8_t pixel_data[64] = {0};
    if (isOn == 1) {
        for (int i = 0; i < 64; i += 2) pixel_data[i] = 0xF8;
    }

    lcd_cmd(lcd, 0x2C);
    int remaining = y_1 - y_0 + 1;
    while (remaining > 0) {
        int n = (remaining > 32) ? 32 : remaining;
        lcd_data_chunk(lcd, pixel_data, n * 2);
        remaining -= n;
    }
}