/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"

///////////////////////////////Change the following configurations according to your board//////////////////////////////
#define STEP_MOTOR_GPIO_EN     0
#define STEP_MOTOR_GPIO_DIR_x     2
#define STEP_MOTOR_GPIO_STEP_x     4
#define STEP_MOTOR_GPIO_DIR_y     2
#define STEP_MOTOR_GPIO_STEP_y     4
#define STEP_MOTOR_GPIO_DIR_z     2
#define STEP_MOTOR_GPIO_STEP_z     4
#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static const char *TAG = "example";

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config_x = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR_x | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config_x));
    gpio_config_t en_dir_gpio_config_y = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR_y | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config_y));
    gpio_config_t en_dir_gpio_config_z = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR_z | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config_z));

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t motor_chan_x = NULL;
    rmt_channel_handle_t motor_chan_y = NULL;
    rmt_channel_handle_t motor_chan_z = NULL;

    rmt_tx_channel_config_t tx_chan_config_x = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP_x,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_x, &motor_chan_x));
    rmt_tx_channel_config_t tx_chan_config_y = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP_y,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_y, &motor_chan_y));
    rmt_tx_channel_config_t tx_chan_config_z = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP_z,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_z, &motor_chan_z));

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR_x, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    gpio_set_level(STEP_MOTOR_GPIO_DIR_y, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    gpio_set_level(STEP_MOTOR_GPIO_DIR_z, STEP_MOTOR_SPIN_DIR_CLOCKWISE);

    ESP_LOGI(TAG, "Enable step motor"); //REVISIT TO CHECK IF NEEDS TO BE TRIPLED
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config_x = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    stepper_motor_curve_encoder_config_t accel_encoder_config_y = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    stepper_motor_curve_encoder_config_t accel_encoder_config_z = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    rmt_encoder_handle_t accel_motor_encoder_x = NULL;
    rmt_encoder_handle_t accel_motor_encoder_y = NULL;
    rmt_encoder_handle_t accel_motor_encoder_z = NULL;

    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config_x, &accel_motor_encoder_x));
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config_y, &accel_motor_encoder_y));
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config_z, &accel_motor_encoder_z));


    stepper_motor_uniform_encoder_config_t uniform_encoder_config_x = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder_x = NULL;
    stepper_motor_uniform_encoder_config_t uniform_encoder_config_y = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder_y = NULL;
    stepper_motor_uniform_encoder_config_t uniform_encoder_config_z = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder_z = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config_x, &uniform_motor_encoder_x));
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config_y, &uniform_motor_encoder_y));
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config_z, &uniform_motor_encoder_z));


    stepper_motor_curve_encoder_config_t decel_encoder_config_x = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = 500,
    };
    stepper_motor_curve_encoder_config_t decel_encoder_config_y = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = 500,
    };
    stepper_motor_curve_encoder_config_t decel_encoder_config_z = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = 500,
    };
    rmt_encoder_handle_t decel_motor_encoder_x = NULL;
    rmt_encoder_handle_t decel_motor_encoder_y = NULL;
    rmt_encoder_handle_t decel_motor_encoder_z = NULL;

    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config_x, &decel_motor_encoder_x));
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config_y, &decel_motor_encoder_y));
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config_z, &decel_motor_encoder_z));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan_x));
    ESP_ERROR_CHECK(rmt_enable(motor_chan_y));
    ESP_ERROR_CHECK(rmt_enable(motor_chan_z));


    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config_x = {
        .loop_count = 0,
    };
    rmt_transmit_config_t tx_config_y = {
        .loop_count = 0,
    };
    rmt_transmit_config_t tx_config_z = {
        .loop_count = 0,
    };

    const static uint32_t accel_samples = 500;
    const static uint32_t uniform_speed_hz = 1500;
    const static uint32_t decel_samples = 500;

    while (1) 
    {
        // acceleration phase
        tx_config_x.loop_count = 0;
        tx_config_y.loop_count = 0;
        tx_config_z.loop_count = 0;

        ESP_ERROR_CHECK(rmt_transmit(motor_chan_x, accel_motor_encoder_x, &accel_samples, sizeof(accel_samples), &tx_config_x));
        ESP_ERROR_CHECK(rmt_transmit(motor_chan_y, accel_motor_encoder_y, &accel_samples, sizeof(accel_samples), &tx_config_y));
        ESP_ERROR_CHECK(rmt_transmit(motor_chan_z, accel_motor_encoder_z, &accel_samples, sizeof(accel_samples), &tx_config_z));


        // uniform phase
        tx_config_x.loop_count = 5000;
        tx_config_y.loop_count = 5000;
        tx_config_z.loop_count = 5000;

        ESP_ERROR_CHECK(rmt_transmit(motor_chan_x, uniform_motor_encoder_x, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config_x));
        ESP_ERROR_CHECK(rmt_transmit(motor_chan_y, uniform_motor_encoder_y, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config_y));
        ESP_ERROR_CHECK(rmt_transmit(motor_chan_z, uniform_motor_encoder_z, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config_z));


        // deceleration phase
        tx_config_x.loop_count = 0;
        tx_config_y.loop_count = 0;
        tx_config_z.loop_count = 0;

        ESP_ERROR_CHECK(rmt_transmit(motor_chan_x, decel_motor_encoder_x, &decel_samples, sizeof(decel_samples), &tx_config_x));
        ESP_ERROR_CHECK(rmt_transmit(motor_chan_y, decel_motor_encoder_y, &decel_samples, sizeof(decel_samples), &tx_config_y));
        ESP_ERROR_CHECK(rmt_transmit(motor_chan_z, decel_motor_encoder_z, &decel_samples, sizeof(decel_samples), &tx_config_z));

        // wait all transactions finished
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan_x, -1));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan_y, -1));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan_z, -1));


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
