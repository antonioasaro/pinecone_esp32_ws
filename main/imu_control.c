#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "sdkconfig.h"

#include <driver/i2c.h>
#include "mpu6050.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68

static char TAG[] = "IMU";
static mpu6050_handle_t mpu6050_dev = NULL;
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                         \
	do                                             \
	{                                              \
		esp_err_t rc = (x);                        \
		if (rc != ESP_OK)                          \
		{                                          \
			ESP_LOGE("err", "esp_err_t = %d", rc); \
			assert(0 && #x);                       \
		}                                          \
	} while (0);

void imu_init(void)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = PIN_SDA, // select SDA GPIO specific to your project
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = PIN_CLK, // select SCL GPIO specific to your project
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000, // select frequency specific to your project
		.clk_flags = 0,				 // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
	};
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	mpu6050_dev = mpu6050_create(0x0, 0x68);
	mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
	mpu6050_wake_up(mpu6050_dev);
	vTaskDelay(500 / portTICK_PERIOD_MS);
}

int32_t imu_control_read_accel()
{
    ESP_LOGI(TAG, "Requesting imu accel status");
    return (0);
}

void imu_task(void *ignore)
{
	imu_init();

	ESP_LOGI(TAG, "Sampling accel.xyz from imu");
	mpu6050_get_acce(mpu6050_dev, &acce);
	mpu6050_get_gyro(mpu6050_dev, &gyro);
	ESP_LOGI(TAG, "imu acce %d %d %d", (int)acce.acce_x, (int)acce.acce_y, (int)acce.acce_z);
	ESP_LOGI(TAG, "imu gyro %d %d %d", (int)gyro.gyro_x, (int)gyro.gyro_y, (int)gyro.gyro_z);

	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelete(NULL);
}