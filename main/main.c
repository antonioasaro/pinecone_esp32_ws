#define ANTONIO
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#ifdef ANTONIO
#include <math.h>
#include <string.h>
#include <std_msgs/msg/int64.h>
#include <rcl_interfaces/msg/log.h>
#include "motor_control.h"
#include "ultrasonic_control.h"
#define MPU6050_LIB
#ifdef MPU6050_LIB
#include <driver/i2c.h>
#include "mpu6050.h"
#else
#include "mpu6050_control.h"
#endif
#endif
static char TAG[] = "MAIN";

#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
#ifdef ANTONIO
#define RADS_PER_ENC 1320
#define PID_LOOP_RATE 100
#define DIFFD_LOOP_RATE 30

void publish_rosout(char *msg_name, int32_t msg_data);
bool publisher_log_en = false;
rcl_publisher_t publisher_log;
rcl_subscription_t subscriber;
std_msgs__msg__Int64 send_msg;
std_msgs__msg__Int64 recv_msg;
#endif

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
#ifdef ANTONIO
		int64_t count = ((int64_t)motor_control_read_encoder()) & 0xFFFFFFFF;
		int64_t right_count = ((int64_t)right_motor_control_read_encoder()) & 0xFFFFFFFF;
		send_msg.data = (right_count << 32) | count;
		RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
#else
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
#endif
	}
}

#ifdef ANTONIO
void publish_rosout(char *msg_name, int32_t msg_data)
{
	char msg_data_str[32];
	sprintf(msg_data_str, "%d", (int)msg_data);

	rcl_interfaces__msg__Log msgLog;
	msgLog.level = rcl_interfaces__msg__Log__INFO;
	msgLog.name.data = msg_name;
	msgLog.name.size = strlen(msgLog.name.data);
	msgLog.msg.data = msg_data_str;
	msgLog.msg.size = strlen(msgLog.msg.data);
	msgLog.file.data = "";
	msgLog.file.size = strlen(msgLog.file.data);
	msgLog.function.data = "";
	msgLog.function.size = strlen(msgLog.function.data);
	msgLog.line = 0;
	if (publisher_log_en)
		RCSOFTCHECK(rcl_publish(&publisher_log, &msgLog, NULL));
}

void subscription_callback(const void *msgin)
{
	const std_msgs__msg__Int64 *msg = (const std_msgs__msg__Int64 *)msgin;
	int32_t left_wheel_speed = (int32_t)((msg->data >> 0) & 0xFFFFFFFF);
	int32_t right_wheel_speed = (int32_t)((msg->data >> 32) & 0xFFFFFFFF);
	left_wheel_speed = (left_wheel_speed / 1000) / ((2 * M_PI) / RADS_PER_ENC) / PID_LOOP_RATE;
	right_wheel_speed = (right_wheel_speed / 1000) / ((2 * M_PI) / RADS_PER_ENC) / PID_LOOP_RATE;
	motor_control_set_speed(left_wheel_speed);
	right_motor_control_set_speed(right_wheel_speed);
	// if (left_wheel_speed != 0)
	// 	publish_rosout("left_wheel", left_wheel_speed);
	// if (right_wheel_speed != 0)
	// 	publish_rosout("right_wheel", right_wheel_speed);
}
#endif

void micro_ros_task(void *arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
#ifdef ANTONIO
	RCCHECK(rclc_node_init_default(&node, "pinecone_esp32", "", &support));
#else
	RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));
#endif

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
#ifdef ANTONIO
		"wheel_encoders"));
#else
		"freertos_int32_publisher"));
#endif

#ifdef ANTONIO
	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_log,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
		"rosout"));
	publisher_log_en = true;

	// create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
		"wheel_speeds"));
#endif

	// create timer,
	rcl_timer_t timer;
#ifdef ANTONIO
	const unsigned int timer_timeout = 1000 / DIFFD_LOOP_RATE;
#else
	const unsigned int timer_timeout = 1000;
#endif
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
#ifdef ANTONIO
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));
#else
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
#endif

	msg.data = 0;

	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
#ifdef ANTONIO
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
#endif
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	rmw_uros_set_custom_transport(
		true,
		(void *)&uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

#ifdef ANTONIO
#ifdef MPU6050_LIB
#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68
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

	static mpu6050_handle_t mpu6050_dev = NULL;
	static mpu6050_acce_value_t acce;
	static mpu6050_gyro_value_t gyro;

	mpu6050_dev = mpu6050_create(0x0, 0x68);
	mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
	mpu6050_wake_up(mpu6050_dev);
	sleep(1);

	for (int i = 0; i < 32; i++)
	{
		mpu6050_get_acce(mpu6050_dev, &acce);
		mpu6050_get_gyro(mpu6050_dev, &gyro);
		ESP_LOGI(TAG, "mpu6050 acce %d %d %d", (int) acce.acce_x, (int) acce.acce_y, (int) acce.acce_z);
		ESP_LOGI(TAG, "mpu6050 gyro %d %d %d", (int) gyro.gyro_x, (int) gyro.gyro_y, (int) gyro.gyro_z);
		sleep(0.1);
	}
	sleep(1);
#else
	xTaskCreate((TaskFunction_t)mpu6050_task,
				"mpu6050",
				4096,
				NULL,
				1,
				NULL);
	sleep(1);
#endif

	xTaskCreate((TaskFunction_t)ultrasonic_task,
				"ultrasonic",
				4096,
				NULL,
				1,
				NULL);
	sleep(1);

	xTaskCreate((TaskFunction_t)motor_control_task,
				"wheel_motor_task",
				4096,
				NULL,
				1,
				NULL);

	sleep(1);
#endif

	xTaskCreate(micro_ros_task,
				"uros_task",
				CONFIG_MICRO_ROS_APP_STACK,
				NULL,
				CONFIG_MICRO_ROS_APP_TASK_PRIO,
				NULL);
}
