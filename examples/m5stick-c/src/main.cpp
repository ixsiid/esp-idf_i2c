#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "i2c.h"

extern "C"
{
	void app_main();
}

using namespace IXS;
#define SLAVE_ADDRESS 0x48
#define REGISTER_ADDRESS 0x55

#pragma pack(1)
union data_u
{
	uint8_t raw[4];
	TickType_t tick;
};
#pragma pack()

void master_task(void *args)
{
	I2CMaster *master = (I2CMaster *)args;
	data_u data = {0};

	esp_err_t err;
	while (true)
	{
		printf("read\n");
		vTaskDelay(3000 / portTICK_RATE_MS);

		err = master->read_bytes(SLAVE_ADDRESS, REGISTER_ADDRESS, data.raw, sizeof(data));
		if (err == ESP_OK) printf("Ticks: %ud\n", data.tick);
	}
}

void slave_task(void *args)
{
	I2CSlave *slave = (I2CSlave *)args;

	uint8_t rx[64] = {0};
	data_u data = {0};

	SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
	while (true)
	{
		vTaskDelay(1);
		xSemaphoreTake(mutex, portMAX_DELAY);
		size_t len = slave->read_bytes(rx, 64);

		printf("[%d]: %d, %d, %d\n", len, rx[0], rx[1], rx[2]);
		if (rx[0] == REGISTER_ADDRESS)
		{
		printf("write\n");
			data.tick = xTaskGetTickCount();
			slave->write_bytes(data.raw, sizeof(data), true);
		}
		xSemaphoreGive(mutex);
	}
	vSemaphoreDelete(mutex);
	vTaskDelete(NULL);
}

void app_main(void)
{
	IXS::wire_s master_conf = M5Stack_Internal;
	I2CMaster *master = new I2CMaster(&master_conf);

	IXS::wire_s slave_conf = M5Stack_Internal;
	slave_conf.i2cnum = I2C_NUM_1;
	I2CSlave *slave = new I2CSlave(&slave_conf, SLAVE_ADDRESS);

	xTaskCreatePinnedToCore(master_task, "master", 1024 * 8, master, 10, NULL, 0);
	xTaskCreatePinnedToCore(slave_task, "slave", 1024 * 8, slave, 10, NULL, 0);
}
