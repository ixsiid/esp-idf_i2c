#pragma once

#include <driver/i2c.h>
#include <esp_log.h>
#include <stdio.h>

namespace IXS
{
	struct wire_s
	{
		i2c_port_t i2cnum;
		gpio_num_t io_scl;
		gpio_num_t io_sda;
		uint32_t i2c_speed;
	};

	static const wire_s M5Atom_Internal = {
	    .i2cnum = (i2c_port_t)0,
	    .io_scl = (gpio_num_t)21,
	    .io_sda = (gpio_num_t)25,
	    .i2c_speed = 400000};

	static const wire_s M5Stick_Internal = {
	    .i2cnum = (i2c_port_t)0,
	    .io_scl = (gpio_num_t)22,
	    .io_sda = (gpio_num_t)21,
	    .i2c_speed = 400000};

	static const wire_s M5Stack_Internal = {
	    .i2cnum = (i2c_port_t)0,
	    .io_scl = (gpio_num_t)22,
	    .io_sda = (gpio_num_t)21,
	    .i2c_speed = 400000};

	static const wire_s M5Stick_Grove = {
	    .i2cnum = (i2c_port_t)1,
	    .io_scl = (gpio_num_t)33,
	    .io_sda = (gpio_num_t)32,
	    .i2c_speed = 400000};

	static const wire_s M5Atom_Grove = {
	    .i2cnum = (i2c_port_t)1,
	    .io_scl = (gpio_num_t)32,
	    .io_sda = (gpio_num_t)26,
	    .i2c_speed = 400000};

	static const size_t default_buffer_length = 512;
	static const TickType_t default_wait_tick = 1000 / portTICK_PERIOD_MS;

	class I2CMaster
	{
	public:
		I2CMaster(const wire_s *conf, TickType_t wait = default_wait_tick);
		I2CMaster(i2c_port_t port, gpio_num_t scl, gpio_num_t sda, uint32_t freq_hz, TickType_t wait);

		esp_err_t get_last_error();

		uint8_t read(uint8_t address, uint8_t registry);
		esp_err_t read_bytes(uint8_t address, uint8_t registry, uint8_t *buffer, size_t buffer_length);
		esp_err_t write(uint8_t address, uint8_t registry, uint8_t data);
		esp_err_t write_bytes(uint8_t address, uint8_t registry, uint8_t *data, size_t data_length);

	private:
		esp_err_t begin_transmission(uint8_t address, uint8_t registry);
		i2c_port_t port;
		esp_err_t last_error;
		TickType_t wait;
	};

	class I2CSlave
	{
	public:
		I2CSlave(const wire_s *conf, uint8_t slave_address, size_t buffer_length = default_buffer_length, TickType_t wait = default_wait_tick);
		I2CSlave(i2c_port_t port, gpio_num_t scl, gpio_num_t sda, uint8_t slave_address, size_t buffer_length = default_buffer_length, TickType_t wait = default_wait_tick);

		esp_err_t get_last_error();

		uint8_t read();
		size_t read_bytes(uint8_t *buffer, size_t buffer_length);
		esp_err_t write(uint8_t data, bool clear_buffer = false);
		esp_err_t write_bytes(uint8_t *data, size_t data_length, bool clear_buffer = false);

	private:
		i2c_port_t port;
		esp_err_t last_error;
		TickType_t wait;
	};

	inline esp_err_t I2CMaster::get_last_error() { return last_error; }
	inline esp_err_t I2CSlave::get_last_error() { return last_error; }

} // namespace IXS
