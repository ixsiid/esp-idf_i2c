#include "i2c.h"

namespace IXS {
I2CSlave::I2CSlave(const wire_s* wire, uint8_t slave_address, size_t buffer_length, TickType_t wait) : I2CSlave(wire->i2cnum, wire->io_scl, wire->io_sda, slave_address, buffer_length, wait) {}
I2CSlave::I2CSlave(i2c_port_t port, gpio_num_t scl, gpio_num_t sda, uint8_t slave_address, size_t buffer_length, TickType_t wait) {
	this->port = port;
	this->wait = wait;
	
	i2c_config_t conf;
	conf.mode		    = I2C_MODE_SLAVE;
	conf.sda_io_num    = sda;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num    = scl;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

	conf.slave.addr_10bit_en = 0;
	conf.slave.slave_addr	= slave_address;

	last_error = i2c_param_config(port, &conf);
	if (last_error == ESP_OK) {
		last_error = i2c_driver_install(port, conf.mode,
								  buffer_length,
								  buffer_length,
								  0);
	}
}

size_t I2CSlave::read_bytes(uint8_t* buffer, size_t buffer_length) {
	return i2c_slave_read_buffer(port, buffer, buffer_length, wait);
}

uint8_t I2CSlave::read() {
	uint8_t data = 0;
	read_bytes(&data, 1);
	i2c_reset_rx_fifo(port);
	return data;
}

esp_err_t I2CSlave::write(uint8_t data, bool clear_buffer) {
	if (clear_buffer) i2c_reset_tx_fifo(port);
	return i2c_slave_write_buffer(port, &data, 1, wait);
}

esp_err_t I2CSlave::write_bytes(uint8_t* data, size_t data_length, bool clear_buffer) {
	if (clear_buffer) i2c_reset_tx_fifo(port);
	return i2c_slave_write_buffer(port, data, data_length, wait);
}

}  // namespace IXS
