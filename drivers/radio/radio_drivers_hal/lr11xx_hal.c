
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include "lr11xx_hal.h"
#include "lr11xx_hal_context.h"

#define LR11XX_HAL_WAIT_ON_BUSY_TIMEOUT_SEC CONFIG_LR11XX_HAL_WAIT_ON_BUSY_TIMEOUT_SEC

typedef enum {
	RADIO_SLEEP,
	RADIO_AWAKE
} radio_mode_t;

static radio_mode_t radio_mode = RADIO_AWAKE;
static bool lr11xx_hal_busy_timeout = false;

/**
 * @brief timer handler, sets timeout to true when timer expires
 *
 */
static void lr11xx_hal_wait_on_busy_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	lr11xx_hal_busy_timeout = true;
}

K_TIMER_DEFINE(lr11xx_hal_wait_on_busy_timer, lr11xx_hal_wait_on_busy_timer_handler, NULL);

/**
 * @brief Wait until radio busy pin returns to inactive state or
 * until LR11XX_HAL_WAIT_ON_BUSY_TIMEOUT_SEC passes.
 *
 * @retval LR11XX_HAL_STATUS_OK
 */
static lr11xx_hal_status_t prv_lr11xx_hal_wait_on_busy(const struct gpio_dt_spec *busy_pin)
{
	lr11xx_hal_busy_timeout = false;
	k_timer_start(&lr11xx_hal_wait_on_busy_timer,
		      K_SECONDS(LR11XX_HAL_WAIT_ON_BUSY_TIMEOUT_SEC), K_NO_WAIT);

	while (gpio_pin_get_dt(busy_pin) && !lr11xx_hal_busy_timeout) {
		k_busy_wait(1000); /* 1 ms */
	}

	k_timer_stop(&lr11xx_hal_wait_on_busy_timer);

	if (lr11xx_hal_busy_timeout) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
}

/**
 * @brief Check if device is ready to receive spi transaction.
 *
 * If the device is in sleep mode, it will awake it and then wait until it is ready
 *
 */
static void prv_lr11xx_hal_check_device_ready(const struct device *dev)
{
	const struct lr11xx_hal_context_cfg_t *lr11xx_cfg = dev->config;

	if (radio_mode != RADIO_SLEEP) {
		prv_lr11xx_hal_wait_on_busy(&lr11xx_cfg->busy);
	} else {
		// Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
		const struct spi_cs_control *ctrl = &lr11xx_cfg->spi.config.cs;

		gpio_pin_set_dt(&ctrl->gpio, 1);
		gpio_pin_set_dt(&ctrl->gpio, 0);
		prv_lr11xx_hal_wait_on_busy(&lr11xx_cfg->busy);
		radio_mode = RADIO_AWAKE;
	}
}

/* PUBLIC API */

lr11xx_hal_status_t lr11xx_hal_write(const void *context, const uint8_t *command,
				     const uint16_t command_length, const uint8_t *data,
				     const uint16_t data_length)
{
#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
	// Compute the CRC over command array first and over data array then
	uint8_t cmd_crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
	cmd_crc = lr11xx_hal_compute_crc(cmd_crc, data, data_length);
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )

	const struct device *lr11xx_dev = (const struct device *)context;
	const struct lr11xx_hal_context_cfg_t *lr11xx_cfg = lr11xx_dev->config;

	int ret;

	prv_lr11xx_hal_check_device_ready(lr11xx_dev);
	const struct spi_buf tx_buf[] = {
		{
			.buf = (uint8_t *)command,
			.len = command_length,
		},
		{.buf = (uint8_t *)data, .len = data_length},
#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
		{.buf = &cmd_crc, .len = 1}
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	ret = spi_write_dt(&lr11xx_cfg->spi, &tx);
	if (ret) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	// LR11XX_SYSTEM_SET_SLEEP_OC=0x011B opcode. In sleep mode the radio busy line is held at 1
	// => do not test it
	if ((command[0] == 0x01) && (command[1] == 0x1B)) {
		radio_mode = RADIO_SLEEP;

		// add a incompressible delay to prevent trying to wake the radio before it is full
		// asleep
		k_busy_wait(500); /* 0.5 ms */
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void *context, uint8_t *data,
					   const uint16_t data_length)
{
	const struct device *lr11xx_dev = (const struct device *)context;
	const struct lr11xx_hal_context_cfg_t *lr11xx_cfg = lr11xx_dev->config;

	prv_lr11xx_hal_check_device_ready(lr11xx_dev);

	int ret;

#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
	uint8_t rx_crc;
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )

	const struct spi_buf rx_buf[] = {
		{.buf = data, .len = data_length},
#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
		// read crc sent by lr11xx at the end of the transaction
		{.buf = &rx_crc, .len = 1}
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	ret = spi_read_dt(&lr11xx_cfg->spi, &rx);
	if (ret) {
		return LR11XX_HAL_STATUS_ERROR;
	}

#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
	// check crc value
	uint8_t computed_crc = lr11xx_hal_compute_crc(0xFF, data, data_length);
	if (rx_crc != computed_crc) {
		return LR11XX_HAL_STATUS_ERROR;
	}
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read(const void *context, const uint8_t *command,
				    const uint16_t command_length, uint8_t *data,
				    const uint16_t data_length)
{
#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
	// Compute the CRC over command array first and over data array then
	uint8_t cmd_crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
#endif

	const struct device *lr11xx_dev = (const struct device *)context;
	const struct lr11xx_hal_context_cfg_t *lr11xx_cfg = lr11xx_dev->config;

	/* When hal_read is called by lr11xx_crypto_restore_from_flash during LoRa initialization,
	 * we sleep for 1 ms so we don't get stuck in an endless wait loop */
	if ((command[0] == 0x05) && (command[1] == 0x0B)) {
		k_busy_wait(1000); /* 1 ms */
	}
	prv_lr11xx_hal_check_device_ready(lr11xx_dev);
	int ret;

	const struct spi_buf tx_buf[] = {
		{
			.buf = (uint8_t *)command,
			.len = command_length,
		},
#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
		{.buf = &cmd_crc, .len = 1}
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	ret = spi_write_dt(&lr11xx_cfg->spi, &tx);
	if (ret) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (data_length > 0) {
		prv_lr11xx_hal_check_device_ready(lr11xx_dev);

		uint8_t dummy_byte;

		const struct spi_buf rx_buf[] = {
			// save dummy for crc calculation
			{
				.buf = &dummy_byte,
				.len = 1,
			},
			{.buf = data, .len = data_length},
#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
			// read crc sent by lr11xx at the end of the transaction
			{.buf = &cmd_crc, .len = 1}
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )
		};

		const struct spi_buf_set rx = {
			.buffers = rx_buf,
			.count = ARRAY_SIZE(rx_buf),
		};

		ret = spi_read_dt(&lr11xx_cfg->spi, &rx);
		if (ret) {
			return LR11XX_HAL_STATUS_ERROR;
		}

#if defined(CONFIG_LR11XX_USE_CRC_OVER_SPI)
		// Check CRC value
		uint8_t computed_crc = lr11xx_hal_compute_crc(0xFF, &dummy_byte, 1);
		computed_crc = lr11xx_hal_compute_crc(computed_crc, data, data_length);
		if (cmd_crc != computed_crc) {
			return LR11XX_HAL_STATUS_ERROR;
		}
#endif // defined( CONFIG_LR11XX_USE_CRC_OVER_SPI )
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset(const void *context)
{
	const struct device *lr11xx_dev = (const struct device *)context;
	const struct lr11xx_hal_context_cfg_t *lr11xx_cfg = lr11xx_dev->config;

	gpio_pin_set_dt(&lr11xx_cfg->reset, 1);
	k_busy_wait(1000); /* 1 ms */
	gpio_pin_set_dt(&lr11xx_cfg->reset, 0);
	k_busy_wait(1000); /* 1 ms */

	// Wait 200ms until internal lr11xx fw is ready
	k_busy_wait(200 * 1000); /* 200 ms */
	radio_mode = RADIO_AWAKE;

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
	const struct device *lr11xx_dev = (const struct device *)context;
	prv_lr11xx_hal_check_device_ready(lr11xx_dev);

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd(const void *context)
{
	const struct device *lr11xx_dev = (const struct device *)context;
	const struct lr11xx_hal_context_cfg_t *lr11xx_cfg = lr11xx_dev->config;

	int ret;
	uint8_t command[4] = { 0 };

	const struct spi_buf tx_buf[] = {
		{
			.buf = (uint8_t *)command,
			.len = sizeof(command),
		}
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	ret = spi_write_dt(&lr11xx_cfg->spi, &tx);
	if (ret) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	return LR11XX_HAL_STATUS_OK;
	//return prv_lr11xx_hal_wait_on_busy(&lr11xx_cfg->busy);
}
