#include "lr11xx_rf_api.h"
#include "lr11xx_system.h"
#include "lr11xx_board.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "smtc_dbpsk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr11xx_rf_api);

typedef struct {
    RF_API_process_cb_t process_cb;
    RF_API_error_cb_t error_cb;
    RF_API_tx_cplt_cb_t tx_cplt_cb;
    RF_API_rx_data_received_cb_t rx_data_received_cb;
} callback_t;

typedef struct {
    callback_t callbacks;
    sfx_bool tx_done_flag;
    sfx_bool rx_done_flag;
    sfx_bool error_flag;
    volatile sfx_bool irq_flag;
    volatile sfx_bool irq_en;
    sfx_u16 backup_bit_rate_bps_patch;
} lr11xx_ctx_t;

static const sfx_u8 LR11XX_RF_API_VERSION[] = "v1.2";

static lr11xx_ctx_t lr11xx_ctx = {
    .callbacks.process_cb = NULL,
    .callbacks.error_cb = NULL,
    .callbacks.tx_cplt_cb = NULL,
    .callbacks.rx_data_received_cb = NULL,
    .tx_done_flag   = 0,
    .rx_done_flag   = 0,
    .error_flag     = 0,
    .irq_flag = 0,
    .irq_en = SFX_FALSE,
    .backup_bit_rate_bps_patch = 0,
};

static void irq_handler(const struct device *dev)
{
    LOG_INF("irq");
    if (lr11xx_ctx.irq_en == 1) {
        lr11xx_ctx.irq_flag = 1;
        if (lr11xx_ctx.callbacks.process_cb != NULL) {
            lr11xx_ctx.callbacks.process_cb();
	}
    }
}

const struct device *device = DEVICE_DT_GET(DT_NODELABEL(lr11xx));

RF_API_status_t LR11XX_RF_API_open(RF_API_config_t *config) {
    RF_API_status_t status = RF_API_SUCCESS;

    /* Set callbacks. */
    lr11xx_ctx.callbacks.process_cb = config->process_cb;
    lr11xx_ctx.callbacks.error_cb = config->error_cb;

    //apps_common_lr11xx_system_init( ( void* ) context );
    //apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    //apps_common_lr11xx_radio_dbpsk_init( ( void* ) context, SIGFOX_PAYLOAD_LENGTH );

    // int err = lr11xx_system_set_dio_irq_params(device, LR11XX_SYSTEM_IRQ_TX_DONE, 0);
    // if (err) {
    //     LOG_ERR("Failed to set dio irq params, err: %d", err);
    //     return err;
    // }

    // err = lr11xx_system_clear_irq_status(device, LR11XX_SYSTEM_IRQ_ALL_MASK);
    // if (err) {
    //     LOG_ERR("Failed to clear irq status, err: %d", err);
    //     return err;
    // }

    /* Enable irq handler. */
    lr11xx_board_attach_interrupt(device, &irq_handler);
    lr11xx_board_enable_interrupt(device);

    return status;
}

RF_API_status_t LR11XX_RF_API_process(void)
{
    LOG_INF("LR11XX_RF_API_process");
    RF_API_status_t status = RF_API_SUCCESS;
    //LR11XX_HW_API_status_t lr11xx_hw_api_status = LR11XX_HW_API_SUCCESS;

    lr11xx_system_irq_mask_t lr11xx_system_irq_mask;
    lr11xx_status_t lr11xx_status;
    if (lr11xx_ctx.irq_flag != 1) {
        status = LR11XX_RF_API_ERROR_STATE;
    	goto errors;
    }
    lr11xx_ctx.irq_flag = 0;

    lr11xx_status = lr11xx_system_get_and_clear_irq_status(device, &lr11xx_system_irq_mask);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        status = LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
        goto errors;
    }

    if (lr11xx_system_irq_mask & LR11XX_SYSTEM_IRQ_TX_DONE) {
        LOG_INF("tx_done");
        // lr11xx_hw_api_status = LR11XX_HW_API_tx_off();
        lr11xx_ctx.tx_done_flag = 1;
        if (lr11xx_ctx.callbacks.tx_cplt_cb) {
            LOG_INF("tx_cplt_cb()");
            lr11xx_ctx.callbacks.tx_cplt_cb();
	}
    }

    if (lr11xx_system_irq_mask & LR11XX_SYSTEM_IRQ_RX_DONE) {
        LOG_INF("rx_done");
        //lr11xx_hw_api_status = LR11XX_HW_API_rx_off();
        lr11xx_ctx.rx_done_flag = 1;
        if (lr11xx_ctx.callbacks.rx_data_received_cb)
            lr11xx_ctx.callbacks.rx_data_received_cb();
    }

    if (lr11xx_system_irq_mask & LR11XX_SYSTEM_IRQ_ERROR) {
        LOG_ERR("irq error");
        lr11xx_ctx.error_flag = 1;
        if (lr11xx_ctx.callbacks.error_cb) {
            lr11xx_ctx.callbacks.error_cb(LR11XX_RF_API_ERROR_CHIP_IRQ);
	}
    }

    return status;
    
errors:
    lr11xx_ctx.callbacks.error_cb(RF_API_ERROR);

    return status;
}

RF_API_status_t LR11XX_RF_API_close(void)
{
    RF_API_status_t status = RF_API_SUCCESS;
    //LR11XX_HW_API_status_t lr11xx_hw_api_status = LR11XX_HW_API_SUCCESS;

    //lr11xx_hw_api_status = LR11XX_HW_API_close();
    return status;
}

RF_API_status_t LR11XX_RF_API_wake_up(void)
{
    RF_API_status_t status = RF_API_SUCCESS;
    lr11xx_status_t lr11xx_status;
    lr11xx_system_errors_t errors;
    lr11xx_system_version_t version;
    const struct lr11xx_hal_context_cfg_t *config = device->config;

    LOG_INF("Reset system.");
    lr11xx_status = lr11xx_system_reset(device);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("System reset failed.");
	    return LR11XX_RF_API_ERROR_CHIP_RESET;
    }

    const lr11xx_system_reg_mode_t regulator = config->reg_mode;
    lr11xx_status = lr11xx_system_set_reg_mode(device, regulator);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to config regulator.");
	    return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    const lr11xx_system_rfswitch_cfg_t rf_switch_setup = config->rf_switch_cfg;
    lr11xx_status = lr11xx_system_set_dio_as_rf_switch(device, &rf_switch_setup);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to config rf switch.");
	    return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    const struct lr11xx_hal_context_tcxo_cfg_t tcxo_cfg = config->tcxo_cfg;
    if (tcxo_cfg.has_tcxo) {
        const uint32_t timeout_rtc_step = lr11xx_radio_convert_time_in_ms_to_rtc_step(tcxo_cfg.timeout_ms );
        lr11xx_status = lr11xx_system_set_tcxo_mode(device, tcxo_cfg.supply, timeout_rtc_step);
        if (lr11xx_status != LR11XX_STATUS_OK) {
            LOG_ERR("Failed to configure TCXO.");
	        return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
        }
    }

    const struct lr11xx_hal_context_lf_clck_cfg_t lf_clk_cfg = config->lf_clck_cfg;
    lr11xx_status = lr11xx_system_cfg_lfclk(device, lf_clk_cfg.lf_clk_cfg, lf_clk_cfg.wait_32k_ready );
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to configure Configure the Low Frequency Clock.");
	    return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_status = lr11xx_system_clear_errors(device);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to clear errors.");
	    return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_status = lr11xx_system_calibrate(device, 0x3F );
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to calibrate.");
    	return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_status = lr11xx_system_set_standby(device, LR11XX_SYSTEM_STANDBY_CFG_XOSC);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to set standby.");
        return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_status = lr11xx_system_set_dio_irq_params(device, 
	        LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_ERROR, 0);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set dio irq params");
        return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_status = lr11xx_system_clear_irq_status(device, LR11XX_SYSTEM_IRQ_ALL_MASK);
    if (lr11xx_status) {
        LOG_ERR("Failed to clear irq status.");
    	return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_status = lr11xx_system_get_version(device, &version);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to get version");
        return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_status = lr11xx_system_get_errors(device, &errors);
    if ((lr11xx_status != LR11XX_STATUS_OK) || (errors != 0)) {
        LOG_ERR("Failed to get errors, or errors.");
	    return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    lr11xx_ctx.irq_en = 1;

    return status;
}

RF_API_status_t LR11XX_RF_API_sleep(void) {
    RF_API_status_t status = RF_API_SUCCESS;
    lr11xx_status_t lr11xx_status;
    lr11xx_system_sleep_cfg_t lr11xx_system_sleep_cfg;

    lr11xx_ctx.irq_en = 0;
    lr11xx_system_sleep_cfg.is_warm_start = 0;
    lr11xx_system_sleep_cfg.is_rtc_timeout= 0;

    lr11xx_status = lr11xx_system_set_sleep(device, lr11xx_system_sleep_cfg, 0);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set sleep.");
        return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    return status;
}

RF_API_status_t LR11XX_RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
    RF_API_status_t status = RF_API_SUCCESS;

    lr11xx_radio_mod_params_bpsk_t lr11xx_radio_mod_params_bpsk;
    lr11xx_radio_pa_cfg_t lr11xx_radio_pa_cfg;
    lr11xx_radio_mod_params_gfsk_t lr11xx_radio_mod_params_gfsk;
    lr11xx_status_t lr11xx_status;
    lr11xx_status = lr11xx_radio_set_rf_freq(device, radio_parameters->frequency_hz);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }

    switch (radio_parameters->modulation) {
        case RF_API_MODULATION_DBPSK:
            lr11xx_status = lr11xx_radio_set_pkt_type(device, LR11XX_RADIO_PKT_TYPE_BPSK);
            if ( lr11xx_status != LR11XX_STATUS_OK) {
		        LOG_ERR("Failed to set bpsk packet type");
                return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
	        }
            lr11xx_radio_mod_params_bpsk.br_in_bps = radio_parameters->bit_rate_bps;
            lr11xx_radio_mod_params_bpsk.pulse_shape = LR11XX_RADIO_DBPSK_PULSE_SHAPE;
            lr11xx_status = lr11xx_radio_set_bpsk_mod_params(device, &lr11xx_radio_mod_params_bpsk);
            if (lr11xx_status != LR11XX_STATUS_OK) {
		        LOG_ERR("Failed to set bpsk modulation params");
                return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
	        }
            break;
        case RF_API_MODULATION_GFSK:
            lr11xx_status = lr11xx_radio_set_pkt_type(device, LR11XX_RADIO_PKT_TYPE_GFSK);
            if ( lr11xx_status != LR11XX_STATUS_OK) {
		        LOG_ERR("Failed to set gfsk packet type.");
                return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
	        }
            lr11xx_radio_mod_params_gfsk.br_in_bps = radio_parameters->bit_rate_bps;
            lr11xx_radio_mod_params_gfsk.fdev_in_hz = radio_parameters->deviation_hz;
            lr11xx_radio_mod_params_gfsk.pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1;
            lr11xx_radio_mod_params_gfsk.bw_dsb_param = LR11XX_RADIO_GFSK_BW_4800;
            lr11xx_status = lr11xx_radio_set_gfsk_mod_params(device, &lr11xx_radio_mod_params_gfsk);
            if ( lr11xx_status != LR11XX_STATUS_OK) {
		    LOG_ERR("Faied to set gfsk modulation params.");
                return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
	        }
            break;
        case RF_API_MODULATION_NONE :
            lr11xx_status = lr11xx_radio_set_pkt_type(device, LR11XX_RADIO_PKT_TYPE_RANGING);
            if (lr11xx_status != LR11XX_STATUS_OK) {
		        LOG_ERR("Failed to set ranging packet type");
                return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
	        }
            break;
        default:
	    LOG_ERR("Unknown modulation.");
            return LR11XX_RF_API_ERROR_MODULATION;
    }

    if (radio_parameters->rf_mode == RF_API_MODE_TX) {
        lr11xx_ctx.backup_bit_rate_bps_patch = radio_parameters->bit_rate_bps;
        if (radio_parameters->tx_power_dbm_eirp > 14 ) {
            lr11xx_radio_pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT;
            lr11xx_radio_pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_HP;
            lr11xx_radio_pa_cfg.pa_hp_sel = 0x07;
        } else {
            lr11xx_radio_pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
            lr11xx_radio_pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_LP;
            lr11xx_radio_pa_cfg.pa_hp_sel = 0x00;
        }
        lr11xx_radio_pa_cfg.pa_duty_cycle = 0x04;

        lr11xx_status = lr11xx_radio_set_pa_cfg(device ,&lr11xx_radio_pa_cfg);
        if (lr11xx_status != LR11XX_STATUS_OK) {
	        LOG_ERR("Failed to set pa cfg.");
            return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
	}

        lr11xx_status = lr11xx_radio_set_tx_params(device, radio_parameters->tx_power_dbm_eirp, LR11XX_RADIO_RAMP_208_US);
        if ( lr11xx_status != LR11XX_STATUS_OK) {
	        LOG_ERR("Failed to set tx params.");
            return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
	}

    }
    if (radio_parameters->rf_mode == RF_API_MODE_RX) {

    }

    return status;
}

RF_API_status_t LR11XX_RF_API_de_init(void) {
    RF_API_status_t status = RF_API_SUCCESS;
    lr11xx_status_t lr11xx_status;

    lr11xx_status = lr11xx_system_set_standby(device, LR11XX_SYSTEM_STANDBY_CFG_XOSC);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set standby.");
        return LR11XX_RF_API_ERROR_CHIP_SYSTEM_REG;
    }

    //lr11xx_hw_api_status = LR11XX_HW_API_rx_off();
    //lr11xx_hw_api_status = LR11XX_HW_API_tx_off();
    
    return status;
}

RF_API_status_t LR11XX_RF_API_send(RF_API_tx_data_t *tx_data) {
    RF_API_status_t status = RF_API_SUCCESS;
    lr11xx_radio_pkt_params_bpsk_t lr11xx_radio_pkt_params_bpsk;
    lr11xx_status_t lr11xx_status;
    sfx_u8 buffer[SIGFOX_UL_BITSTREAM_SIZE_BYTES + 1];
    sfx_u8 i;
    lr11xx_ctx.callbacks.tx_cplt_cb = tx_data->cplt_cb;
    lr11xx_ctx.tx_done_flag = 0;
    lr11xx_ctx.error_flag = 0;
    for (i = 0; i < tx_data->bitstream_size_bytes; i++) {
        buffer[i] = tx_data->bitstream[i];
    }
    buffer[tx_data->bitstream_size_bytes] = 0x80;
    smtc_dbpsk_encode_buffer(buffer, tx_data->bitstream_size_bytes * 8 + 2, buffer);

    /* Set the BPSK packet params. */
    lr11xx_radio_pkt_params_bpsk.pld_len_in_bits = smtc_dbpsk_get_pld_len_in_bits(tx_data->bitstream_size_bytes * 8);
    lr11xx_radio_pkt_params_bpsk.pld_len_in_bytes = smtc_dbpsk_get_pld_len_in_bytes(tx_data->bitstream_size_bytes * 8);
    if (lr11xx_ctx.backup_bit_rate_bps_patch == 100) {
        lr11xx_radio_pkt_params_bpsk.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS;
        lr11xx_radio_pkt_params_bpsk.ramp_up_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS;
    } else if (lr11xx_ctx.backup_bit_rate_bps_patch == 600) {
        lr11xx_radio_pkt_params_bpsk.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS;
        lr11xx_radio_pkt_params_bpsk.ramp_up_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS;
    }

    lr11xx_status = lr11xx_radio_set_bpsk_pkt_params(device, &lr11xx_radio_pkt_params_bpsk);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set bpsk packet params.");
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }

    lr11xx_status = lr11xx_regmem_write_buffer8(device, buffer, lr11xx_radio_pkt_params_bpsk.pld_len_in_bytes);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to write regmem.");
        return LR11XX_RF_API_ERROR_CHIP_REGMEM_REG;
    }

    //lr11xx_hw_api_status = LR11XX_HW_API_tx_on();
    lr11xx_status = lr11xx_radio_set_tx(device, 5000);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set tx.");
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }

    return status;
}

RF_API_status_t LR11XX_RF_API_receive(RF_API_rx_data_t *rx_data) {
    LOG_INF("LR11XX_RF_API_receive");
    RF_API_status_t status = RF_API_SUCCESS;
    //LR11XX_HW_API_status_t lr11xx_hw_api_status = LR11XX_HW_API_SUCCESS;
    lr11xx_status_t lr11xx_status;
    lr11xx_radio_pkt_params_gfsk_t lr11xx_radio_pkt_params_gfsk;
    sfx_u8 sync_word_short[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
    sfx_u8 sync_word[LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH];
    memcpy(sync_word, sync_word_short, SIGFOX_DL_FT_SIZE_BYTES);
    lr11xx_ctx.callbacks.rx_data_received_cb = rx_data->data_received_cb;
    lr11xx_ctx.rx_done_flag = 0;
    lr11xx_ctx.error_flag = 0;
    lr11xx_radio_pkt_params_gfsk.address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE;
    lr11xx_radio_pkt_params_gfsk.crc_type = LR11XX_RADIO_GFSK_CRC_OFF;
    lr11xx_radio_pkt_params_gfsk.dc_free = LR11XX_RADIO_GFSK_DC_FREE_OFF;
    lr11xx_radio_pkt_params_gfsk.header_type = LR11XX_RADIO_GFSK_PKT_FIX_LEN;
    lr11xx_radio_pkt_params_gfsk.pld_len_in_bytes = SIGFOX_DL_PHY_CONTENT_SIZE_BYTES;
    lr11xx_radio_pkt_params_gfsk.preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    lr11xx_radio_pkt_params_gfsk.preamble_len_in_bits = 16;
    lr11xx_radio_pkt_params_gfsk.sync_word_len_in_bits = SIGFOX_DL_FT_SIZE_BYTES * 8;

    lr11xx_status = lr11xx_radio_set_gfsk_pkt_params(device, &lr11xx_radio_pkt_params_gfsk);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set gfsk packet params.");
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }

    lr11xx_status = lr11xx_radio_set_gfsk_sync_word(device, sync_word);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set gfsk sync word.");
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }

    lr11xx_status = lr11xx_radio_cfg_rx_boosted(device, 0x01);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set cfg rx boosted.");
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }

    //lr11xx_hw_api_status = LR11XX_HW_API_rx_on();
    
    lr11xx_status = lr11xx_radio_set_rx_with_timeout_in_rtc_step(device, 0xFFFFFF);
    if (lr11xx_status != LR11XX_STATUS_OK) {
	    LOG_ERR("Failed to set rx with timeout.");
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }

    return status;
}

RF_API_status_t LR11XX_RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm)
{
    RF_API_status_t status = RF_API_SUCCESS;
    lr11xx_status_t lr11xx_status;
    lr11xx_radio_pkt_status_gfsk_t lr11xx_radio_pkt_status_gfsk;
    lr11xx_radio_rx_buffer_status_t lr11xx_radio_rx_buffer_status;

    // Check parameters.
    if ((dl_phy_content == SFX_NULL) || (dl_rssi_dbm == SFX_NULL)) {
        LOG_ERR("Null parameter.");
        return LR11XX_RF_API_ERROR_NULL_PARAMETER;
    }
    if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
        LOG_ERR("Buffer size.");
        return LR11XX_RF_API_ERROR_BUFFER_SIZE;
    }

    if (lr11xx_ctx.rx_done_flag != SFX_TRUE) {
        LOG_ERR("API state.");
        return LR11XX_RF_API_ERROR_STATE;
    }

    lr11xx_status = lr11xx_radio_get_gfsk_pkt_status(device, &lr11xx_radio_pkt_status_gfsk);
    if (lr11xx_status != LR11XX_STATUS_OK) {
        LOG_ERR("Failed to get gfsk packet status");
        return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
    }
    if (lr11xx_radio_pkt_status_gfsk.is_received == 1) {
        *dl_rssi_dbm = (sfx_s16)lr11xx_radio_pkt_status_gfsk.rssi_avg_in_dbm;
        lr11xx_status = lr11xx_radio_get_rx_buffer_status(device, &lr11xx_radio_rx_buffer_status);
        if (lr11xx_status != LR11XX_STATUS_OK) {
            LOG_ERR("Failed to get rx buffer status");
            return LR11XX_RF_API_ERROR_CHIP_RADIO_REG;
        }
        lr11xx_status = lr11xx_regmem_read_buffer8(device, dl_phy_content, lr11xx_radio_rx_buffer_status.buffer_start_pointer,
                dl_phy_content_size);
        if (lr11xx_status != LR11XX_STATUS_OK) {
            LOG_ERR("Failed to read buffer");
            return LR11XX_RF_API_ERROR_CHIP_REGMEM_REG;
        }
        lr11xx_status = lr11xx_regmem_clear_rxbuffer(device);
        if (lr11xx_status != LR11XX_STATUS_OK) {
            LOG_ERR("Failed to clear rxbuffer");
            return LR11XX_RF_API_ERROR_CHIP_REGMEM_REG;
        }
    }

    return status;
}

RF_API_status_t LR11XX_RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char)
{
    RF_API_status_t status = RF_API_SUCCESS;

    (*version) = (sfx_u8*) LR11XX_RF_API_VERSION;
    (*version_size_char) = (sfx_u8) sizeof(LR11XX_RF_API_VERSION);
    
    return status;
}
        
void LR11XX_RF_API_error(void) {
    lr11xx_regmem_clear_rxbuffer(device);
    LR11XX_RF_API_de_init();
    LR11XX_RF_API_sleep();
}

inline RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config) {
        return LR11XX_RF_API_open(rf_api_config);
}

inline RF_API_status_t RF_API_close(void) {
        return LR11XX_RF_API_close();
}

inline RF_API_status_t RF_API_process(void) {
        return LR11XX_RF_API_process();
}
    
inline RF_API_status_t RF_API_wake_up(void) {
        return LR11XX_RF_API_wake_up();
}   

inline RF_API_status_t RF_API_sleep(void) {
        return LR11XX_RF_API_sleep();
}

inline RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
        return LR11XX_RF_API_init(radio_parameters);
}

inline RF_API_status_t RF_API_de_init(void) {
        return LR11XX_RF_API_de_init();
}

inline RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data) {
        return LR11XX_RF_API_send(tx_data);
}

inline RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data) {
        return LR11XX_RF_API_receive(rx_data);
}

inline RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
        return LR11XX_RF_API_get_dl_phy_content_and_rssi(dl_phy_content, dl_phy_content_size, dl_rssi_dbm);
}

inline RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
        return LR11XX_RF_API_get_version(version, version_size_char);
}

inline void RF_API_error(void) {
        LR11XX_RF_API_error();
}
