#include <stdio.h>
#include <string.h>

#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_board.h"
#include "lr11xx_regmem.h"
#include "main_sigfox.h"
#include "smtc_dbpsk.h"

#include "sigfox_ep_api.h"
#include "sigfox_error.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"
#include "manuf/rf_api.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main_sigfox);

#define SIGFOX_PAYLOAD_LENGTH 26
#define IRQ_MASK LR11XX_SYSTEM_IRQ_TX_DONE

static void send_frame( uint8_t *payload, uint8_t payload_len );

static void uplink_cplt_cb(void)
{
    LOG_INF("uplink_cplt_cb");
}

// Global variables.
static volatile sfx_bool sigfox_process_flag = SFX_FALSE;
static volatile sfx_bool sigfox_message_completion_flag = SFX_FALSE;

static void sigfox_process_work_fn(struct k_work *work)
{
    LOG_INF("SIGFOX_process_callback");
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_process();
}

K_WORK_DEFINE(sigfox_process_work, sigfox_process_work_fn);

// Process callback.
void SIGFOX_process_callback(void) {
    k_work_submit(&sigfox_process_work);
}

// Message completion callback.
void SIGFOX_message_completion_callback(void) {
    sigfox_message_completion_flag = SFX_TRUE;
}

static void send_application_message(void)
{
    #define APP_UL_PAYLOAD_SIZE 9

    LOG_INF("send_application_message");
    SIGFOX_EP_API_status_t status;

    SIGFOX_EP_API_application_message_t application_message;
    SIGFOX_EP_API_message_status_t message_status;
    sfx_u8 app_ul_payload[APP_UL_PAYLOAD_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    sfx_u8 app_dl_payload[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm = 0;

    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.common_parameters.tx_power_dbm_eirp = 14;
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.t_ifu_ms = 500;
    application_message.common_parameters.ep_key_type = SIGFOX_EP_KEY_PRIVATE;
    application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
    application_message.ul_payload = (sfx_u8*) app_ul_payload;
    application_message.ul_payload_size_bytes = APP_UL_PAYLOAD_SIZE;
    application_message.t_conf_ms = 2000;
    application_message.uplink_cplt_cb = &uplink_cplt_cb;
    application_message.message_cplt_cb = &SIGFOX_message_completion_callback;

    application_message.bidirectional_flag = SFX_FALSE;

    status = SIGFOX_EP_API_send_application_message(&application_message);

    LOG_INF("status: %d", status);
}

int main( void )
{
    LOG_INF( "===== LR11xx Sigfox PHY example =====" );

    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_status_t sigfox_ep_api_status;
    SIGFOX_EP_API_message_status_t message_status;

    lib_config.rc = &SIGFOX_RC1;
    lib_config.message_counter_rollover = SIGFOX_MESSAGE_COUNTER_ROLLOVER_4096;
    lib_config.process_cb = &SIGFOX_process_callback;

    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);

    uint8_t *version;
    uint8_t version_size;
    sigfox_ep_api_status = SIGFOX_EP_API_get_version(SIGFOX_VERSION_EP_LIBRARY, &version, &version_size);
    LOG_INF("SIGFOX EP LIBARY version: %s", version);

    uint8_t ep_id[SIGFOX_EP_ID_SIZE_BYTES];
    uint8_t ep_id_size_bytes = SIGFOX_EP_ID_SIZE_BYTES;
    sigfox_ep_api_status = SIGFOX_EP_API_get_ep_id(ep_id, ep_id_size_bytes);
    LOG_HEXDUMP_INF(ep_id, ep_id_size_bytes, "ep_id:");

    uint8_t initial_pac[SIGFOX_EP_PAC_SIZE_BYTES];
    uint8_t initial_pac_size_bytes = SIGFOX_EP_PAC_SIZE_BYTES;
    sigfox_ep_api_status = SIGFOX_EP_API_get_initial_pac(initial_pac,
		   initial_pac_size_bytes);
    LOG_HEXDUMP_INF(initial_pac, initial_pac_size_bytes, "initial_pac:");

    send_application_message();
}

void on_tx_done( void )
{
    LOG_INF("Tx done");

    //send_frame();
}

void send_frame( uint8_t *payload, uint8_t payload_len )
{
    LOG_INF("send_frame");

    uint8_t frame_buffer[smtc_dbpsk_get_pld_len_in_bytes( payload_len << 3 )];
    

    smtc_dbpsk_encode_buffer( payload, payload_len << 3, &frame_buffer[0] );

    //lr11xx_regmem_write_buffer8( context, &frame_buffer[0], smtc_dbpsk_get_pld_len_in_bytes( payload_len << 3 ) );

    LOG_HEXDUMP_INF(&frame_buffer[0], smtc_dbpsk_get_pld_len_in_bytes( payload_len << 3 ), "frame_buffer:");

    //lr11xx_radio_set_tx(context, 0);
}
