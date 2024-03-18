#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/random/rand32.h>

#include "apps_common.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_board.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/**
 *  @brief Defines the Bluetooth channel used for beacon transmission.
 */
#define BLE_BEACON_CHANNEL 37u

/**
 *  @brief Defines the beacon transmission period [ms].
 */
#define BLE_BEACON_PERIOD_MS 2000u

/**
 *  @brief Defines the beacon transmission power [dBm].
 */
#define BLE_BEACON_TX_POWER 13

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK ( LR11XX_SYSTEM_IRQ_TX_DONE )

const struct device *context = DEVICE_DT_GET(DT_NODELABEL(lr11xx));

// /* Advertising Channel PDU (Eddystone-format Beacon) */
// const uint8_t pdu_buf[30] = {
//     0x02,                                     // Preamble (ADV_NONCONN_IND)
//     28,                                       // Length
//     0xa4, 0x63, 0xef, 0x8c, 0x89, 0xe6,       // ADV Address
//     0x02, 0x01, 0x06,                         // Advertising Data AD0
//     0x03, 0x03, 0xaa, 0xfe,                   // Advertising Data AD1
//     0x0e,                                     // Advertising Data AD2 - AD Length
//     0x16,                                     // Advertising Data AD2 - AD Type
//     0xaa, 0xfe,                               // Advertising Data AD2 - Eddystone UUID
//     0x10,                                     // Eddystone URL Frame - Frame Type
//     0x09,                                     // Eddystone URL Frame - TX Power
//     0x00,                                     // Eddystone URL Frame - URL Scheme
//     's',  'e',  'm',  't',  'e',  'c',  'h',  // Eddystone URL Frame - Encoded URL
//     0x07                                      // Eddystone URL Frame - Encoded URL (.com)
// };

static uint8_t pdu_buf[39] = {
    0x02,                                           /* Preamble (ADV_NONCONN_IND) */
    37,                                             /* Length (remainder) */
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06,             /* Advertising address */
    30,                                             /* Advertising length */
    0xff,                                           /* Advertising tag */
    0x4c, 0x00,                                     /* Company ID (Apple) */
    0x12, 0x19,                                     /* Offline Finding type and length */
    0x00,                                           /* Apple device */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Manufacturer data */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,                                           /* First two bits */
    0x00,                                           /* Hint (0x00) */
};   

/* mm4HWxe */
static char public_key[28] = {
    0xe4, 0x72, 0x8b, 0x6f, 0x40, 0xff, 0xb3, 0x37, 0x9b, 0x3b, 0x63, 0x91,
    0xaa, 0x5b, 0x7f, 0xa6, 0x2b, 0x7a, 0x89, 0xcd, 0x76, 0x10, 0x03, 0x02,
    0x51, 0xab, 0x7f, 0xaa
};

static void update_pdu(uint8_t pdu[39], char key[28])
{
    /* Set the address from the public key */
    pdu[2] = key[5];
    pdu[3] = key[4];
    pdu[4] = key[3];
    pdu[5] = key[2];
    pdu[6] = key[1];
    pdu[7] = key[0] | 0b11000000;

    /* Copy last 22 bytes of the public key */
    memcpy(&pdu[15], &key[6], 22);

    /* Append the first to bits of the public key */
    pdu[37] = key[0] >> 6;
}


static const uint8_t ble_beacon_channels[] = { 37, 38, 39 };

static void beacon_tx_work_handler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(beacon_tx_work, beacon_tx_work_handler);

K_SEM_DEFINE(beacon_tx_sem, 1, 1);

static void beacon_tx_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    // uint32_t start = k_uptime_get_32();
    for (int i = 0; i < ARRAY_SIZE(ble_beacon_channels); i++) {
        k_sem_take(&beacon_tx_sem, K_FOREVER);
        LOG_INF("Sending beacon on channel %d", ble_beacon_channels[i]);
        int err = lr11xx_radio_cfg_bluetooth_low_energy_beaconning_compatibility(context,
                ble_beacon_channels[i],
                pdu_buf,
                sizeof(pdu_buf));
        lr11xx_radio_set_tx(context, 0);
        LOG_INF("err: %d", err);
    }
    // uint32_t end = k_uptime_get_32();
    // uint32_t took = end - start;
    
    uint32_t adv_delay_ms = sys_rand32_get() % 10;

    k_work_reschedule(&beacon_tx_work, K_MSEC(BLE_BEACON_PERIOD_MS + adv_delay_ms));
}

void on_tx_done(void)
{
    LOG_INF("tx_done");
    k_sem_give(&beacon_tx_sem);
    // /* Switch to the next channel */
    // if (channel < (BLUETOOTH_LOW_ENERGY_BEACON_CHANNEL + 2)) {
    //     channel++;
    //     lr11xx_radio_cfg_bluetooth_low_energy_beaconning_compatibility(context,
    //             channel, pdu_buf, sizeof(pdu_buf));
    //     lr11xx_radio_set_tx(context, 0);
    // } else {
    //     channel = BLUETOOTH_LOW_ENERGY_BEACON_CHANNEL;
    //     k_sleep(K_MSEC(BLUETOOTH_LOW_ENERGY_BEACON_PERIOD_MS));
    //     lr11xx_radio_cfg_bluetooth_low_energy_beaconning_compatibility(context,
    //             channel, pdu_buf, sizeof(pdu_buf));
    //     lr11xx_radio_set_tx(context, 0);
    // }
}

static void radio_on_dio_irq(const struct device *dev)
{
	LOG_INF("done");

        lr11xx_system_irq_mask_t irq_regs;
        lr11xx_system_get_and_clear_irq_status( context, &irq_regs );

        LOG_INF( "Interrupt flags = 0x%08X", irq_regs );

        irq_regs &= IRQ_MASK;

        LOG_INF( "Interrupt flags (after filtering) = 0x%08X", irq_regs );

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_TX_DONE ) == LR11XX_SYSTEM_IRQ_TX_DONE )
        {
            LOG_INF( "Tx done" );
            on_tx_done( );
        }

}

void main(void)
{
    int ret = 0;

    LOG_INF( "===== LR11xx BLE beacon example =====" );

    apps_common_lr11xx_system_init( context );

    apps_common_lr11xx_fetch_and_print_version( context );

    /* Check version - expect LR1110 or LR1120 */
    lr11xx_system_version_t version = { 0 };
    lr11xx_system_get_version(context, &version);
    if ((version.type != 0x01) &&
        (version.type != 0x02)) {
        LOG_ERR("BLE beacon example application requires LR1110 or LR1120");
        return;
    }

    /* Initialize the PLL for the LR1110 */
    if (version.type == 0x01) {
        LOG_INF("Calibrating PLL for 2.4GHz");
        lr11xx_radio_set_rf_freq((void*) context, 2400000000);
        lr11xx_system_calibrate(context, 0x20);        
    }

    LOG_INF("Set dio irq mask");
    ret = lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 );
    if(ret) {
        LOG_ERR("Failed to set dio irq params.");
    }

    LOG_INF("Clear irq status");
    ret = lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );
    if(ret) {
        LOG_ERR("Failed to set dio irq params.");
    }

    /* Power the HF PA from VREG */
    LOG_INF("Set PA config");
    const lr11xx_radio_pa_cfg_t pa_config = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HF,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
        .pa_duty_cycle = 4u,
        .pa_hp_sel     = 0u,
    };
    lr11xx_radio_set_pa_cfg(context, &pa_config);

    /* Set the TX power */
    LOG_INF("Set TX power to %d", BLE_BEACON_TX_POWER);
    lr11xx_radio_set_tx_params(context, BLE_BEACON_TX_POWER, 
            LR11XX_RADIO_RAMP_16_US);

    update_pdu(pdu_buf, public_key);

    //apps_common_lr11xx_enable_irq(context);
    lr11xx_board_attach_interrupt(context, radio_on_dio_irq);
    lr11xx_board_enable_interrupt(context);

    k_work_schedule(&beacon_tx_work, K_NO_WAIT);


    // /* Send a BLE beacon */
    // lr11xx_radio_cfg_bluetooth_low_energy_beaconning_compatibility(context,
    //         channel, pdu_buf, sizeof(pdu_buf));
    // ret = lr11xx_radio_set_tx(context, 0);
    // if (ret) {
    //     LOG_ERR("Failed to set TX mode, ret: %d", ret);
    // }
    
    //while (true) {
    //     apps_common_lr11xx_irq_process(context, IRQ_MASK);
    //     k_sleep(K_MSEC(1));
    //}
}
