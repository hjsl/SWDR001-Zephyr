# SWDR001-Zephyr
IRNAS port of the Semtech SWDR001 LR11XX driver to Zephyr. Original package proposes an implementation in C of the driver for **LR11XX** radio component.

This repository is based on the following repositories from Semtech:

- [SWDR001](https://github.com/Lora-net/SWDR001) - LR11XX radio driver
- [SWSD003](https://github.com/Lora-net/SWSD003) - LR11XX radio samples

## Folder structure

Driver is located in the `drivers/radio` folder where `lr11xx_driver` contains Semtech SWDR001 LR11XX driver files and Zephyr compatible HAL implementation is contained in the `radio_drivers_hal` folder. Board specific interface functions and defined in the `lr11xx_board.h` and `lr11xx_board.c` files.

Compatible Device Tree Binding is contained in the `dts` folder. To use pre-defined macros for RF switches in DT binding, `lr11xx_bindings_def.h` needs to be included in your DT file.

The `samples` folder contains functional samples compatible with SWSD003 Semtech examples. Additional example for almanac update has also been added.

## Installation

This driver was written and tested for nrf-sdk v2.6.1

To install, modify your project's `west.yml` and add the following sections:

1. In `remotes`, add the following if not already added:

```yaml
 - name: irnas
   url-base: https://github.com/irnas
```

2. In the `projects` section add at the bottom (select revision you need):

```yaml
- name: SWDR001-Zephyr
      repo-path: SWDR001-Zephyr
      path: irnas/SWDR001-Zephyr
      remote: irnas
      revision: <release-tag | branch | commit hash>
```

Then run `west update` in your freshly created bash/command prompt session.

Above command will clone `SWDR001-Zephyr` repository inside of `ncs/irnas/`. You can now use the driver in your application projects.

## Usage

Compatible Device Tree binding for `lr11xx` needs to be added to DT file, for example:

```dts
&spi2 {
    cs-gpios = <&gpio1 8 GPIO_ACTIVE_LOW>;
    lr11xx: lr11xx@0 {
        compatible = "irnas,lr11xx";
        reg = <0>;
        spi-max-frequency = <4000000>;

        /* Pin configuration */
        reset-gpios = <&gpio0 6 GPIO_ACTIVE_LOW>;
        pwr-en-gpios = <&gpio0 0 GPIO_ACTIVE_HIGH>;
        busy-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;
        event-gpios = <&gpio0 30 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;

        /* TX path configuration */
        lf-tx-path = <LR11XX_TX_PATH_LF_HP>;

        /* TCXO configuration */
        tcxo-supply = <LR11XX_TCXO_SUPPLY_1_8V>;
        tcxo-wakeup-time = <3>;

        /* LF clock config */
        lf-clk = <LR11XX_LFCLK_EXT>;

        /* Regulator configuration */
        reg-mode = <LR11XX_REG_MODE_DCDC>;

        /* RF switch configuration */
        rf-sw-enable = <(LR11XX_DIO5 | LR11XX_DIO6 | LR11XX_DIO7 | LR11XX_DIO8)>;
        rf-sw-rx-mode = <(LR11XX_DIO6 | LR11XX_DIO8)>;
        rf-sw-tx-hp-mode = <(LR11XX_DIO5 | LR11XX_DIO6 | LR11XX_DIO8)>;
        rf-sw-wifi-mode = <(LR11XX_DIO5 | LR11XX_DIO8)>;
        rf-sw-gnss-mode = <(LR11XX_DIO7)>;
    };
};
```

LR11XX device structure can then be accessed trough device binding:

```c
const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(lr11xx));
```

## Development installation

This repository can also be used directly when developing this driver further.
Just initialize it as any west project:

```bash
mkdir SWDR001-Zephyr
cd SWDR001-Zephyr
west init -m https://github.com/IRNAS/SWDR001-Zephyr
west update
```

## SWDR001 LR11XX driver Components

NOTE: This section and everything bellow is a direct copy of the readme from SWDR001.

The driver is split in several components:

### Bootloader

This component is used to update the firmware.

### Register / memory access

This component is used to read / write data from registers or internal memory.

### System configuration

This component is used to interact with system-wide parameters like clock sources, integrated RF switches, etc.

### Radio

This component is used to send / receive data through the different modems (LoRa and GFSK) or perform a LoRa CAD (Channel Activity Detection). Parameters like power amplifier selection, output power and fallback modes are also accessible through this component.

### Wi-Fi Passive Scanning

This component is used to configure and initiate the passive scanning of the Wi-Fi signals that can be shared to request a geolocation.

### GNSS Scanning

This component is used to configure and initiate the acquisition of GNSS signals that can be shared to request a geolocation.

### Crypto engine

This component is used to set and derive keys in the internal keychain and perform cryptographic operations with the integrated hardware accelerator.

## Structure

Each component is based on different files:

- `lr11xx_component.c`: implementation of the functions related to component
- `lr11xx_component.h`: declarations of the functions related to component
- `lr11xx_component_types.h`: type definitions related to components

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions that the user shall implement to write platform-dependent calls to the host. The list of functions is the following:

- `lr11xx_hal_reset()`
- `lr11xx_hal_wakeup()`
- `lr11xx_hal_write()`
- `lr11xx_hal_read()`
- `lr11xx_hal_direct_read()`

The following driver contains Zephyr-compatible implementation.
