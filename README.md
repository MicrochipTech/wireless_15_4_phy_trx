![Microchip logo](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_logo.png)
![Harmony logo small](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_mplab_harmony_logo_small.png)

# MPLAB® Harmony 3 Standalone IEEE 802.15.4 Physical Layer

MPLAB® Harmony 3 is an extension of the MPLAB® ecosystem for creating embedded firmware solutions for Microchip 32-bit SAM and PIC® microcontroller and microprocessor devices.  Refer to the following links for more information.

- [Microchip 32-bit MCUs](https://www.microchip.com/design-centers/32-bit)
- [Microchip 32-bit MPUs](https://www.microchip.com/design-centers/32-bit-mpus)
- [Microchip MPLAB X IDE](https://www.microchip.com/mplab/mplab-x-ide)
- [Microchip MPLAB® Harmony](https://www.microchip.com/mplab/mplab-harmony)
- [Microchip MPLAB® Harmony Pages](https://microchip-mplab-harmony.github.io/)

This repository contains the MPLAB® Harmony 3 Module for Standalone IEEE 802.15.4 Physical Layer that provide an interface to the microcontrollers (like SAML21, SAMD21) to access the transceiver functionality of radio transceivers like AT86RF233 or AT86RF212B. 
With the help of PHY layer module user can enable various functionaities of the transceiver. Refer to
the following links for release notes, training materials, and interface
reference information.


- [MPLAB® Harmony License](mplab_harmony_license.md)


# Contents Summary

| Folder     | Description                                                       |
| -----------| ------------------------------------------------------------------|
| config     | Standalone IEEE 802.15.4 Physical Layer module |
| drivers    | phy layer files     |

# Configuring IEEE 802.15.4 PHY Component in MPLAB Harmony:
- The IEEE 802.15.4 PHY component will be available under Wireless->Drivers->IEEE 802.15.4 in the Device Resources tab of MPLAB Harmony Window.
- The Pins must be configured under Plugins option available in MCC.
- The radio transceiver communication to host MCU happens via SPI. Hence the related SPI pins such as MOSI, MISO, SCK, SS must be configured as per the specific Host MCU.
- The other transceiver pins that need to be configured are IRQ, RST, SLP_TR, CLKM, DIG1, DIG2, DIG3, DIG4. (please refer respective device/transceiver user guide for the pin numbers).

____

____

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/user/MicrochipTechnology)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/microchip-technology)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/microchiptechnology/)
[![Follow us on Twitter](https://img.shields.io/twitter/follow/MicrochipTech.svg?style=social)](https://twitter.com/MicrochipTech)



