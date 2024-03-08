![Microchip logo](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_logo.png)
![Harmony logo small](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_mplab_harmony_logo_small.png)

# Microchip Standalone IEEE 802.15.4 Physical Layer Release Notes


## Release v1.0.0

The physical layer contains the transceiver specific functionalities as mentioned as the requirements of IEEE 802.15.4 specification. It gives the interface to the MAC core layer which is independent of the underlying transceiver.
Besides that, the PHY layer provides the set of APIs which can be used to interface a basic application.
The following are the funcionalities of PHY layer

-	Frame Transmission  (including automatic frame retries)
-	Frame reception  (including automatic acknowledgement handling)
-	PHY PIB storage
-	CSMA module
-	Energy detection
-	Power management(Trx Sleep)
-	Interrupt handling
-	Initialization and Reset
-	Enabling High Datarate Support
-	Enabling Promiscuous Mode
-	Enabling Antenna Diversity
-	Enabling Reduced Power consumption modes (Only for RF233. Not supported by RF212B)


## Known Issues / Limitations

-	Deep Sleep feature is not implemented.

## Development Tools
-	MPLAB X v6.15
-	MPLAB® XC32 C/C++ Compiler v4.35
-	MPLAB® X IDE plug-ins: MPLAB® Code Configurator (MCC) v5.3.7 and above
-	Device Pack: SAML21_DFP (3.7.217)

## Notes
-	None


