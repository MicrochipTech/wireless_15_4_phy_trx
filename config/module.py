"""*****************************************************************************
* Copyright (C) 2024 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*****************************************************************************"""


def loadModule():
    print('Load Module: Harmony IEEE 802.15.4 PHY Standalone Library')


    rfHostComponent  = Module.CreateComponent('IEEE_802154_PHY', 'IEEE 802.15.4 PHY', 'Wireless/Drivers/IEEE 802.15.4','driver/config/drv_ieee802154phy_standalone.py')
    rfHostComponent.setDisplayType('HOST<->TRANSCEIVER INTERFACE\n\n\n')
    rfHostComponent.addDependency('TRANSCEIVER_SPI_Dependency', 'SPI','SERCOM',False, True)
    rfHostComponent.addDependency('HarmonyCoreDependency', 'Core Service', 'Core', True, True)
    rfHostComponent.addDependency('SysTimeDependency', 'SYS_TIME', 'SYS_TIME', True, True)
    rfHostComponent.addDependency('FreeRtosDependency', 'RTOS', 'RTOS', True, True)
    rfHostComponent.addCapability('ieee802154phy_Capability', 'IEEE 802.15.4 PHY', True)



