/*******************************************************************************
  System Definitions

  File Name:
    definitions.h

  Summary:
    project system definitions.

  Description:
    This file contains the system-wide prototypes and definitions for a project.

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "peripheral/nvmctrl/plib_nvmctrl.h"
#include "peripheral/evsys/plib_evsys.h"
#include "peripheral/port/plib_port.h"
#include "peripheral/clock/plib_clock.h"
#include "peripheral/nvic/plib_nvic.h"
#include "peripheral/sercom/spi_master/plib_sercom4_spi_master.h"
#include "peripheral/eic/plib_eic.h"
#include "peripheral/tc/plib_tc3.h"
#include "system/time/sys_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system/int/sys_int.h"
#include "osal/osal.h"
#include "system/debug/sys_debug.h"
#include "app.h"



// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

/* Device Information */
#define DEVICE_NAME			 "ATSAMR21G18A"
#define DEVICE_ARCH			 "CORTEX-M0PLUS"
#define DEVICE_FAMILY		 "SAMR"
#define DEVICE_SERIES		 "SAMR21"

/* CPU clock frequency */
#define CPU_CLOCK_FREQUENCY 48000000

// *****************************************************************************
// *****************************************************************************
// Section: System Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* System Initialization Function

  Function:
    void SYS_Initialize( void *data )

  Summary:
    Function that initializes all modules in the system.

  Description:
    This function initializes all modules in the system, including any drivers,
    services, middleware, and applications.

  Precondition:
    None.

  Parameters:
    data            - Pointer to the data structure containing any data
                      necessary to initialize the module. This pointer may
                      be null if no data is required and default initialization
                      is to be used.

  Returns:
    None.

  Example:
    <code>
    SYS_Initialize ( NULL );

    while ( true )
    {
        SYS_Tasks ( );
    }
    </code>

  Remarks:
    This function will only be called once, after system reset.
*/

void SYS_Initialize( void *data );

// *****************************************************************************
/* System Tasks Function

Function:
    void SYS_Tasks ( void );

Summary:
    Function that performs all polled system tasks.

Description:
    This function performs all polled system tasks by calling the state machine
    "tasks" functions for all polled modules in the system, including drivers,
    services, middleware and applications.

Precondition:
    The SYS_Initialize function must have been called and completed.

Parameters:
    None.

Returns:
    None.

Example:
    <code>
    SYS_Initialize ( NULL );

    while ( true )
    {
        SYS_Tasks ( );
    }
    </code>

Remarks:
    If the module is interrupt driven, the system will call this routine from
    an interrupt context.
*/

void SYS_Tasks ( void );

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* System Objects

Summary:
    Structure holding the system's object handles

Description:
    This structure contains the object handles for all objects in the
    MPLAB Harmony project's system configuration.

Remarks:
    These handles are returned from the "Initialize" functions for each module
    and must be passed into the "Tasks" function for each module.
*/

typedef struct
{
    SYS_MODULE_OBJ  sysTime;

} SYSTEM_OBJECTS;

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

#include "stack_config.h"
#include "driver/IEEE_802154_PHY/pal/inc/pal.h"
#include "driver/IEEE_802154_PHY/phy/inc/phy.h"
#include "driver/IEEE_802154_PHY/phy/inc/phy_tasks.h"
#include "driver/IEEE_802154_PHY/phy/at86rf/inc/phy_internal.h"
#include "framework_defs.h"
#if defined(PROTOCOL_MESH)
#include "driver/IEEE_802154_PHY/pal/inc/pal.h"
#include "stack_config.h"
#include "MiWi/MiWi_Mesh/inc/miwi_config.h"
#if defined(ENABLE_NETWORK_FREEZER)
#include "MiWi/MiWi_Mesh/Services/inc/wlPdsMemIds.h"
#include "MiWi/MiWi_Mesh/Services/inc/wlPdsTypesConverter.h"
#endif
#ifdef MESH_SECURITY
#include "MiWi/MiWi_Mesh/Services/SAL/inc/sal_generic.h"
#include "MiWi/MiWi_Mesh/Services/SAL/inc/sal.h"
#include "MiWi/MiWi_Mesh/Services/SAL/inc/sal_types.h"
#include "MiWi/MiWi_Mesh/Services/STB/inc/stb_generic.h"
#include "MiWi/MiWi_Mesh/Services/STB/inc/stb.h"
#endif
#include "MiWi/MiWi_Mesh/Services/inc/miwi_tmr.h"
#include "MiWi/MiWi_Mesh/inc/miwi_config_mesh.h"
#include "MiWi/MiWi_Mesh/inc/miwi_appconfig.h"
#include "MiWi/MiWi_Mesh/inc/miwi_api.h"
#include "MiWi/MiWi_Mesh/inc/miwi_app.h"
#include "MiWi/MiWi_Mesh/inc/miwi_init.h"
#include "MiWi/MiWi_Mesh/inc/mimac.h"
#include "MiWi/MiWi_Mesh/inc/miwi_mesh.h"
#include "MiWi/MiWi_Mesh/inc/miwi_mesh_frame.h"
#include "MiWi/MiWi_Mesh/inc/miwi_mesh_join.h"
#include "MiWi/MiWi_Mesh/inc/miwi_mesh_commissioning.h"
#include "MiWi/MiWi_Mesh/inc/miwi_mesh_routing.h"
#include "MiWi/MiWi_Mesh/inc/miwi_mesh_security.h"
#include "MiWi/MiWi_Mesh/inc/commands.h"
#if defined(OTAU_ENABLED)
#include "MiWi/MiWi_Mesh/Services/otau/otau.h"
#include "MiWi/MiWi_Mesh/Services/otau/otau_parser.h"
#include "MiWi/MiWi_Mesh/Services/otau/circularBuffer.h"
#include "MiWi/MiWi_Mesh/Services/otau/debug/otau_debug.h"
#include "MiWi/MiWi_Mesh/Services/otau/notify/otau_notify.h"
#include "MiWi/MiWi_Mesh/Services/otau/upgrade/otau_upgrade.h"
#ifdef OTAU_SERVER
#include "MiWi/MiWi_Mesh/Services/otau/debug/server_debug.h"
#include "MiWi/MiWi_Mesh/Services/otau/notify/server_notify.h"
#include "MiWi/MiWi_Mesh/Services/otau/upgrade/server_upgrade.h"
#else
#include "MiWi/MiWi_Mesh/Services/otau/debug/client_debug.h"
#include "MiWi/MiWi_Mesh/Services/otau/notify/client_notify.h"
#include "MiWi/MiWi_Mesh/Services/otau/upgrade/client_upgrade.h"
#endif
#endif
#if defined(LED_ENABLED)
#include "MiWi/MiWi_Mesh/Services/inc/led.h"
#endif
#endif
#if defined(PROTOCOL_P2P)
#include "driver/IEEE_802154_PHY/pal/inc/pal.h"
#include "stack_config.h"
#include "MiWi/MiWi_P2P/inc/miwi_config.h"
#if defined(ENABLE_NETWORK_FREEZER)
#include "MiWi/MiWi_P2P/Services/inc/wlPdsMemIds.h"
#include "MiWi/MiWi_P2P/Services/inc/wlPdsTypesConverter.h"
#endif
#ifdef ENABLE_SECURITY
#include "MiWi/MiWi_P2P/Services/SAL/inc/sal_generic.h"
#include "MiWi/MiWi_P2P/Services/SAL/inc/sal.h"
#include "MiWi/MiWi_P2P/Services/SAL/inc/sal_types.h"
#include "MiWi/MiWi_P2P/Services/STB/inc/stb_generic.h"
#include "MiWi/MiWi_P2P/Services/STB/inc/stb.h"
#endif
#include "MiWi/MiWi_P2P/Services/inc/miwi_tmr.h"
#include "MiWi/MiWi_P2P/inc/miwi_config_p2p.h"
#include "MiWi/MiWi_P2P/inc/miwi_app.h"
#include "MiWi/MiWi_P2P/inc/miwi_appconfig.h"
#include "MiWi/MiWi_P2P/inc/miwi_api.h"
#include "MiWi/MiWi_P2P/inc/miwi_init.h"
#include "MiWi/MiWi_P2P/inc/mimac.h"
#include "MiWi/MiWi_P2P/inc/miwi_p2p_star.h"
#include "MiWi/MiWi_P2P/inc/p2p_demo.h"
#if defined(OTAU_ENABLED)
#include "MiWi/MiWi_P2P/Services/otau/otau.h"
#include "MiWi/MiWi_P2P/Services/otau/otau_parser.h"
#include "MiWi/MiWi_P2P/Services/otau/circularBuffer.h"
#include "MiWi/MiWi_P2P/Services/otau/debug/otau_debug.h"
#include "MiWi/MiWi_P2P/Services/otau/notify/otau_notify.h"
#include "MiWi/MiWi_P2P/Services/otau/upgrade/otau_upgrade.h"
#ifdef OTAU_SERVER
#include "MiWi/MiWi_P2P/Services/otau/debug/server_debug.h"
#include "MiWi/MiWi_P2P/Services/otau/notify/server_notify.h"
#include "MiWi/MiWi_P2P/Services/otau/upgrade/server_upgrade.h"
#else
#include "MiWi/MiWi_P2P/Services/otau/debug/client_debug.h"
#include "MiWi/MiWi_P2P/Services/otau/notify/client_notify.h"
#include "MiWi/MiWi_P2P/Services/otau/upgrade/client_upgrade.h"
#endif
#endif
#if defined(LED_ENABLED)
#include "MiWi/MiWi_P2P/Services/inc/led.h"
#endif
#endif
#if defined(PROTOCOL_STAR)
#include "driver/IEEE_802154_PHY/pal/inc/pal.h"
#include "stack_config.h"
#include "MiWi/MiWi_Star/inc/miwi_config.h"
#if defined(ENABLE_NETWORK_FREEZER)
#include "MiWi/MiWi_Star/Services/inc/wlPdsMemIds.h"
#include "MiWi/MiWi_Star/Services/inc/wlPdsTypesConverter.h"
#endif
#ifdef ENABLE_SECURITY
#include "MiWi/MiWi_Star/Services/SAL/inc/sal_generic.h"
#include "MiWi/MiWi_Star/Services/SAL/inc/sal.h"
#include "MiWi/MiWi_Star/Services/SAL/inc/sal_types.h"
#include "MiWi/MiWi_Star/Services/STB/inc/stb_generic.h"
#include "MiWi/MiWi_Star/Services/STB/inc/stb.h"
#endif
#include "MiWi/MiWi_Star/Services/inc/miwi_tmr.h"
#include "MiWi/MiWi_Star/inc/miwi_config_p2p.h"
#include "MiWi/MiWi_Star/inc/miwi_app.h"
#include "MiWi/MiWi_Star/inc/miwi_appconfig.h"
#include "MiWi/MiWi_Star/inc/miwi_api.h"
#include "MiWi/MiWi_Star/inc/miwi_init.h"
#include "MiWi/MiWi_Star/inc/mimac.h"
#include "MiWi/MiWi_Star/inc/miwi_p2p_star.h"
#include "MiWi/MiWi_Star/inc/star_demo.h"
#if defined(OTAU_ENABLED)
#include "MiWi/MiWi_Star/Services/otau/otau.h"
#include "MiWi/MiWi_Star/Services/otau/otau_parser.h"
#include "MiWi/MiWi_Star/Services/otau/circularBuffer.h"
#include "MiWi/MiWi_Star/Services/otau/debug/otau_debug.h"
#include "MiWi/MiWi_Star/Services/otau/notify/otau_notify.h"
#include "MiWi/MiWi_Star/Services/otau/upgrade/otau_upgrade.h"
#ifdef OTAU_SERVER
#include "MiWi/MiWi_Star/Services/otau/debug/server_debug.h"
#include "MiWi/MiWi_Star/Services/otau/notify/server_notify.h"
#include "MiWi/MiWi_Star/Services/otau/upgrade/server_upgrade.h"
#else
#include "MiWi/MiWi_Star/Services/otau/debug/client_debug.h"
#include "MiWi/MiWi_Star/Services/otau/notify/client_notify.h"
#include "MiWi/MiWi_Star/Services/otau/upgrade/client_upgrade.h"
#endif
#endif
#if defined(LED_ENABLED)
#include "MiWi/MiWi_Star/Services/inc/led.h"
#endif
#endif


extern SYSTEM_OBJECTS sysObj;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* DEFINITIONS_H */
/*******************************************************************************
 End of File
*/

