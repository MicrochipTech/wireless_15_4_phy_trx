// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2023 Microchip Technology Inc. and its subsidiaries.
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
// DOM-IGNORE-END

/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <string.h>
#include "app.h"
#include "definitions.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
uint16_t register_addresses[] = {
     0x00,    0x01,   0x02,  0x03,   0x04,   0x05,   0x06,   0x07,   0x08,   0x09,   
     0x0A,    0x0B,   0x0C,  0x0D,   0x0E,   0x0100, 0x0101, 0x0102, 0x0103, 0x0104, 
     0x0105, 0x0106, 0x0107, 0x0108, 0x0109, 0x010A, 0x010B, 0x010C, 0x010D, 0x010E, 
     0x010F, 0x0110, 0x0111, 0x0112, 0x0113, 0x0114, 0x0118, 0x0121, 0x0122, 0x0125, 
     0x0126, 0x0127, 0x0128, 0x0200, 0x0201, 0x0202, 0x0203, 0x0204, 0x0205, 0x0206, 
     0x0207, 0x0208, 0x0209, 0x020A, 0x020B, 0x020C, 0x020D, 0x020E, 0x020F, 0x0210, 
     0x0211, 0x0212, 0x0213, 0x0214, 0x0216, 0x0221, 0x0222, 0x0223, 0x0224, 0x0225, 
     0x0226, 0x0227, 0x0228, 0x0300, 0x0301, 0x0302, 0x0303, 0x0304, 0x0305, 0x0306, 
     0x0307, 0x0308, 0x0309, 0x030A, 0x030B, 0x030C, 0x030D, 0x030E, 0x030F, 0x0310, 
     0x0311, 0x0312, 0x0313, 0x0314, 0x0315, 0x0320, 0x0321, 0x0322, 0x0323, 0x0324,
     0x0325, 0x0326, 0x0327, 0x0328, 0x0329, 0x032A, 0x032B, 0x032C, 0x032D, 0x032E, 
     0x032F, 0x0330, 0x0331, 0x0332, 0x0333, 0x0334, 0x0335, 0x0336, 0x0337, 0x0338, 
     0x0339, 0x033A, 0x033B, 0x033C, 0x0340, 0x0341, 0x0342, 0x0343, 0x0344, 0x0360, 
     0x0361, 0x0362, 0x0363, 0x0364, 0x0365, 0x0366, 0x0367, 0x0368, 0x0369, 0x036A, 
     0x036B, 0x036C, 0x036D, 0x036E, 0x0370, 0x0371, 0x0372, 0x0373, 0x0374, 0x0375, 
     0x0380, 0x0381, 0x0382, 0x0383, 0x0384, 0x0390, 0x0391, 0x0392, 0x0393, 0x0394, 
     0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0406, 0x0407, 0x0408, 0x0409, 
     0x040A, 0x040B, 0x040C, 0x040D, 0x040E, 0x040F, 0x0410, 0x0411, 0x0412, 0x0413, 
     0x0414, 0x0415, 0x0420, 0x0421, 0x0422, 0x0423, 0x0424, 0x0425, 0x0426, 0x0427, 
     0x0428, 0x0429, 0x042A, 0x042B, 0x042C, 0x042D, 0x042E, 0x042F, 0x0430, 0x0431, 
     0x0432, 0x0433, 0x0434, 0x0435, 0x0436, 0x0437, 0x0438, 0x0439, 0x043A, 0x043B, 
     0x043C, 0x0440, 0x0441, 0x0442, 0x0443, 0x0444, 0x0460, 0x0461, 0x0462, 0x0463, 
     0x0464, 0x0465, 0x0466, 0x0467, 0x0468, 0x0469, 0x046A, 0x046B, 0x046C, 0x046D, 
     0x046E, 0x0470, 0x0471, 0x0472, 0x0473, 0x0474, 0x0475, 0x0480, 0x0481, 0x0482, 
     0x0483, 0x0484, 0x0490, 0x0491, 0x0492, 0x0493, 0x0494, 0x2000, 0x27FE, 0x2800, 
     0x2FFE, 0x3000, 0x37FE, 0x3800, 0x3FFE
};


/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;


    appData.appQueue = xQueueCreate( 5, sizeof(APP_Msg_T) );
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    APP_Msg_T    appMsg[1];
    APP_Msg_T *p_appMsg;
    p_appMsg = appMsg;

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            //appData.appQueue = xQueueCreate( 10, sizeof(APP_Msg_T) );
#if defined(RF215v3)
             SYS_CONSOLE_MESSAGE("\r\n=====P2P PHY APP INIT======\r\n");
            bool temp, temp1 = false;
            PHY_ConfigTrxId(RF24);
            temp = app_P2P_Phy_Init();
            PHY_ConfigTrxId(RF09);
            temp1 = app_P2P_Phy_Init();
            if((temp != true) || (temp1 != true))
#else
            if(app_P2P_Phy_Init() != true)
#endif
            {
                appInitialized = false;
            }

            if (appInitialized)
            {
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            if (OSAL_QUEUE_Receive(&appData.appQueue, &appMsg, OSAL_WAIT_FOREVER))
            {
                app_P2P_Phy_TaskHandler(p_appMsg);
            }
            break;
        }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
