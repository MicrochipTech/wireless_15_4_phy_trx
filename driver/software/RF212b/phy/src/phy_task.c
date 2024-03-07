/*******************************************************************************
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
*******************************************************************************/



/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
#include "../../phy/inc/phy.h"
#include "definitions.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Externals                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

extern OSAL_SEM_HANDLE_TYPE semPhyInternalHandler;
extern OSAL_SEM_HANDLE_TYPE semTalInternalHandler;

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Prototypes                                                  */
/* ************************************************************************** */
/* ************************************************************************** */
void PHY_Tasks(void)
{    
    if (semPhyInternalHandler != NULL)    
    {
        OSAL_SEM_Pend(&semPhyInternalHandler, OSAL_WAIT_FOREVER);
        PHY_TaskHandler();
    }
}
/* ************************************************************************** */
void TAL_Tasks(void)
{    
    if (semTalInternalHandler != NULL)    
    {
        OSAL_SEM_Pend(&semTalInternalHandler, OSAL_WAIT_FOREVER);
        TAL_TaskHandler();
    }
}

/* ************************************************************************** */
void PHY_PostTask(bool isISRContext)
{
    if(semPhyInternalHandler != NULL)
    {
        if(isISRContext)
        {
            OSAL_SEM_PostISR(&semPhyInternalHandler);
        }
        else
        {
            OSAL_SEM_Post(&semPhyInternalHandler);
        }
    }
    else
    {
        PHY_TaskHandler();
    }
}

/* ************************************************************************** */
void TAL_PostTask(bool isISRContext)
{
    if(semTalInternalHandler != NULL)
    {
        if(isISRContext)
        {
            OSAL_SEM_PostISR(&semTalInternalHandler);
        }
        else
        {
            OSAL_SEM_Post(&semTalInternalHandler);
        }
    }
    else
    {
        TAL_TaskHandler();
    }
}

/* *****************************************************************************
 End of File
 */
