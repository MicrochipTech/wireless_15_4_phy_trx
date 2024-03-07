<#--
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
-->

    // Set up OSAL for RF Stack Library usage
    osalAPIList.OSAL_CRIT_Enter      = OSAL_CRIT_Enter;
    osalAPIList.OSAL_CRIT_Leave      = OSAL_CRIT_Leave;

    osalAPIList.OSAL_SEM_Create      = OSAL_SEM_Create;
    osalAPIList.OSAL_SEM_Pend        = OSAL_SEM_Pend;
    osalAPIList.OSAL_SEM_Post        = OSAL_SEM_Post;
    osalAPIList.OSAL_SEM_PostISR     = OSAL_SEM_PostISR;
    osalAPIList.OSAL_SEM_GetCount    = OSAL_SEM_GetCount;

    osalAPIList.OSAL_QUEUE_Create    = OSAL_QUEUE_Create;
    osalAPIList.OSAL_QUEUE_Send      = OSAL_QUEUE_Send;
    osalAPIList.OSAL_QUEUE_SendISR   = OSAL_QUEUE_SendISR;
    osalAPIList.OSAL_QUEUE_Receive   = OSAL_QUEUE_Receive;
    osalAPIList.OSAL_QUEUE_IsFullISR = OSAL_QUEUE_IsFullISR;
    osalAPIList.OSAL_QUEUE_CreateSet = OSAL_QUEUE_CreateSet;
    osalAPIList.OSAL_QUEUE_AddToSet  = OSAL_QUEUE_AddToSet;
    osalAPIList.OSAL_QUEUE_SelectFromSet = OSAL_QUEUE_SelectFromSet;

    osalAPIList.OSAL_MemAlloc = OSAL_Malloc;
    osalAPIList.OSAL_MemFree = OSAL_Free;

    //Create the semmaphore
    OSAL_SEM_Create(&semPhyInternalHandler, OSAL_SEM_TYPE_COUNTING, 20, 0);
    OSAL_SEM_Create(&semTalInternalHandler, OSAL_SEM_TYPE_COUNTING, 20, 0);

    