/*******************************************************************************
* File Name: Timer_QD_PM.c
* Version 2.80
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "Timer_QD.h"

static Timer_QD_backupStruct Timer_QD_backup;


/*******************************************************************************
* Function Name: Timer_QD_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_QD_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Timer_QD_SaveConfig(void) 
{
    #if (!Timer_QD_UsingFixedFunction)
        Timer_QD_backup.TimerUdb = Timer_QD_ReadCounter();
        Timer_QD_backup.InterruptMaskValue = Timer_QD_STATUS_MASK;
        #if (Timer_QD_UsingHWCaptureCounter)
            Timer_QD_backup.TimerCaptureCounter = Timer_QD_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Timer_QD_UDB_CONTROL_REG_REMOVED)
            Timer_QD_backup.TimerControlRegister = Timer_QD_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Timer_QD_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_QD_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Timer_QD_RestoreConfig(void) 
{   
    #if (!Timer_QD_UsingFixedFunction)

        Timer_QD_WriteCounter(Timer_QD_backup.TimerUdb);
        Timer_QD_STATUS_MASK =Timer_QD_backup.InterruptMaskValue;
        #if (Timer_QD_UsingHWCaptureCounter)
            Timer_QD_SetCaptureCount(Timer_QD_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Timer_QD_UDB_CONTROL_REG_REMOVED)
            Timer_QD_WriteControlRegister(Timer_QD_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Timer_QD_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_QD_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Timer_QD_Sleep(void) 
{
    #if(!Timer_QD_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Timer_QD_CTRL_ENABLE == (Timer_QD_CONTROL & Timer_QD_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Timer_QD_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Timer_QD_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Timer_QD_Stop();
    Timer_QD_SaveConfig();
}


/*******************************************************************************
* Function Name: Timer_QD_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_QD_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Timer_QD_Wakeup(void) 
{
    Timer_QD_RestoreConfig();
    #if(!Timer_QD_UDB_CONTROL_REG_REMOVED)
        if(Timer_QD_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Timer_QD_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
