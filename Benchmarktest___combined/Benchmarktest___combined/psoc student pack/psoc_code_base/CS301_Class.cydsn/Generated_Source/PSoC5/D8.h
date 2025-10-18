/*******************************************************************************
* File Name: D8.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_D8_H) /* Pins D8_H */
#define CY_PINS_D8_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "D8_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 D8__PORT == 15 && ((D8__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    D8_Write(uint8 value);
void    D8_SetDriveMode(uint8 mode);
uint8   D8_ReadDataReg(void);
uint8   D8_Read(void);
void    D8_SetInterruptMode(uint16 position, uint16 mode);
uint8   D8_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the D8_SetDriveMode() function.
     *  @{
     */
        #define D8_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define D8_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define D8_DM_RES_UP          PIN_DM_RES_UP
        #define D8_DM_RES_DWN         PIN_DM_RES_DWN
        #define D8_DM_OD_LO           PIN_DM_OD_LO
        #define D8_DM_OD_HI           PIN_DM_OD_HI
        #define D8_DM_STRONG          PIN_DM_STRONG
        #define D8_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define D8_MASK               D8__MASK
#define D8_SHIFT              D8__SHIFT
#define D8_WIDTH              1u

/* Interrupt constants */
#if defined(D8__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in D8_SetInterruptMode() function.
     *  @{
     */
        #define D8_INTR_NONE      (uint16)(0x0000u)
        #define D8_INTR_RISING    (uint16)(0x0001u)
        #define D8_INTR_FALLING   (uint16)(0x0002u)
        #define D8_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define D8_INTR_MASK      (0x01u) 
#endif /* (D8__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define D8_PS                     (* (reg8 *) D8__PS)
/* Data Register */
#define D8_DR                     (* (reg8 *) D8__DR)
/* Port Number */
#define D8_PRT_NUM                (* (reg8 *) D8__PRT) 
/* Connect to Analog Globals */                                                  
#define D8_AG                     (* (reg8 *) D8__AG)                       
/* Analog MUX bux enable */
#define D8_AMUX                   (* (reg8 *) D8__AMUX) 
/* Bidirectional Enable */                                                        
#define D8_BIE                    (* (reg8 *) D8__BIE)
/* Bit-mask for Aliased Register Access */
#define D8_BIT_MASK               (* (reg8 *) D8__BIT_MASK)
/* Bypass Enable */
#define D8_BYP                    (* (reg8 *) D8__BYP)
/* Port wide control signals */                                                   
#define D8_CTL                    (* (reg8 *) D8__CTL)
/* Drive Modes */
#define D8_DM0                    (* (reg8 *) D8__DM0) 
#define D8_DM1                    (* (reg8 *) D8__DM1)
#define D8_DM2                    (* (reg8 *) D8__DM2) 
/* Input Buffer Disable Override */
#define D8_INP_DIS                (* (reg8 *) D8__INP_DIS)
/* LCD Common or Segment Drive */
#define D8_LCD_COM_SEG            (* (reg8 *) D8__LCD_COM_SEG)
/* Enable Segment LCD */
#define D8_LCD_EN                 (* (reg8 *) D8__LCD_EN)
/* Slew Rate Control */
#define D8_SLW                    (* (reg8 *) D8__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define D8_PRTDSI__CAPS_SEL       (* (reg8 *) D8__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define D8_PRTDSI__DBL_SYNC_IN    (* (reg8 *) D8__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define D8_PRTDSI__OE_SEL0        (* (reg8 *) D8__PRTDSI__OE_SEL0) 
#define D8_PRTDSI__OE_SEL1        (* (reg8 *) D8__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define D8_PRTDSI__OUT_SEL0       (* (reg8 *) D8__PRTDSI__OUT_SEL0) 
#define D8_PRTDSI__OUT_SEL1       (* (reg8 *) D8__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define D8_PRTDSI__SYNC_OUT       (* (reg8 *) D8__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(D8__SIO_CFG)
    #define D8_SIO_HYST_EN        (* (reg8 *) D8__SIO_HYST_EN)
    #define D8_SIO_REG_HIFREQ     (* (reg8 *) D8__SIO_REG_HIFREQ)
    #define D8_SIO_CFG            (* (reg8 *) D8__SIO_CFG)
    #define D8_SIO_DIFF           (* (reg8 *) D8__SIO_DIFF)
#endif /* (D8__SIO_CFG) */

/* Interrupt Registers */
#if defined(D8__INTSTAT)
    #define D8_INTSTAT            (* (reg8 *) D8__INTSTAT)
    #define D8_SNAP               (* (reg8 *) D8__SNAP)
    
	#define D8_0_INTTYPE_REG 		(* (reg8 *) D8__0__INTTYPE)
#endif /* (D8__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_D8_H */


/* [] END OF FILE */
