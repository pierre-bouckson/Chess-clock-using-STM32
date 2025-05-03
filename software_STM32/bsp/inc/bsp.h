/*
 * bsp.h
 *
 *  Created on: 5 août 2017
 *      Author: Laurent
 */

#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32f0xx.h"

/*
 * LED driver functions
 */

void	BSP_LED_Init	(void);
void	BSP_LED_On	(void);
void	BSP_LED_Off	(void);
void	BSP_LED_Toggle	(void);


void       BSP_PB_Init		(void);
uint8_t    BSP_PB_GetState	(void);

void BSP_PBPA4_Init();
void BSP_PBPC3_Init();
uint8_t BSP_PB_IsReleased(uint32_t debounce);

uint8_t BSP_PBPA4_GetState();
uint8_t BSP_PBPC3_GetState();

/*
 * ADC functions
 */
void BSP_ADC_Init		(void);

/*
 * Debug Console init
 */

void	BSP_Console_Init	(void);

/*
 * Timer functions
 */
void BSP_TIMER6_Timebase_Init(void);
void BSP_TIMER7_Timebase_Init(void);

/*
 * Timer functions
 */
uint8_t BSP_PB_IsPressed(uint32_t debounce);

void BSP_TIMER_IC_Init		(void);

void BSP_TIMER_PWM_Init(void);

void BSP_DAC_Init(void);

void BSP_I2C1_Init1(void);

void BSP_I2C1_Init2(void);

void BSP_DBG_Pins_Init(void);   // fais moi meme

uint8_t	BSP_I2C1_Read( uint8_t device_address,
                       uint8_t register_address,
                       uint8_t *buffer,
                       uint8_t nbytes );

uint8_t	BSP_I2C1_Write1( uint8_t device_address,
                        uint8_t register_address,
                        uint8_t *buffer, uint8_t nbytes );

uint8_t	BSP_I2C1_Write2( uint8_t device_address,
                        uint8_t register_address,
                        uint8_t *buffer, uint8_t nbytes );

void BSP_SPI1_Init();

uint8_t BSP_SPI_SendReceive(uint8_t tx_byte);

void BSP_LPS25H_Read(uint8_t register_address, uint8_t *buffer, uint8_t nbytes);

void BSP_LPS25H_Write(uint8_t register_address, uint8_t data);

/*
 * NVIC initialization
 */
void BSP_NVIC_Init				(void);
void BSP_TIMER_Timebase_Init(void);

void BSP_Console_Init(void);

void BSP_DBG_Pin_Init(void);

void MAX6958_Init1(void);

void print_digit1(uint32_t heure);

void MAX6958_Init2(void);

void print_digit2(uint32_t heure);

#endif /* BSP_INC_BSP_H_ */
