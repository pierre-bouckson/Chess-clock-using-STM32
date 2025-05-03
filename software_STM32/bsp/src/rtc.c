/*
 * rtc.c
 *
 *  Created on: 20 août 2017
 *      Author: Laurent
 */

#include "rtc.h"


static uint8_t byte2bcd	(uint8_t byte);
static uint8_t bcd2byte	(uint8_t bcd);

/*
 * RTC_Config()
 * Setup RTC clock and start RTC
 */

void BSP_RTC_Clock_Config()
{
	// Enable the PWR clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// Enable access to RTC and Backup registers
	PWR->CR |= PWR_CR_DBP;

	// Resets Backup Domain Config
	RCC->BDCR |= RCC_BDCR_BDRST;
	RCC->BDCR &= ~RCC_BDCR_BDRST;

	// Set driving capability to medium high
	RCC->BDCR &= ~RCC_BDCR_LSEDRV_Msk;
	RCC->BDCR |= (0x02 <<RCC_BDCR_LSEDRV_Pos);

	// Start LSE clock
	RCC->BDCR |= RCC_BDCR_LSEON;

	// Wait until LSE is ready
	while ( (RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY);

	// Select LSE as RTC clock source
	RCC->BDCR &= ~RCC_BDCR_RTCSEL_Msk;
	RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;

	// Enable RTC clock
	RCC->BDCR |= RCC_BDCR_RTCEN;
}

/*
 * BSP_RTC_SetTime(uint32_t time)
 * - Sets RTC Prescalers
 * - Sets RTC time
 */

void BSP_RTC_SetTime(time_t *ptime)
{
	uint32_t	bcdtime;
	bcdtime = 	( (byte2bcd(ptime->hours))   <<16U) |
			( (byte2bcd(ptime->minutes)) <<8U)  |
			( (byte2bcd(ptime->seconds)) <<0U);

	// Enable Write access for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Enter Init
	RTC->ISR |= RTC_ISR_INIT;
	while ((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF);

	// Setup prescalers for 1s RTC clock
	RTC->PRER = 0x007F00FF;

	// Set time
	RTC->TR = bcdtime;

	// Exit Init
	RTC->ISR &= ~RTC_ISR_INIT;

	// Disable Write access for RTC registers
	RTC->WPR = 0xFE;
	RTC->WPR = 0x64;
}

/*
 * BSP_RTC_GetTime()
 * Retreive current RTC time in binary format
*/

void BSP_RTC_GetTime(time_t *ptime)
{
   ptime->hours   = bcd2byte((RTC->TR & (RTC_TR_HT_Msk  | RTC_TR_HU_Msk )) >>RTC_TR_HU_Pos);
   ptime->minutes = bcd2byte((RTC->TR & (RTC_TR_MNT_Msk | RTC_TR_MNU_Msk)) >>RTC_TR_MNU_Pos);
   ptime->seconds = bcd2byte((RTC->TR & (RTC_TR_ST_Msk  | RTC_TR_SU_Msk )) >>RTC_TR_SU_Pos);
}




static uint8_t byte2bcd(uint8_t byte)
{
  uint8_t bcdhigh = 0;
  while (byte >= 10)
  {
    bcdhigh++;
    byte -= 10;
  }
  return  ((uint8_t)(bcdhigh << 4) | byte);
}

/*
 * Convert from 2 digit BCD to Binary
 * param  BCD value to be converted.
 * retval Converted word
 */

static uint8_t bcd2byte(uint8_t bcd)
{
  uint8_t tmp = 0;
  tmp = ((uint8_t)(bcd & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (bcd & (uint8_t)0x0F));
}

/*
 * BSP_RTC_EXTI_Init()
 * Enable RTC (EXTI17) rising edge Interrupt
 */

void BSP_RTC_EXTI_Init()
{
	// Enable EXTI line 17
	EXTI->IMR |= EXTI_IMR_IM17;

	// Enable Rising / Disable Falling trigger
	EXTI->RTSR |=  EXTI_RTSR_RT17;
	EXTI->FTSR &= ~EXTI_FTSR_FT17;
}

/*
 * BSP_RTC_SetAlarmA(time_t *ptime)
 */

void BSP_RTC_SetAlarmA(time_t *ptime)
{
	uint32_t	bcdtime;

	bcdtime = 	( (byte2bcd(ptime->hours))   <<16U) |
			( (byte2bcd(ptime->minutes)) <<8U)  |
			( (byte2bcd(ptime->seconds)) <<0U);

	// Enable Write access for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Disable Alarm A
	RTC->CR &= ~RTC_CR_ALRAE;

	// Set Alarm time and discard date matching
	RTC->ALRMAR = RTC_ALRMAR_MSK4 | bcdtime;

	// Enable Alarm A and associated interrupt
	RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;

	// Disable Write access for RTC registers
	RTC->WPR = 0xFE;
	RTC->WPR = 0x64;
}

/*
 * BSP_RTC_GetAlarmA(time_t *ptime)
 */

void BSP_RTC_GetAlarmA(time_t *ptime)
{
  ptime->hours   = bcd2byte((RTC->ALRMAR & (RTC_ALRMAR_HT_Msk  |
                             RTC_ALRMAR_HU_Msk )) >>RTC_ALRMAR_HU_Pos);

  ptime->minutes = bcd2byte((RTC->ALRMAR & (RTC_ALRMAR_MNT_Msk |
                             RTC_ALRMAR_MNU_Msk)) >>RTC_ALRMAR_MNU_Pos);

  ptime->seconds = bcd2byte((RTC->ALRMAR & (RTC_ALRMAR_ST_Msk  |
                             RTC_ALRMAR_SU_Msk )) >>RTC_ALRMAR_SU_Pos);
}
