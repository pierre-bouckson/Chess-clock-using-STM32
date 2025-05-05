/*
 * bsp.c
 *
 *  Created on: 5 août 2017
 *      Author: Laurent
 */

 #include "bsp.h"

 /*
  * BSP_PB_Init()
  * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
  */
 
 void BSP_PB_Init()
 {
	 // Enable GPIOB clock
	 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
 
	 // Configure PB0 as input
	 GPIOB->MODER &= ~GPIO_MODER_MODER0_Msk;
	 GPIOB->MODER |= (0x00 <<GPIO_MODER_MODER0_Pos);
 
	 // Enable PB0 Pull-down
	 GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk;
	 GPIOB->PUPDR |= (0x2UL << GPIO_PUPDR_PUPDR0_Pos);
 
	 // Enable SYSCFG clock
	 RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 
	 // Select Port C as interrupt source for EXTI line 13
	 SYSCFG->EXTICR[0] &= ~ SYSCFG_EXTICR1_EXTI0_Msk;
	 SYSCFG->EXTICR[0] |=   SYSCFG_EXTICR1_EXTI0_PB;
 
	 // Enable EXTI line 0
	 EXTI->IMR |= EXTI_IMR_IM0;
 
	 // Disable Rising / Enable Falling trigger
	 EXTI->RTSR &= ~EXTI_RTSR_RT0;
	 EXTI->FTSR |=  EXTI_FTSR_FT0;
 }
 /*
  * BSP_PB_GetState()
  * Returns the state of the button (0=released, 1=pressed)
  */
 
 void BSP_PBPA4_Init()
 {
	 // Enable GPIOA clock
	 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
 
	 // Configure PA4 as input
	 GPIOA->MODER &= ~GPIO_MODER_MODER4_Msk;
	 GPIOA->MODER |= (0x00 <<GPIO_MODER_MODER4_Pos);
 
	 // Enable PA4 Pull-down
	 GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4_Msk;
	 GPIOA->PUPDR |= (0x2UL << GPIO_PUPDR_PUPDR4_Pos);
 
	 // Enable SYSCFG clock
	 RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 
	 // Select Port C as interrupt source for EXTI line 13
	 SYSCFG->EXTICR[1] &= ~ SYSCFG_EXTICR2_EXTI4_Msk;
	 SYSCFG->EXTICR[1] |=   SYSCFG_EXTICR2_EXTI4_PA;
 
	 // Enable EXTI line 4
	 EXTI->IMR |= EXTI_IMR_IM4;
 
	 // Disable Rising / Enable Falling trigger
	 EXTI->RTSR &= ~EXTI_RTSR_RT4;
	 EXTI->FTSR |=  EXTI_FTSR_FT4;
 
 }
 
 
 uint8_t BSP_PB_GetState()
 {
	 uint8_t state;
	 if ((GPIOB->IDR & GPIO_IDR_0) == GPIO_IDR_0)
	 {
		 state = 1;
	 }
	 else
	 {
		 state = 0;
	 }
	 return state;
 }
 
 uint8_t BSP_PBPA4_GetState()
 {
	 uint8_t state;
	 if ((GPIOA->IDR & GPIO_IDR_4) == GPIO_IDR_4)
	 {
		 state = 1;
	 }
	 else
	 {
		 state = 0;
	 }
	 return state;
 }
 
 uint8_t BSP_PBPA4_Toggle()
 {
	 uint8_t etat = 0;
	 if(BSP_PBPA4_GetState() == 1)
	 {
 
	 }
 }
 
 
 /*
  * BSP_TIMER_Timebase_Init()
  * TIM6 at 48MHz
  * Prescaler   = 48000  -> Counting frequency is 1kHz
  * Auto-reload = 10 	-> Update period is 10ms
  * Enable Update Interrupt
  */
 extern uint16_t    Temps_a_jouer;
 void BSP_TIMER6_Timebase_Init()
 {
	 // Enable TIM6 clock
	 RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
 
	 // Reset TIM6 configuration
	 TIM6->CR1 = 0x0000;
	 TIM6->CR2 = 0x0000;
 
	 // Set TIM6 prescaler
	 // Fck = 48MHz -> /48000 = 1KHz counting frequency
	 TIM6->PSC = (uint16_t) 48000 -1;
 
	 // Set TIM6 auto-reload register for 60s
	 TIM6->ARR = (uint16_t) 60000 -1;
 
	 //Enable STOP
	 TIM6->CR1 |= (0x1UL << (3U));
 
	 // Enable auto-reload preload
	 TIM6->CR1 |= TIM_CR1_ARPE;
 
	 //Enable Interrupt
	 TIM6->DIER = (0x1UL << (0U));
 
	 TIM6->CR1 &= ~TIM_CR1_CEN;
 
	 //Enable TIM6
	 //TIM6->CR1 |= TIM_CR1_CEN;
 }
 
 void BSP_TIMER7_Timebase_Init()
 {
	 // Enable TIM6 clock
	 RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
 
	 // Reset TIM6 configuration
	 TIM7->CR1 = 0x0000;
	 TIM7->CR2 = 0x0000;
 
	 // Set TIM6 prescaler
	 // Fck = 48MHz -> /48000 = 1KHz counting frequency
	 TIM7->PSC = (uint16_t) 48000 -1;
 
	 // Set TIM6 auto-reload register for 60s
	 TIM7->ARR = (uint16_t) 60000 -1;
 
	 //Enable STOP
	 TIM7->CR1 |= (0x1UL << (3U));
 
	 // Enable auto-reload preload
	 TIM7->CR1 |= TIM_CR1_ARPE;
 
	 //Enable Interrupt
	 TIM7->DIER = (0x1UL << (0U));
 
	 TIM7->CR1 &= ~TIM_CR1_CEN;
 
	 //Enable TIM6
	 //TIM7->CR1 |= TIM_CR1_CEN;
 }
 
 void BSP_NVIC_Init()
 {
	 // Set maximum priority for EXTI line 4 to 15 interrupts
	 NVIC_SetPriority(EXTI0_1_IRQn, 2);
 
	 // Enable EXTI line 4 to 15 (user button on line 13) interrupts
	 NVIC_EnableIRQ(EXTI0_1_IRQn);
 
 //	// Set maximum priority for EXTI line 4 to 15 interrupts
 //	NVIC_SetPriority(EXTI4_15_IRQn, 2);
 //
 //	// Enable EXTI line 4 to 15 (user button on line 13) interrupts
 //	NVIC_EnableIRQ(EXTI4_15_IRQn);
 //
 //	// Set maximum priority for EXTI line 4 to 15 interrupts
 //	NVIC_SetPriority(EXTI2_3_IRQn, 3);
 //
 //	// Enable EXTI line 4 to 15 (user button on line 13) interrupts
 //	NVIC_EnableIRQ(EXTI2_3_IRQn);
 
	 // Interrupt TIM6
	 NVIC_SetPriority(TIM6_DAC_IRQn, 0);
 
	 // Enable
	 NVIC_EnableIRQ(TIM6_DAC_IRQn);
 
	 // TIM7
	 NVIC_SetPriority(TIM7_IRQn, 1);
 
	 // Enable
	 NVIC_EnableIRQ(TIM7_IRQn);
 }
 
 
 void BSP_I2C1_Init1()
 {
	 // Pin configuration for I2C2 pins
	 // SCL -> PB8
	 // SDA -> PB9
	 // Enable GPIOB clock
	 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
 
	 // Configure PB8, PB9 as AF mode
	 GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	 GPIOB->MODER |= (0x02 <<16U) | (0x02 <<18U);
 
	 // Connect to I2C1 (AF1)
	 GPIOB->AFR[1] &= ~(0x000000FF);
	 GPIOB->AFR[1] |=   0x00000011;
 
	 // Setup Open-Drain
	 GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;
 
	 // Select SYSCLK as I2C1 clock (48MHz)
	 RCC->CFGR3 |= RCC_CFGR3_I2C1SW;
 
	 // Enable I2C1 clock
	 RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
 
	 // Make sure I2C1 is disabled
	 I2C1->CR1 &= ~I2C_CR1_PE;
 
	 // Reset I2C1 Configuration to default values
	 I2C1->CR1 	  = 0x00000000;
	 I2C1->CR2 	  = 0x00000000;
	 I2C1->TIMINGR = 0x00000000;
 
	 // Configure timing for 100kHz, 50% duty cycle
	 I2C1->TIMINGR |= ((4 -1) <<I2C_TIMINGR_PRESC_Pos); // Clock prescaler /4 -> 12MHz
	 I2C1->TIMINGR |= (120 	 <<I2C_TIMINGR_SCLH_Pos);  // High half-period = 5µs
	 I2C1->TIMINGR |= (120     <<I2C_TIMINGR_SCLL_Pos);  // Low  half-period = 5µs
 
	 // Enable I2C1
	 I2C1->CR1 |= I2C_CR1_PE;
 }
 
 
 uint8_t	BSP_I2C1_Write1( uint8_t device_address,
		 uint8_t register_address,
		 uint8_t *buffer, uint8_t nbytes )
 {
	 uint32_t 	timeout;	// Flag waiting timeout
	 uint8_t		n;			// Loop counter
 
	 // Set device address
	 I2C1->CR2 &= ~I2C_CR2_SADD_Msk;
	 I2C1->CR2 |= ((device_address <<1U) <<I2C_CR2_SADD_Pos);
 
	 // Set I2C in Write mode
	 I2C1->CR2 &= ~I2C_CR2_RD_WRN;
 
	 // Transfer NBYTES, with AUTOEND
	 I2C1->CR2 &= ~I2C_CR2_NBYTES;
	 I2C1->CR2 |= ((nbytes+1) <<16U);
	 I2C1->CR2 |= I2C_CR2_AUTOEND;
 
	 // Clear STOPF flag
	 I2C1->ICR |= I2C_ICR_STOPCF;
 
	 // Start I2C transaction
	 I2C1->CR2 |= I2C_CR2_START;
 
	 // Wait for TXIS with timeout
	 timeout = 100000;
	 while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
	 {
		 timeout--;
		 if (timeout == 0) return 1;
	 }
 
	 // Send register address
	 I2C1->TXDR = register_address;
 
	 n = nbytes;
	 while(n>0)
	 {
		 // Wait for TXIS with timeout
		 timeout = 100000;
		 while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
		 {
			 timeout--;
			 if (timeout == 0) return 2;
		 }
 
		 // Send data
		 I2C1->TXDR = *buffer;
		 buffer++;
		 n--;
	 }
 
	 // Wait for STOPF with timeout
	 timeout = 100000;
	 while (((I2C1->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF)
	 {
		 timeout--;
		 if (timeout == 0) return 3;
	 }
 
	 // Return success
	 return 0;
 }
 
 void BSP_I2C1_Init2()
 {
	 // Pin configuration for I2C2 pins
	 // SCL -> PB10
	 // SDA -> PB11
	 // Enable GPIOB clock
	 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
 
	 // Configure PB10, PB11 as AF mode
	 GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
	 GPIOB->MODER |= (0x02 <<20U) | (0x02 <<22U);
 
	 // Connect to I2C2 (AF1)
	 GPIOB->AFR[1] &= ~(0x0000FF00);
	 GPIOB->AFR[1] |=   0x00001100;
 
	 // Setup Open-Drain
	 GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;
 
	 // Select SYSCLK as I2C1 clock (48MHz)
	 //RCC->CFGR3 |= RCC_CFGR3_I2C1SW; //   PROBLEME : I2C2 ne supporte pas les horloge extern
	 // il faut gerer la clock differement
 
	 // Enable I2C1 clock
	 RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
 
	 // Make sure I2C1 is disabled
	 I2C2->CR1 &= ~I2C_CR1_PE;
 
	 // Reset I2C1 Configuration to default values
	 I2C2->CR1 	  = 0x00000000;
	 I2C2->CR2 	  = 0x00000000;
	 I2C2->TIMINGR = 0x00000000;
 
	 // Configure timing for 100kHz, 50% duty cycle
	 I2C2->TIMINGR |= ((4 -1) <<I2C_TIMINGR_PRESC_Pos); // Clock prescaler /4 -> 12MHz
	 I2C2->TIMINGR |= (120 	 <<I2C_TIMINGR_SCLH_Pos);  // High half-period = 5µs
	 I2C2->TIMINGR |= (120     <<I2C_TIMINGR_SCLL_Pos);  // Low  half-period = 5µs
 
	 // Enable I2C1
	 I2C2->CR1 |= I2C_CR1_PE;
 }
 
 
 uint8_t	BSP_I2C1_Write2( uint8_t device_address,
		 uint8_t register_address,
		 uint8_t *buffer, uint8_t nbytes )
 {
	 uint32_t 	timeout;	// Flag waiting timeout
	 uint8_t		n;			// Loop counter
 
	 // Set device address
	 I2C2->CR2 &= ~I2C_CR2_SADD_Msk;
	 I2C2->CR2 |= ((device_address <<1U) <<I2C_CR2_SADD_Pos);
 
	 // Set I2C in Write mode
	 I2C2->CR2 &= ~I2C_CR2_RD_WRN;
 
	 // Transfer NBYTES, with AUTOEND
	 I2C2->CR2 &= ~I2C_CR2_NBYTES;
	 I2C2->CR2 |= ((nbytes+1) <<16U);
	 I2C2->CR2 |= I2C_CR2_AUTOEND;
 
	 // Clear STOPF flag
	 I2C2->ICR |= I2C_ICR_STOPCF;
 
	 // Start I2C transaction
	 I2C2->CR2 |= I2C_CR2_START;
 
	 // Wait for TXIS with timeout
	 timeout = 100000;
	 while (((I2C2->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
	 {
		 timeout--;
		 if (timeout == 0) return 1;
	 }
 
	 // Send register address
	 I2C2->TXDR = register_address;
 
	 n = nbytes;
	 while(n>0)
	 {
		 // Wait for TXIS with timeout
		 timeout = 100000;
		 while (((I2C2->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
		 {
			 timeout--;
			 if (timeout == 0) return 2;
		 }
 
		 // Send data
		 I2C2->TXDR = *buffer;
		 buffer++;
		 n--;
	 }
 
	 // Wait for STOPF with timeout
	 timeout = 100000;
	 while (((I2C2->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF)
	 {
		 timeout--;
		 if (timeout == 0) return 3;
	 }
 
	 // Return success
	 return 0;
 }
 
 uint8_t		tx_data[0];
 void MAX6958_Init1()
 {
	 //Normal Operation
	 tx_data[0] = 0x01; 			//0x01     OR    0x3E
	 BSP_I2C1_Write1(0b0111000, 0x04, tx_data, 1);
 
	 //Mode Hexadecimal
	 tx_data[0] = 0x0F;
	 BSP_I2C1_Write1(0b0111000, 0x01, tx_data, 1);
 
	 //Max luminosity
	 tx_data[0] = 0x3F;
	 BSP_I2C1_Write1(0b0111000, 0x02, tx_data, 1);
 
 
 
 //	tx_data[0] = 0x08;
 //	BSP_I2C1_Write1(0b0111000, 0x23, tx_data, 1);
 //	BSP_I2C1_Write1(0b0111000, 0x22, tx_data, 1);
 //	BSP_I2C1_Write1(0b0111000, 0x21, tx_data, 1);
 //	BSP_I2C1_Write1(0b0111000, 0x20, tx_data, 1);
 }
 
 
 void print_digit1(uint32_t heure)
 {
 
	 uint32_t secondes = heure / 1000; // Convertir en secondes
	 uint32_t minutes = secondes / 60;  // Obtenir les minutes
	 uint32_t sec_restantes = secondes % 60; // Obtenir les secondes restantes
 
	 uint8_t dix_min = minutes / 10;
	 uint8_t min = minutes % 10;
	 uint8_t dix_sec = sec_restantes / 10;
	 uint8_t sec = sec_restantes % 10;
 
	 tx_data[0] = sec;
	 BSP_I2C1_Write1(0b0111000, 0x23, tx_data, 1);
 
	 tx_data[0] = dix_sec;
	 BSP_I2C1_Write1(0b0111000, 0x22, tx_data, 1);
 
 
	 tx_data[0] = min;
	 BSP_I2C1_Write1(0b0111000, 0x21, tx_data, 1);
 
 
	 tx_data[0] = dix_min;
	 BSP_I2C1_Write1(0b0111000, 0x20, tx_data, 1);
 
 }
 
 void MAX6958_Init2()
 {
	 //Normal Operation
	 tx_data[0] = 0x01; 			//0x01     OR    0x3E
	 BSP_I2C1_Write2(0b0111000, 0x04, tx_data, 1);
 
	 //Mode Hexadecimal
	 tx_data[0] = 0x0F;
	 BSP_I2C1_Write2(0b0111000, 0x01, tx_data, 1);
 
	 //Max luminosity
	 tx_data[0] = 0x3F;
	 BSP_I2C1_Write2(0b0111000, 0x02, tx_data, 1);
 
 //	tx_data[0] = 0x08;
 //	BSP_I2C1_Write2(0b0111000, 0x23, tx_data, 1);
 //	BSP_I2C1_Write2(0b0111000, 0x22, tx_data, 1);
 //	BSP_I2C1_Write2(0b0111000, 0x21, tx_data, 1);
 //	BSP_I2C1_Write2(0b0111000, 0x20, tx_data, 1);
 }
 
 
 void print_digit2(uint32_t heure)
 {
 
	 uint32_t secondes = heure / 1000; // Convertir en secondes
	 uint32_t minutes = secondes / 60;  // Obtenir les minutes
	 uint32_t sec_restantes = secondes % 60; // Obtenir les secondes restantes
 
	 uint8_t dix_min = minutes / 10;
	 uint8_t min = minutes % 10;
	 uint8_t dix_sec = sec_restantes / 10;
	 uint8_t sec = sec_restantes % 10;
 
	 tx_data[0] = sec;
	 BSP_I2C1_Write2(0b0111000, 0x23, tx_data, 1);
 
	 tx_data[0] = dix_sec;
	 BSP_I2C1_Write2(0b0111000, 0x22, tx_data, 1);
 
 
	 tx_data[0] = min;
	 BSP_I2C1_Write2(0b0111000, 0x21, tx_data, 1);
 
 
	 tx_data[0] = dix_min;
	 BSP_I2C1_Write2(0b0111000, 0x20, tx_data, 1);
 
 }
 
 
 