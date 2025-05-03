/*
 * bsp.c
 *
 *  Created on: 5 août 2017
 *      Author: Laurent
 */

#include "bsp.h"

/*
 * BSP_LED_Init()
 * Initialize LED pin (PA5) as a High-Speed Push-Pull output
 * Set LED initial state to OFF
 */

void BSP_LED_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA5 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER5_Pos);

	// Configure PA5 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

	// Configure PA5 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR5_Pos);

	// Disable PA5 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5_Msk;

	// Set Initial State OFF
	GPIOA->BSRR |= GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_On()
 * Turn ON LED on PA5
 */

void BSP_LED_On()
{
	GPIOA->BSRR = GPIO_BSRR_BS_5;
}

/*
 * BSP_LED_Off()
 * Turn OFF LED on PA5
 */

void BSP_LED_Off()
{
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_Toggle()
 * Toggle LED on PA5
 */

void BSP_LED_Toggle()
{
	GPIOA->ODR ^= GPIO_ODR_5;
}

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

void BSP_PBPC3_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC3 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER3_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER3_Pos);

	// Enable PC3 Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3_Msk;
	GPIOC->PUPDR |= (0x2UL << GPIO_PUPDR_PUPDR3_Pos);

	// Enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Select Port C as interrupt source for EXTI line 3
	SYSCFG->EXTICR[0] &= ~ SYSCFG_EXTICR1_EXTI3_Msk;
	SYSCFG->EXTICR[0] |=   SYSCFG_EXTICR1_EXTI3_PC;

	// Enable EXTI line 3
	EXTI->IMR |= EXTI_IMR_IM3;

	// Disable Rising / Enable Falling trigger
	EXTI->RTSR &= ~EXTI_RTSR_RT3;
	EXTI->FTSR |=  EXTI_FTSR_FT3;
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

uint8_t BSP_PBPC3_GetState()
{
	uint8_t state;
	if ((GPIOC->IDR & GPIO_IDR_3) == GPIO_IDR_3)
	{
		state = 1;
	}
	else
	{
		state = 0;
	}
	return state;
}

uint8_t BSP_PB_IsPressed(uint32_t debounce){
	uint32_t i = 0;
	uint8_t a = 1;
	while(i<debounce){
		if(BSP_PB_GetState() == 0){
			a = 0;
			break;
		}
		i++;
	}
	return a;
}

uint8_t BSP_PB_IsReleased(uint32_t debounce){
	uint32_t i = 0;
	uint8_t a = 1;
	while(i<debounce){
		if(BSP_PB_GetState() == 1){
			a = 0;

			break;
		}
		i++;
	}
	return a;
}


//extern uint8_t rx_dma_buffer[8];
void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER2_Pos) | (0x02 <<GPIO_MODER_MODER3_Pos);
	// Set PA2 and PA3 to AF1 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00001100);
	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;
	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART2SW_Msk;
	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Actual BaudRate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Actual BaudRate = 115246.0984 -> 0.04% error (better choice)
	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 833;
	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
	// Enable interrupt on RXNE event (disabled with DMA)
	// USART2->CR1 |= USART_CR1_RXNEIE;
	// Setup RX on DMA Channel 5
	// Start DMA clock
//	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//	// Reset DMA1 Channel 5 configuration
//	DMA1_Channel5->CCR = 0x00000000;
//	// Set direction Peripheral -> Memory
//	DMA1_Channel5->CCR &= ~DMA_CCR_DIR;
//	// Peripheral is USART2 RDR
//	DMA1_Channel5->CPAR = (uint32_t)&USART2->RDR;
//	// Peripheral data size is 8-bit (byte)
//	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_PSIZE_Pos);
//	// Disable auto-increment Peripheral address
//	DMA1_Channel5->CCR &= ~DMA_CCR_PINC;
//	// Memory is rx_dma_buffer
//	DMA1_Channel5->CMAR = (uint32_t)rx_dma_buffer;
//	// Memory data size is 8-bit (byte)
//	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);
//	// Enable auto-increment Memory address
//	DMA1_Channel5->CCR |= DMA_CCR_MINC;
//	// Set Memory Buffer size
//	DMA1_Channel5->CNDTR = 8;
//	// DMA mode is circular
//	DMA1_Channel5->CCR |= DMA_CCR_CIRC;
//	// Enable DMA HT & TC interrupts
//	DMA1_Channel5->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;
//	// Enable DMA1 Channel 5
//	DMA1_Channel5->CCR |= DMA_CCR_EN;
//	// Enable USART2 DMA Request on RX
//	USART2->CR3 |= USART_CR3_DMAR;
	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}


/*
 * ADC_Init()
 * Initialize ADC for single channel conversion
 * - Channel 11 -> pin PC1
 */




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

	// Set TIM6 auto-reload register for 1s
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

	// Set TIM6 auto-reload register for 1s
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


void BSP_TIMER_PWM_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA8 and PA9 as Alternate Function
	GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER8_Pos) | (0x02 <<GPIO_MODER_MODER9_Pos);

	// Set PA8 and PA9 to AF2 (TIM1)
	GPIOA->AFR[1] &= ~(0x000000FF);
	GPIOA->AFR[1] |=  (0x00000022);

	// Enable TIM1 clock
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

	// Reset TIM1 configuration
	TIM1->CR1  = 0x0000;
	TIM1->CR2  = 0x0000;
	TIM1->CCER = 0x0000;

	// Set TIM1 prescaler
	// Fck = 48MHz -> /48 = 1MHz counting frequency (1µs resolution)
	TIM1->PSC = (uint16_t) 48 -1;

	// Set Auto-Reload to period = 11ms
	TIM1->ARR = (uint16_t) 3000;

	// Enable Auto-Reload Preload register
	TIM1->CR1 |= TIM_CR1_ARPE;

	// Setup Input Capture
	TIM1->CCMR1 = 0x0000;
	TIM1->CCMR2 = 0x0000;

	// Setup PWM mode 1 output
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

	// Set default PWM values
	TIM1->CCR1 = 1500;
	TIM1->CCR2 = 1500;

	// Enable Outputs
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	// Enable Main output
	TIM1->BDTR |= TIM_BDTR_MOE;

	// Enable TIM1
	TIM1->CR1 |= TIM_CR1_CEN;
}


void BSP_NVIC_Init()
{
	// Set maximum priority for EXTI line 4 to 15 interrupts
	NVIC_SetPriority(EXTI0_1_IRQn, 4);

	// Enable EXTI line 4 to 15 (user button on line 13) interrupts
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	// Set maximum priority for EXTI line 4 to 15 interrupts
	NVIC_SetPriority(EXTI4_15_IRQn, 2);

	// Enable EXTI line 4 to 15 (user button on line 13) interrupts
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Set maximum priority for EXTI line 4 to 15 interrupts
	NVIC_SetPriority(EXTI2_3_IRQn, 3);

	// Enable EXTI line 4 to 15 (user button on line 13) interrupts
	NVIC_EnableIRQ(EXTI2_3_IRQn);

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


