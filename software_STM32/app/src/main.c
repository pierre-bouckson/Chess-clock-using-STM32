/*
 * main.c
 *
 *  Created on: 5 août 2017
 *      Author: Pierre
 */


 #include "stm32f0xx.h"
 #include "bsp.h"
 #include "delay.h"
 #include "main.h"
 #include "sinewave_table.h"
 #include "rtc.h"
 
 #define N 60000
 
 // Static functions
 static void SystemClock_Config(void);
 void temps_bonus(uint8_t joueur);
 
 
 uint16_t	TIM6_counter;
 uint16_t	TIM7_counter;
 
 uint8_t min_passe_TIM6 = 0;
 uint8_t min_passe_TIM7 = 0;
 
 uint8_t min_TIM6;
 uint8_t min_TIM7;
 
 uint8_t etat = 1;
 uint8_t etat_btn;
 uint8_t temps = 1;
 uint8_t button_irq = 0;
 uint8_t button_irqPA4;
 uint8_t button_irqPC3;
 uint8_t choose = 1;
 
 uint32_t temps_bonus1 = 0;
 uint32_t temps_bonus2 = 0;
 
 
 uint32_t    Temps_a_jouer; //   <-----------  Temps des joueurs
 
 
 // Main function
 int main()
 {
	 // Configure System Clock
	 SystemClock_Config();
 
	 BSP_PB_Init();
	 BSP_PBPA4_Init();
 
	 BSP_TIMER6_Timebase_Init();  //Timer6
	 BSP_TIMER7_Timebase_Init();  //Timer7
 
	 BSP_I2C1_Init1();
	 BSP_I2C1_Init2();
 
	 MAX6958_Init1();  //Driver LED 1
	 MAX6958_Init2();  //Driver LED 2
 
	 BSP_NVIC_Init();
 
	 // Main loop
	 while(1)
	 {
 
		 if (button_irq >= 1)
		 {
			 if(temps == 5)
			 {
				 temps=1;
			 }
			 else
			 {
				 temps++;
			 }
 
			 TIM6->EGR |= TIM_EGR_UG;
			 TIM7->EGR |= TIM_EGR_UG;
			 TIM6->CR1 &= ~TIM_CR1_CEN;
			 TIM7->CR1 &= ~TIM_CR1_CEN;
 
			 etat = 1;
			 choose = 1;
 
			 min_passe_TIM6 = 0;
			 min_passe_TIM7 = 0;
			 min_TIM6 = 0;
			 min_TIM7 = 0;
			 temps_bonus1=0;
			 temps_bonus2=0;
 
			 MAX6958_Init1();
			 MAX6958_Init2();
 
			 button_irq = 0;
 
			 etat_btn = BSP_PBPA4_GetState();
		 }
 
 
		 switch(temps)
		 {
		 case 1:
			 Temps_a_jouer = 60000;
			 break;
		 case 2 :
			 Temps_a_jouer = 180000;
			 break;
		 case 3 :
			 Temps_a_jouer = 180000;
			 break;
		 case 4 :
			 Temps_a_jouer = 300000;
			 break;
		 case 5 :
			 Temps_a_jouer = 600000;
			 break;
		 }
 
 
		 switch(etat)
		 {
		 case 1 :  //		Etat de debut, aucune clock lancer
			 if(BSP_PBPA4_GetState() != etat_btn){
				 choose = 0;
				 if(BSP_PBPA4_GetState() == 0)
				 {
					 etat = 3;
				 }
				 if(BSP_PBPA4_GetState() == 1)
				 {
					 etat = 2;
				 }
			 }
			 break;
 
		 case 2 :
 
			 TIM6->CR1 &= ~TIM_CR1_CEN;//		Pause TIM6
			 TIM7->CR1 |= TIM_CR1_CEN;//         TIM7 lancer
			 if(BSP_PBPA4_GetState() == 0)
			 {
				 etat = 3;
				 temps_bonus(1);
			 }
			 break;
 
		 case 3 :
 
			 TIM7->CR1 &= ~TIM_CR1_CEN;// 		Pause TIM7
			 TIM6->CR1 |= TIM_CR1_CEN;//         TIM6 lancer
			 if(BSP_PBPA4_GetState() == 1)
			 {
				 etat = 2;
				 temps_bonus(2);
			 }
			 break;
		 }
 
		 // PRINT DIGIT
		 if(choose == 0)
		 {
			 print_digit1(Temps_a_jouer + temps_bonus1 - (min_passe_TIM7*60000) - TIM7->CNT);
			 print_digit2(Temps_a_jouer + temps_bonus2 - (min_passe_TIM6*60000) - TIM6->CNT);
		 }
		 else
		 {
			 print_digit2(Temps_a_jouer);
			 if(temps == 3)
			 {
				 print_digit1(2000);
			 }
			 else if(temps == 1)
			 {
				 print_digit1(1000);
			 }
			 else
			 {
				 print_digit1(0);
			 }
 
		 }
 
		 // AJOUT MIN
		 if(min_TIM6 == 1)
		 {
			 min_passe_TIM6++;
			 min_TIM6 = 0;
		 }
 
		 if(min_TIM7 == 1)
		 {
			 min_passe_TIM7++;
			 min_TIM7 = 0;
		 }
 
		 // SI PERDU :
		 if((Temps_a_jouer + temps_bonus1 - (min_passe_TIM6*60000) - TIM6->CNT) < 500){
			 etat = 1;				//retourne a l'etat 1
			 print_digit2(0);		//Rester a 00:00 si perdu au temps
		 }
 
		 if((Temps_a_jouer + temps_bonus2 - (min_passe_TIM7*60000) - TIM7->CNT) < 500){
			 etat = 1;
			 print_digit1(0);		//Rester a 00:00 si perdu au temps
		 }
 
	 }
 
 }
 
 void temps_bonus(uint8_t joueur)
 {
	 if(temps == 3)
	 {
		 if(joueur == 1)
		 {
			 temps_bonus1 += 2000;
		 }
		 else
		 {
			 temps_bonus2 += 2000;
		 }
	 }
	 if(temps == 1)
	 {
		 if(joueur == 1)
		 {
			 temps_bonus1 += 1000;
		 }
		 else
		 {
			 temps_bonus2 += 1000;
		 }
	 }
 }
 
 
 
 
 
 static void SystemClock_Config()
 {
	 uint32_t	HSE_Status;
	 uint32_t	PLL_Status;
	 uint32_t	SW_Status;
	 uint32_t	timeout = 0;
	 timeout = 1000000;
	 // Start HSE in Bypass Mode
	 RCC->CR |= RCC_CR_HSEBYP;
	 RCC->CR |= RCC_CR_HSEON;
	 // Wait until HSE is ready
	 do
	 {
		 HSE_Status = RCC->CR & RCC_CR_HSERDY_Msk;
		 timeout--;
	 } while ((HSE_Status == 0) && (timeout > 0));
	 // Select HSE as PLL input source
	 RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	 RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);
	 // Set PLL PREDIV to /1
	 RCC->CFGR2 = 0x00000000;
	 // Set PLL MUL to x6
	 RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	 RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);
	 // Enable the main PLL
	 RCC-> CR |= RCC_CR_PLLON;
	 // Wait until PLL is ready
	 do
	 {
		 PLL_Status = RCC->CR & RCC_CR_PLLRDY_Msk;
		 timeout--;
	 } while ((PLL_Status == 0) && (timeout > 0));
	 // Set AHB prescaler to /1
	 RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	 RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	 //Set APB1 prescaler to /1
	 RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	 RCC->CFGR |= RCC_CFGR_PPRE_DIV1;
	 // Enable FLASH Prefetch Buffer and set Flash Latency
	 FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
	 /* --- Until this point, MCU was still clocked by HSI at 8MHz ---*/
	 /* --- Switching to PLL at 48MHz Now!  Fasten your seat belt! ---*/
	 // Select the main PLL as system clock source
	 RCC->CFGR &= ~RCC_CFGR_SW;
	 RCC->CFGR |= RCC_CFGR_SW_PLL;
	 // Wait until PLL becomes main switch input
	 do
	 {
		 SW_Status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		 timeout--;
	 } while ((SW_Status != RCC_CFGR_SWS_PLL) && (timeout > 0));
	 /* --- Here we go! ---*/
	 /*--- Use PA8 as MCO output at 48/16 = 3MHz ---*/
	 // Set MCO source as SYSCLK (48MHz)
	 RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	 RCC->CFGR |=  RCC_CFGR_MCOSEL_SYSCLK;
	 // Set MCO prescaler to /16 -> 3MHz
	 RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	 RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV16;
	 // Enable GPIOA clock
	 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	 // Configure PA8 as Alternate function
	 GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	 GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);
	 // Set to AF0 (MCO output)
	 GPIOA->AFR[1] &= ~(0x0000000F);
	 GPIOA->AFR[1] |=  (0x00000000);
	 // Update SystemCoreClock global variable
	 SystemCoreClockUpdate();
 }
 