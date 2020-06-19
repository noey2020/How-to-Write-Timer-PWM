#ifndef _NSC_MAIN_H
#define _NSC_MAIN_H

#include <stdint.h>

/* Standard STM32L1xxx driver headers */
#include "stm32l1xx.h"

/* STM32L1xx Discovery Kit:
    - USER Pushbutton: connected to PA0 (GPIO Port A, PIN 0), CLK RCC_AHBENR_GPIOAEN
    - RESET Pushbutton: connected RESET
    - GREEN LED: connected to PB7 (GPIO Port B, PIN 7), CLK RCC_AHBENR_GPIOBEN 
    - BLUE LED: connected to PB6 (GPIO Port B, PIN 6), CLK RCC_AHBENR_GPIOBEN
    - Linear touch sensor/touchkeys: PA6, PA7 (group 2),  PC4, PC5 (group 9),  PB0, PB1 (group 3)
*/

/* stm32l1.h  line 815. GPIO_TypeDef pointer to GPIO_BASE address 0x40020400 */
#define GPIOB               ((GPIO_TypeDef *) 0x40020400)
//#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define RCC_AHBENR_GPIOBEN  ((uint32_t)0x00000002)        /*!< GPIO port B clock enable */
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM_CCMR1_OC1M_1   ((uint16_t)0x0020)            /*!<Bit 1 line 3770 0x20 = 0b100000 */
#define TIM_CCMR1_OC1M_2   ((uint16_t)0x0040)            /*!<Bit 2 line 3771 0x40 = 0b1000000*/
#define TIM_CCMR1_OC1PE    ((uint16_t)0x0008)            /*!<Output Compare 1 Preload enable line 3766 0x8 = 0b1000 */
#define LED_PIN		(6)

#endif	// Prevent the file from being included more than once.

void GPIO_Clock_Enable(){
		// Enable clock to GPIO port B
		//RCC->AHBENR	|= 0x00000002;
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}

void GPIO_Pin_Init(){
		// Set pin 6 as general-purpose output
		GPIOB->MODER &= ~(0x03<<(2*6));		// Mode mask
		GPIOB->MODER |= 0x02<<(2*6);			// Set pin 6 as digital output
	
		// Set output type as push-pull
		GPIOB->OTYPER &= ~(1<<6);
	
		// Set pin 6 as alternative function 2 (TIM4). Connects as output compare to TIM4
		GPIOB->AFR[0] |= 0x2 << (4*6);
	
		// Set IO output speed
		GPIOB->OSPEEDR &= ~(0x03<<(2*6));
		GPIOB->OSPEEDR |= 0x03<<(2*6);
	
		// Set IO no pull-up pull-down
		GPIOB->PUPDR |= ~(0x00<<(2*6));	
}

void TIM4_Clock_Enable(){
		// Enable clock to TIM4
		RCC->APB1ENR	|= RCC_APB1ENR_TIM4EN;
}

void TIM4_IRQ_handler(void) {
		// Handle a timer 'update' interrupt event
		if(TIM4->SR & TIM_SR_UIF){
				TIM4->SR &= ~(TIM_SR_UIF);
				// Toggle the LED output pin.
				GPIOB->ODR ^= (1<<LED_PIN);
		}
}

void TIM4_Init(){
		TIM4->PSC		= 63;									// Prescaler value
		TIM4->ARR		= 200 - 1;						// Auto-reload value
		//TIM4->CCR1	= 500;								// Compare and output register
		// Bit 7 OC1CE: Output compare 1 clear enable not used
		// Bits 6:4 OC1M: Output compare 1 mode
		// 110: PWM mode 1 - In upcounting, channel 1 is active
		TIM4->CCMR1	|= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;		// (0x20 | 0x40) equivalent to 0b01100000
		// Bit 3 OC1PE: Output compare 1 preload enable
		TIM4->CCMR1	|= TIM_CCMR1_OC1PE;		// Output compare 1 preload enable 0x8 = 0b1000
		// Bits 1:0 CC1S: Capture/Compare 1 selection. 00: CC1 channel is configured as output
		//TIM4->CCMR1	|= 0x00;							// 00: CC1 channel is configured as output
		/* Bits 15:0 CCR1[15:0]: Capture/Compare 1 value. If channel CC1 is configured as output:
		CCR1 is the value to be loaded in the actual capture/compare 1 register (preload value). */
		TIM4->CR1		|= TIM_CR1_ARPE;			// Auto-reload preload enable
		TIM4->CCER	|= TIM_CCER_CC1E;			// Enable output for channel 1
		TIM4->SR		&= ~TIM_SR_UIF;				// Clear the update flag
		TIM4->DIER	|= TIM_DIER_UIE;			// Enable interrupt on update
		TIM4->CR1		= TIM_CR1_CEN;				// Enable timer 4
		NVIC_SetPriority(TIM4_IRQn, 0x03);	// Set EXTI0 priority 3(low priority)
		NVIC_EnableIRQ(TIM4_IRQn);					// Enable EXTI0 interrupt
}

int main(void){
		int i;
		int brightness = 0;
		int stepSize;
	
		GPIO_Clock_Enable();
		GPIO_Pin_Init();

		TIM4_Clock_Enable(); 
		TIM4_Init();
		// Infinite loop
		while(1){
			if((brightness >= 200) || (brightness <= 0))
				stepSize = -stepSize;				// Reverse direction
			brightness += stepSize;				// Change brightness
			TIM4->CCR1 = brightness;			// Set brightness for channel 1
			for(i = 0; i < 10000; i++);		// Short delay
		}
}
