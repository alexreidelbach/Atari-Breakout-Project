/**
  ******************************************************************************
  * @file    main.c
  * @author  Eli Ade
  * @date    April 4th, 2025
  * @brief   ECE 362 Project Screen
  ******************************************************************************
*/
#include "stm32f0xx.h"
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
#include <stdint.h>
#include "lcd.h"
#include "commands.h"
#include <math.h>

//---------------------------------------------------------------------
// Breakout paddle and keypad
//---------------------------------------------------------------------
#define SCREEN_WIDTH 320

// Paddle state globals
volatile int paddle_x = 220;  // This moves the paddle up and down
volatile int paddle_y = 140;  // This moves the paddle left and right
volatile int paddle_width = 5;
volatile int paddle_height = 40;

uint16_t score = 0;
uint16_t h_score = 0;

// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700};
extern uint8_t font[];

// A helper to allow commands.c to get the paddle state.
void get_paddle(int *x, int *y, int *width, int *height) {
    *x = paddle_x;
    *y = paddle_y;
    *width = paddle_width;
    *height = paddle_height;
}

void nano_wait(int);
int joystick_position = 0;
volatile int game_in_progress = 0;

void remote(int *p) {
    *p = game_in_progress;
}

/// JOYSTICK LOGIC 
void StartGame(){
    //TitleStart = 0;
    //GPIOC -> ODR |= 1 << 6;
    game_in_progress = 1;    
}


void PauseGame (){
    //GPIOC -> ODR &= ~(1 << 6);
    game_in_progress = 0;
}

void setup_GPIO (){ //Use PA3 for switch connection (Alex: I'm not sure what this is)
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC -> AHBENR |= RCC_APB2ENR_SYSCFGCOMPEN; // (Alex: i think this is needed for external interrupts)
    GPIOA -> MODER &= ~(GPIO_MODER_MODER3);
    GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR3_0;
    SYSCFG -> EXTICR[0] = 0x0; 
    EXTI -> FTSR |= EXTI_FTSR_TR3;
    EXTI -> IMR |= EXTI_IMR_IM3;
    NVIC -> ISER[0] |= (1 << EXTI2_3_IRQn);
}

void EXTI2_3_IRQHandler (){ //Use for pause and start in the game selecting game mode potentially?
    EXTI -> PR = EXTI_PR_PR3;
    if(game_in_progress){
        PauseGame();
    }
    else {
        StartGame();
    }
  }

//ADC for Joystick
void setup_adc (){ //Uses PA1
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA -> MODER |= GPIO_MODER_MODER1;
    RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1 -> CR &= ~(ADC_CR_ADEN);
    ADC1 -> CFGR1 |= ADC_CFGR1_RES; //Resolution to 6 bits (Change if needed)
    ADC1 -> CR |= ADC_CR_ADEN;
    RCC -> CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    ADC1 -> CR |= ADC_CR_ADEN;
    while(!(ADC1-> ISR & ADC_ISR_ADRDY));
    ADC1 -> CHSELR = 0;
    ADC1 -> CHSELR |= 1 << 1;
    while(!(ADC1-> ISR & ADC_ISR_ADRDY));
}

void init_tim2(void) { //Timer for triggering interrupt for joystick reading
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN; 
    TIM2 -> PSC = 48000-1; 
    TIM2 -> ARR = 16-1; //62.5 fps
    TIM2 -> DIER |= TIM_DIER_UIE;
    NVIC -> ISER[0] = 1 << TIM2_IRQn;
    TIM2 -> CR1 |= TIM_CR1_CEN;
}


void TIM2_IRQHandler (){ //Read joystick position              
    TIM2 -> SR &= ~TIM_SR_UIF;
    ADC1 -> CR |= ADC_CR_ADSTART;
    while(!(ADC1 -> ISR & ADC_ISR_EOC));
    joystick_position = ADC1->DR;
    if (joystick_position < 25) {
        if (paddle_y > 7) {
            paddle_y -= 2;
        }
    } else if (joystick_position > 35) {
        if (paddle_y < (SCREEN_WIDTH - paddle_width - 43))
            paddle_y += 2;
    }
    // Look into debounce for button
    nano_wait(20000);
}

// // --- Keypad globals ---
// // We define a simple keymap array; arranged column-wise (4 columns, 4 rows)
// extern char keymap;
// char* keymap_arr = &keymap;

// volatile uint8_t col = 0; // used in scanning columns

// // Initialize keypad GPIO ports.
// // (Note: these settings come from your lab code in main(1).c)
// void enable_keypad_ports() {
//     // Enable GPIOB and GPIOC clocks.
//     RCC->AHBENR |= (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN);
    
//     // Configure PB0-10 as outputs (for keypad driving)
//     GPIOB->MODER &= ~(0x3FFFFF);      // Clear mode bits for PB0-PB10
//     GPIOB->MODER |=  (0x155555);       // Set as outputs (01 for each)
    
//     // Configure PC4-7 as outputs (for keypad column driving)
//     GPIOC->MODER &= ~(0xFF << (4 * 2));
//     GPIOC->MODER |=  (0x55 << (4 * 2));
    
//     // Configure PC0-3 as inputs (for keypad rows) with pull-down resistors
//     GPIOC->MODER &= ~(0xFF << (0 * 2));
//     GPIOC->PUPDR &= ~(0xFF << (0 * 2));
//     GPIOC->PUPDR |=  (0xAA << (0 * 2)); // 10: pull-down
// }

// // Drive the specified keypad column.
// void drive_column(int c) {
//     c &= 0x3; // use only lower 2 bits (columns 0-3)
//     // Clear PC4-7 output bits:
//     GPIOC->BSRR = (0xF << 20);
//     // Set the appropriate column pin (PC4, PC5, PC6, or PC7)
//     GPIOC->BSRR = (1 << (4 + c));
// }

// // Read the 4-bit row value from PC0-3.
// int read_rows() {
//     uint32_t idr_value = GPIOC->IDR;
//     int row_data = idr_value & 0xF;
//     return row_data;
// }

// // Given the row bits, determine which key is pressed.
// char rows_to_key(int rows) {
//     if (rows == 0)
//         return '\0';
    
//     int row_index = -1;
//     if (rows & 0x1) {
//         row_index = 0;
//     } else if (rows & 0x2) {
//         row_index = 1;
//     } else if (rows & 0x4) {
//         row_index = 2;
//     } else if (rows & 0x8) {
//         row_index = 3;
//     }
    
//     if (row_index == -1)
//         return '\0';
    
//     // Use the current column (col global variable) to compute offset.
//     int offset = (col * 4) + row_index;
//     char key = keymap_arr[offset];
//     return key;
// }

// // Handle a pressed key by updating the paddle.
// // For our Breakout game we use key '4' to move left and '6' to move right.
// void handle_key(char key) {
//     if (key == '*') {
//         pause = 1;
//         resume = 0;
//         paddle_y = paddle_y;
//     } else if (key == '0') {
//         resume = 1;
//         pause = 0;
//     } 
//     //printf("%c", key);
// }

// // TIM7 interrupt handler: scans the keypad.
// void TIM7_IRQHandler(void) {
//     // Acknowledge the interrupt.
//     TIM7->SR &= ~TIM_SR_UIF;
    
//     int rows = read_rows();
//     if (rows != 0) {
//         char key = rows_to_key(rows);
//         handle_key(key);
//     }
    
//     // Cycle to the next column (we assume 4 columns).
//     col = (col + 1) % 4;
//     drive_column(col);
// }

// // Setup Timer 7 for keypad scanning (1 kHz rate).
// void setup_tim7(void) {
//     RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
//     TIM7->PSC = 47;        // Prescaler for 1 kHz (assuming 48MHz clock)
//     TIM7->ARR = 999;       // 1 ms period
//     TIM7->DIER |= TIM_DIER_UIE;
//     NVIC_EnableIRQ(TIM7_IRQn);
//     TIM7->CR1 |= TIM_CR1_CEN;
// }

//---------------------------------------------------------------------
// End of keypad/paddle
//---------------------------------------------------------------------

//*****************************************************************************
// 2.1 Configure SPI1 in slow mode for SD card initialization.
//*****************************************************************************
void init_spi1_slow(void) {
    // Enable clocks for SPI1 and GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    
    // Configure PB3 (SCK), PB4 (MISO), and PB5 (MOSI) for alternate function mode.
    // Set these pins to alternate function mode (10).
    GPIOB->MODER &= ~((3 << (2 * 3)) | (3 << (2 * 4)) | (3 << (2 * 5)));
    GPIOB->MODER |=  ((2 << (2 * 3)) | (2 << (2 * 4)) | (2 << (2 * 5)));
    
    // Set alternate function to AF0 (default for SPI1) for PB3, PB4, and PB5.
    GPIOB->AFR[0] &= ~((0xF << (4 * 3)) | (0xF << (4 * 4)) | (0xF << (4 * 5)));
    
    // Disable SPI1 while configuring.
    SPI1->CR1 &= ~SPI_CR1_SPE;
    
    // Set the baud rate divisor to maximum (BR = 7 -> fPCLK/256) to achieve a slow SPI clock.
    SPI1->CR1 &= ~(7 << SPI_CR1_BR_Pos);
    SPI1->CR1 |=  (7 << SPI_CR1_BR_Pos);
    
    // Configure SPI1 as Master.
    SPI1->CR1 |= SPI_CR1_MSTR;
    
    // Enable Software Slave Management and set Internal Slave Select.
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    
    // Configure the data frame to 8-bit.
    // For STM32F0, the data size is set in CR2 using the DS field.
    //SPI1->CR2 &= ~SPI_CR2_DS;
    //SPI1->CR2 |= (7 << SPI_CR2_DS_Pos); // 7 means 8-bit (DS+1 = 8)
    SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    SPI1->CR2 &= ~SPI_CR2_DS_3;
    
    // Set FIFO reception threshold to release received 8-bit values immediately.
    SPI1->CR2 |= SPI_CR2_FRXTH;
    
    // Finally, enable the SPI1 peripheral.
    SPI1->CR1 |= SPI_CR1_SPE;
}

//*****************************************************************************
// 2.2 Enable SD Card: set PB2 low.
//*****************************************************************************
void enable_sdcard(void) {
    // Setting PB2 low enables the SD card.
    GPIOB->BSRR = GPIO_BSRR_BR_2;
}

//*****************************************************************************
// 2.3 Disable SD Card: set PB2 high.
//*****************************************************************************
void disable_sdcard(void) {
    // Setting PB2 high disables the SD card.
    GPIOB->BSRR = GPIO_BSRR_BS_2;
}

//*****************************************************************************
// 2.4 Initialize SD card I/O.
//*****************************************************************************
void init_sdcard_io(void) {
    // Start with the slow SPI configuration.
    init_spi1_slow();
    
    // Enable the clock for GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    
    // Configure PB2 as a general-purpose output
    GPIOB->MODER &= ~(3 << (2 * 2));
    GPIOB->MODER |=  (1 << (2 * 2));
    
    // Initially disable the SD card.
    disable_sdcard();
}

//*****************************************************************************
// 2.5 Increase SPI1 speed for SD card I/O (and LCD if shared).
//*****************************************************************************
void sdcard_io_high_speed(void) {
    // Disable SPI1 before changing its settings.
    SPI1->CR1 &= ~SPI_CR1_SPE;
    
    // Set SPI1 baud rate for a ~12 MHz clock.
    // For STM32, divisor = 2^(BR+1); so for 48/4 = 12, BR should be 1.
    SPI1->CR1 &= ~(7 << SPI_CR1_BR_Pos);
    SPI1->CR1 |=  (1 << SPI_CR1_BR_Pos);
    
    // Re-enable SPI1.
    SPI1->CR1 |= SPI_CR1_SPE;
}

//*****************************************************************************
// 2.7 Initialize the LCD SPI interface.
//*****************************************************************************
void init_lcd_spi(void) {
    // Enable clock for GPIOB.
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    
    // Configure PB8, PB11, and PB14 as GPIO outputs.
    // Clear mode bits and set them to output mode (01).
    GPIOB->MODER &= ~((3 << (2 * 8)) | (3 << (2 * 11)) | (3 << (2 * 14)));
    GPIOB->MODER |=  ((1 << (2 * 8)) | (1 << (2 * 11)) | (1 << (2 * 14)));
    
    // Use the slow SPI initialization as a starting point.
    init_spi1_slow();
    
    // Switch to high speed for the LCD interface.
    sdcard_io_high_speed();
}


///////////////////////////////////////////////////////////
//                 USART CODE
///////////////////////////////////////////////////////////
void init_usart5() {
    // Enable clocks for GPIOC and GPIOD
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    // Configure PC12 as Alternate Function for USART5_TX
    GPIOC->MODER &= ~GPIO_MODER_MODER12;           // Clear PC12 mode bits
    GPIOC->MODER |= GPIO_MODER_MODER12_1;            // Set PC12 to Alternate Function mode
    GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL12;             // Clear alternate function selection for PC12
    GPIOC->AFR[1] |= (2 << GPIO_AFRH_AFSEL12_Pos); // Set AF8 (USART5) for PC12

    // Configure PD2 as Alternate Function  for USART5_RX
    GPIOD->MODER &= ~GPIO_MODER_MODER2;              // Clear PD2 mode bits
    GPIOD->MODER |= GPIO_MODER_MODER2_1;               // Set PD2 to Alternate Function mode
    GPIOD->AFR[0] &= ~GPIO_AFRL_AFSEL2;              // Clear alternate function selection for PD2
    GPIOD->AFR[0] |= (2 << GPIO_AFRL_AFSEL2_Pos); // Set AF8 for PD2

    // Enable clock for USART5
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    // Disable USART5 by clearing the UE bit
    USART5->CR1 &= ~USART_CR1_UE;

    /* Configure USART5:
       - 8-bit word length (clear both M0 and M1 bits)
       - 1 stop bit (STOP bits cleared)
       - No parity (PCE cleared)
       - 16x oversampling (OVER8 cleared)
    */
    USART5->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);
    USART5->CR2 &= ~USART_CR2_STOP;
    USART5->CR1 &= ~USART_CR1_PCE;
    USART5->CR1 &= ~USART_CR1_OVER8;

    // Set baud rate to 115200.
    USART5->BRR = 0x1A1;

    // Enable transmitter and receiver
    USART5->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Enable the USART
    USART5->CR1 |= USART_CR1_UE;

    // Wait until TEACK and REACK bits in the ISR are both set, indicating that the transmitter and receiver are ready
    while (!(USART5->ISR & (USART_ISR_TEACK | USART_ISR_REACK)))
    {
    }
}

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void enable_tty_interrupt(void) {
    /* Enable RCC clock for DMA2 */
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    
    // Remap DMA2 Channel 2 to USART5_RX.
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    
    // Disable DMA2 Channel 2 before configuration
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    
    // Set the peripheral address (CPAR) to USART5 RDR address
    DMA2_Channel2->CPAR = (uint32_t)&USART5->RDR;
    // Set the memory address (CMAR) to the serfifo buffer
    DMA2_Channel2->CMAR = (uint32_t)serfifo;
    // Set the number of data items to transfer
    DMA2_Channel2->CNDTR = FIFOSIZE;
    
    /* Configure DMA2_Channel2:
         - Peripheral-to-memory (DIR = 0)
         - Memory increment mode enabled (MINC)
         - Peripheral increment mode disabled (PINC = 0)
         - Data sizes: both peripheral and memory are 8 bits (PSIZE/MSIZE = 0)
         - Circular mode enabled (CIRC)
         - Highest priority (set PL bits to 11)
         - Transfer complete and half-transfer interrupts remain disabled
         - Memory-to-memory mode disabled (MEM2MEM = 0)
    */
    DMA2_Channel2->CCR &= ~DMA_CCR_DIR;
    // Size settings: both PSIZE and MSIZE are 0 for 8-bit transfers.
    DMA2_Channel2->CCR |= DMA_CCR_MINC;   // Memory increment enabled
    // Do NOT set DMA_CCR_PINC; CPAR remains fixed.
    DMA2_Channel2->CCR |= DMA_CCR_CIRC;   // Enable circular mode

    // Set channel priority to highest.
    // First, clear the priority bits (usually bits 12-13)
    DMA2_Channel2->CCR &= ~(0x3 << 12);
    // Then set them to 0b11 (highest priority)
    DMA2_Channel2->CCR |= (0x3 << 12);

    // Finally, enable the DMA channel
    DMA2_Channel2->CCR |= DMA_CCR_EN;
    
    // Enable RXNE interrupt
    USART5->CR1 |= USART_CR1_RXNEIE;
    // Enable DMA mode for reception
    USART5->CR3 |= USART_CR3_DMAR;
    // Enable the NVIC interrupt for USART5
    NVIC_EnableIRQ(USART3_8_IRQn);
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    while (!fifo_newline(&input_fifo))
    {
        asm volatile ("wfi"); // Wait For Interrupt
    }
    return fifo_remove(&input_fifo);
}

int __io_putchar(int c) {
    // If the character is a newline, first send a carriage return.
    if(c == '\n')
    {
        // Wait for TXE flag to be set before transmitting
        while(!(USART5->ISR & USART_ISR_TXE)) {}
        USART5->TDR = '\r';
    }
    // Wait for TXE flag to be set before sending the actual character
    while(!(USART5->ISR & USART_ISR_TXE)) {}
    USART5->TDR = c;
    
    return c;
}

int __io_getchar(void) {
    return interrupt_getchar();
}

// TODO Copy the content for the USART5 ISR here
void USART3_8_IRQHandler(void) {
    // The DMA channel is continuously transferring received characters into serfifo.
    // DMA2_Channel2->CNDTR is the number of data items left to be transferred before it resets.
    // When new characters arrive, CNDTR decreases. We check for any new characters by comparing
    // the current CNDTR to what it would be if no new characters had been received.
    while (DMA2_Channel2->CNDTR != (sizeof(serfifo) - seroffset)) {
        // If the input FIFO is not full, copy the new character and echo it.
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        
        // Update our offset in the circular buffer, wrapping around if needed.
        seroffset = (seroffset + 1) % sizeof(serfifo);
    }
}
//===========================================================================
// Get score from gameplay and configure it to be displayed on ssd.
//===========================================================================
/// Make sure that your ssd is configured like in lab6-spi for this to work!
void print(const char str[])
{
    const char *p = str;
    for(int i=0; i<8; i++) {
        if (*p == '\0') {
            msg[i] = (i<<8);
        } else {
            msg[i] = (i<<8) | font[*p & 0x7f] | (*p & 0x80);
            p++;
        }
    }
}

void write_display() {
    char buff[10];
    if (game_in_progress == 0) {
        snprintf(buff, sizeof(buff), "SCR: %3d", score);
    } else {
        snprintf(buff, sizeof(buff), "HSC: %3d", h_score);
    }
    print(buff);
}


//===========================================================================
// Initialize the SPI2 peripheral.
//===========================================================================
void init_spi2(void) {
    SPI2->CR1 &= ~(0x1 << 6);
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~((0xF << 24) | (0x3 << 30)); 
    GPIOB->MODER |= (0xA << 24) | (0x2 << 30);
    GPIOB->AFR[1] &= ~((0xFF << 16 ) | (0xF << 28));
    RCC->APB1ENR |= (0x1 << 14);
    SPI2->CR1 |= (0x7 << SPI_CR1_BR_Pos);
    SPI2->CR2 |= (0b1111 << 8);
    SPI2->CR1 |= (0x1 << SPI_CR1_MSTR_Pos);
    SPI2->CR2 |= (0x1 << SPI_CR2_SSOE_Pos);
    SPI2->CR2 |= (0x1 << SPI_CR2_NSSP_Pos);
    SPI2->CR2 |= (0x1 << SPI_CR2_TXDMAEN_Pos);
    SPI2->CR1 |= (0x1 << 6);

}

//===========================================================================
// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.  Use the code from setup_dma from lab 5.
//===========================================================================
void spi2_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR &= ~0x1;
    DMA1_Channel5->CMAR = (uint32_t) &msg;
    DMA1_Channel5->CPAR = (uint32_t) &(SPI2->DR);
    DMA1_Channel5->CNDTR = 8;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR &= ~DMA_CCR_PINC;
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
    SPI2->CR2 |= 0x1 << SPI_CR2_TXDMAEN_Pos;
    DMA1_Channel5->CCR |= 0x1; // enable dma channel
}

//============================================================================
// setup_adc() for music
//============================================================================
// void setup_adc2(void) {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//     GPIOA->MODER |= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1;
//     RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
//     RCC->CR2 |= RCC_CR2_HSI14ON;
//     while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
//     ADC1->CR |= ADC_CR_ADEN;
//     while(!(ADC1->ISR & ADC_ISR_ADRDY));
//     ADC1->CHSELR = ADC_CHSELR_CHSEL2;
//     while(!(ADC1->ISR & ADC_ISR_ADRDY));
// }

//============================================================================
// Varables for boxcar averaging.
//============================================================================
#define BCSIZE 32
// int bcsum = 0;
// int boxcar[BCSIZE];
// int bcn = 0;
uint32_t volume = 2048;

//============================================================================
// Timer 14 ISR for music
//============================================================================
// void TIM14_IRQHandler(void) {
//     TIM14->SR &= ~TIM_SR_UIF;
//     ADC1->CR |= ADC_CR_ADSTART;
//     while(!(ADC1->ISR & ADC_ISR_EOC));
//     bcsum -= boxcar[bcn];
//     bcsum += boxcar[bcn] = ADC1->DR;
//     bcn += 1;
//     if (bcn >= BCSIZE) {
//         bcn = 0;
//     }
//     volume = bcsum / BCSIZE;
// }

//============================================================================
// init_tim14() for music
//============================================================================
// void init_tim14(void) {
//     RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
//     TIM14->PSC = 4800 - 1;
//     TIM14->ARR = 1000 - 1;
//     TIM14->DIER |= TIM_DIER_UIE;
//     NVIC->ISER[0] = 1 << TIM2_IRQn;
//     TIM14->CR1 |= TIM_CR1_CEN;
// }

//===========================================================================
//Create an analog sine wave of a specified frequency
//===========================================================================
void dialer(void);

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

//===========================================================================
// init_wavetable()
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
//===========================================================================
void init_wavetable(void) {
    for(int i=0; i < N; i++) {
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
    }
}

//============================================================================
// set_freq()
//============================================================================
void set_freq(int chan, float f) {
    if (chan == 1) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 2) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 3<<(2*4);
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR &= ~(DAC_CR_TSEL1);
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_EN1;

}

//============================================================================
// Timer 6 ISR
//============================================================================
void TIM6_DAC_IRQHandler(void) {
    TIM6->SR &= ~TIM_SR_UIF;
    offset0 += step0;
    offset1 += step1;
    if (offset0 >= (N << 16)) {
        offset0 -= (N << 16);
    }
    if (offset1 >= (N << 16)) {
        offset1 -= (N << 16);
    }
    int samp = wavetable[offset0>>16] + wavetable[offset1>>16] / 2;
    samp = samp * volume;
    samp = (samp >> 0x11);
    samp += 2048;
    DAC->DHR12R1 = samp;
}

//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->ARR = 100 - 1;
    TIM6->PSC = (480000/(RATE)) - 1;
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR2 |= TIM_CR2_MMS_1;
    NVIC->ISER[0] = 1 << TIM6_IRQn;
}
//============================================================================
// Building song
//============================================================================
typedef struct {
    uint8_t ch;
    float freq;
    uint16_t duration;
} Note;

typedef struct {
    const Note* notes;
    uint8_t length;
} Song;

const Note songA_arr[] = {
    {2, 0, 10},
    {1, 262, 250}, 
    {2, 0, 10},
    {1, 330, 250}, 
    {2, 0, 10},
    {1, 392, 500}, 
    {2, 0, 10},
    {1, 523, 250}, 
    {2, 0, 10},
    {1, 392, 250}, 
    {2, 0, 10},
    {1, 330, 500}, 
    {2, 0, 10},
    {1, 349, 250}, 
    {2, 0, 10},
    {1, 440, 250}, 
    {2, 0, 10},
    {1, 523, 500}, 
    {2, 0, 10},
};
#define SONG_A_LENTH (sizeof(songA_arr)/sizeof(songA_arr[0]))

const Note songB_arr[] = {
    {2, 950, 100},
};
#define SONG_B_LENGTH (sizeof(songB_arr)/sizeof(songB_arr[0]))

const Note songC_arr[] = {
    {1, 600, 100},
    {2, 800, 100}
};
#define SONG_C_LENGTH (sizeof(songC_arr)/sizeof(songC_arr[0]))

const Note songD_arr[] = {
    {2, 0, 100},
    {1, 700, 100},
    {1, 362, 300},
    {1, 320, 400},
    {1, 296, 400},
    {1, 275, 500},
    {1, 220, 1000},
    {1, 0, 100}

};
#define SONG_D_LENGTH (sizeof(songD_arr)/sizeof(songD_arr[0]))

Song background = {songA_arr, SONG_A_LENTH};
Song breaker = {songB_arr, SONG_B_LENGTH};
Song paddle = {songC_arr, SONG_C_LENGTH};
Song failed = {songD_arr, SONG_D_LENGTH};

volatile Song* currentSong = &songC_arr;
volatile uint8_t song_index = 0;

void init_tim3_song_timer(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 48000 - 1;
    TIM3->ARR = 100 - 1;
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = (1 << TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM3_IRQHandler(void) {
    TIM3->SR &= ~TIM_SR_UIF;
    song_index++;
    if (song_index >= currentSong->length) {
        if (currentSong == &background) {
            song_index = 0;
            set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
            TIM3->ARR = currentSong->notes[song_index].duration - 1;
            TIM3->CNT = 0; 
        }
        else {
        }
    }
    else {
        set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
        TIM3->ARR = currentSong->notes[song_index].duration - 1;
        TIM3->CNT = 0; 
    }
}

// Song functions for commands
void play_background(void) {
    currentSong = &background;
    song_index = 0;
    set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
    TIM3->ARR = currentSong->notes[song_index].duration - 1;
    TIM3->CNT = 0;
}
void play_breaker(void) {
    currentSong = &breaker;
    uint8_t background_index2 = song_index;
    song_index = 0;
    set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
    TIM3->ARR = currentSong->notes[song_index].duration - 1;
    TIM3->CNT = 0;
    currentSong = &background;
    song_index = background_index2;
}
void play_paddle(void) {
    currentSong = &paddle;
    uint8_t background_index = song_index;
    song_index = 0;
    set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
    TIM3->ARR = currentSong->notes[song_index].duration - 1;
    TIM3->CNT = 0;
    currentSong = &background;
    song_index = background_index;
}
void play_fail(void) {
    currentSong = &failed;
    song_index = 0;
    set_freq(currentSong->notes[song_index].ch, currentSong->notes[song_index].freq);
    TIM3->ARR = currentSong->notes[song_index].duration - 1;
    TIM3->CNT = 0;  
}


// Function prototypes for initialization and command shell
void internal_clock(void);
void enable_tty_interrupt(void);
void init_sdcard_io(void);
void init_lcd_spi(void);
void command_shell(void);
void print(const char str[]);

int main(void) {
    // Initialize system clock and TTY interrupts
    internal_clock();
    init_usart5();
    enable_tty_interrupt();

    // Joystick
    setup_adc();
    init_tim2();
    setup_GPIO();

    // For audio
    // setup_adc2();
    // init_tim14();
    init_wavetable();
    setup_dac();
    init_tim6();
    set_freq(1,0);
    set_freq(2,0);
    init_tim3_song_timer();

    // Turn off buffering to speed up I/O
    setbuf(stdin, 0);
    setbuf(stdout, 0);
    setbuf(stderr, 0);

    // Prompt for the current date
    // printf("Enter current date (YYYYMMDDHHMMSS): ");
    // char date[16];
    // fgets(date, sizeof(date), stdin);

    // Initialize SD card I/O: sets up SPI1 in slow mode and configures PB2 as chip select
    init_sdcard_io();

    // Initialize LCD SPI interface:
    // This configures additional GPIO pins and then calls SPI initialization functions.
    init_lcd_spi();

    //
    init_spi2();
    spi2_setup_dma();
    write_display();
    

    // Inform the user about the command shell environment
    printf("This is the STM32 command shell.\n");
    printf("Type 'mount' before trying any file system commands.\n");
    printf("Type 'lcd_init' before trying any draw commands.\n");

    // Transfer control to the command shell
    lcd_init(NULL, NULL);
    game(NULL, NULL);

    for (;;) {
        char c = getchar();
        putchar(c);
    }
}