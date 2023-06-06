#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include <stdint.h>

#define MEMORY_SIZE 1024  // Size of memory to test in bytes

#define BUTTON_PIN     GPIO_Pin_0
#define BUTTON_PORT    GPIOA

void delay(uint32_t time) {
  while (time--) {
    __NOP();
  }
}

// Function to test the CPU
uint32_t cpuTest(void) {
  // Arithmetic operations
  uint32_t a = 10;
  uint32_t b = 5;
  uint32_t sum = a + b;
  uint32_t diff = a - b;
  uint32_t product = a * b;
  uint32_t quotient = a / b;

  // Logic operations
  uint32_t logicalAnd = a & b;
  uint32_t logicalOr = a | b;
  uint32_t logicalXor = a ^ b;

  // Verify results
  if (sum == 15 && diff == 5 && product == 50 && quotient == 2 &&
      logicalAnd == 0 && logicalOr == 15 && logicalXor == 15) {
    // CPU test passed
    return 1;
  } else {
    // CPU test failed
    return 0;
  }
}

// Function to test memory
uint32_t memoryTest(void) {
  uint8_t memory[MEMORY_SIZE];

  // Write test pattern to memory
  for (uint32_t i = 0; i < MEMORY_SIZE; i++) {
    memory[i] = 0xAA;  // Write a test pattern
  }

  // Read and verify memory contents
  for (uint32_t i = 0; i < MEMORY_SIZE; i++) {
    if (memory[i] != 0xAA) {
      // Memory test failed
      return 0;
    }
  }

  // Memory test passed
  return 1;
}


void UART_Configuration(void) {
  // Enable USART2 and GPIOA peripheral clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  // Configure PA2 and PA3 as USART2 pins
  GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);
  GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1;

  // Configure USART2 for desired baud rate, data bits, parity, and stop bits
  USART2->BRR = 0x0683;  // Baud rate of 9600 with APB1 clock at 16 MHz
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void UART_SendData(uint8_t data) {
  while (!(USART2->SR & USART_SR_TXE))
    ;  // Wait until transmit data register is empty
  USART2->DR = data;
}

uint8_t UART_ReceiveData(void) {
  while (!(USART2->SR & USART_SR_RXNE))
    ;  // Wait until receive data register is not empty
  return USART2->DR;
}


void UART_Configuration(void) {
  // Enable USART2 and GPIOA peripheral clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  // Configure PA2 and PA3 as USART2 pins
  GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);
  GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1;

  // Configure USART2 for desired baud rate, data bits, parity, and stop bits
  USART2->BRR = 0x0683;  // Baud rate of 9600 with APB1 clock at 16 MHz
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void UART_SendData(uint8_t data) {
  while (!(USART2->SR & USART_SR_TXE))
    ;  // Wait until transmit data register is empty
  USART2->DR = data;
}

uint8_t UART_ReceiveData(void) {
  while (!(USART2->SR & USART_SR_RXNE))
    ;  // Wait until receive data register is not empty
  return USART2->DR;
}

void I2C_Configuration(void) {
  // Enable I2C1 and GPIOB peripheral clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  // Configure PB6 (SCL) and PB9 (SDA) as I2C1 pins
  GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE9_1;
  GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos) | (4 << GPIO_AFRL_AFSEL9_Pos);

  // Configure I2C1 for desired clock speed, addressing mode, and acknowledgment
  I2C1->CR1 = 0;
  I2C1->CR2 = 0;
  I2C1->CR2 |= (8 << I2C_CR2_FREQ_Pos);  // Set clock frequency to 8 MHz
  I2C1->CCR = 80;                        // Set CCR value for desired I2C clock speed
  I2C1->TRISE = 17;                      // Set TRISE value for desired I2C clock speed
  I2C1->CR1 |= I2C_CR1_PE;               // Enable I2C1 peripheral
}

void I2C_SendData(uint8_t address, uint8_t* data, uint8_t length) {
  // Send START condition and wait until it is sent successfully
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;

  // Send slave address and wait until it is sent successfully
  I2C1->DR = address;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;

  // Clear ADDR flag by reading SR1 and SR2 registers
  volatile uint32_t dummyRead = I2C1->SR1 | I2C1->SR2;

  // Send data bytes
  for (uint8_t i = 0; i < length; i++) {
    while (!(I2C1->SR1 & I2C_SR1_TXE))
      ;  // Wait until transmit buffer is empty
    I2C1->DR = data[i];
  }

  // Wait until all data bytes are sent successfully
  while (!(I2C1->SR1 & I2C_SR1_BTF))
    ;

  // Send STOP condition
  I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_ReceiveData(uint8_t address, uint8_t* data, uint8_t length) {
  // Send START condition and wait until it is sent successfully
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;

  // Send slave address and wait until it is sent successfully
  I2C1->DR = address | 0x01;  // Set R/W bit to Read (LSB: 1)
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;

  // Clear ADDR flag by reading SR1 and SR2 registers
  volatile uint32_t dummyRead = I2C1->SR1 | I2C1->SR2;

  // Receive data bytes
  for (uint8_t i = 0; i < length; i++) {
    if (i == length - 1) {
      // Disable ACK before receiving the last byte
      I2C1->CR1 &= ~I2C_CR1_ACK;
    }
    while (!(I2C1->SR1 & I2C_SR1_RXNE))
      ;                  // Wait until receive buffer is not empty
    data[i] = I2C1->DR;  // Read received data
  }

  // Send STOP condition
  I2C1->CR1 |= I2C_CR1_STOP;
}


// Function to send I2C start condition
void I2C_Start(void) {
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
        ;
}

// Function to send I2C stop condition
void I2C_Stop(void) {
    I2C_GenerateSTOP(I2C1, ENABLE);
}

// Function to send I2C address and select transmit mode
void I2C_SendAddress(uint8_t address) {
    I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;
}

// Function to send data byte over I2C
void I2C_SendDataByte(uint8_t data) {
    I2C_SendData(I2C1, data);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;
}

// Function to receive data byte over I2C
uint8_t I2C_ReceiveDataByte(void) {
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
        ;
    return I2C_ReceiveData(I2C1);
}

// Function to perform power supply and voltage testing
void perform_power_supply_test(void) {
    uint8_t voltageData;

    // Start I2C communication
    I2C_Start();

    // Send slave address for writing
    I2C_SendAddress(I2C_SLAVE_ADDR);

    // Send command to read voltage value
    I2C_SendDataByte(0x01);  // Assuming command 0x01 for voltage reading

    // Restart I2C communication
    I2C_Start();

    // Send slave address for reading
    I2C_SendAddress(I2C_SLAVE_ADDR | 0x01);

    // Read voltage data
    voltageData = I2C_ReceiveDataByte();

    // Stop I2C communication
    I2C_Stop();

    // Perform tests on voltage data
    if (voltageData >= 2 && voltageData <= 3.6) {
        printf("Voltage is acceptable");
    } else {
        printf("Voltage unacceptable");
    }
}


volatile uint32_t buttonPressed = 0;

void GPIO_Init(void) {
    // Enable GPIO clock for the button pin
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Configure button pin as input
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
}

void EXTI_Init(void) {
    // Enable SYSCFG clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Connect the button pin to EXTI line
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    // Configure the EXTI line for the button pin
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Trigger on falling edge
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable and set the EXTI interrupt in NVIC (Nested Vector Interrupt Controller)
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; // Set the highest priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        // Clear the interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line0);

        // Set the buttonPressed flag
        buttonPressed = 1;
    }
}

void perform_interrupt_test(void) {
    while (1) {
        if (buttonPressed) {
            // Button interrupt occurred
            printf("Button pressed");

            // Reset the buttonPressed flag
            buttonPressed = 0;
        }
    }
}

volatile uint32_t systemTicks = 0;

void SysTick_Init(void) {
    // Configure SysTick to generate an interrupt every millisecond
    SysTick_Config(SystemCoreClock / 1000); // SystemCoreClock is the system clock frequency in Hz
}

void SysTick_Handler(void) {
    // Increment system tick count
    systemTicks++;
}

void delay_ms(uint32_t milliseconds) {
    uint32_t startTicks = systemTicks;
    while ((systemTicks - startTicks) < milliseconds) {
        // Wait for the specified number of milliseconds
    }
}

void perform_system_timer_test(void) {
      // Generate a delay of 1 second
    delay_ms(1000);

    // Perform necessary actions after the delay
    toggle_led();

    // Generate another delay of 500 milliseconds
    delay_ms(500);

    // Perform necessary actions after the delay
    toggle_led();
}

void toggle_led(void) {
    // Toggle the LED state
    GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
}

// Compute XOR checksum of data buffer
uint8_t computeChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Validate checksum
int validateChecksum(const uint8_t* data, size_t length, uint8_t checksum) {
    uint8_t computedChecksum = computeChecksum(data, length);
    return (computedChecksum == checksum);
}


int main(void) {
  // Perform the CPU test
  uint32_t cpuTestResult = cpuTest();

  // Check the result and take appropriate actions
  if (cpuTestResult) {
    // CPU test passed, continue with the rest of the program
  } else {
    // CPU test failed, handle the error condition
  }

    // Perform the memory test
  uint32_t memoryTestResult = memoryTest();

  // Check the result and take appropriate actions
  if (memoryTestResult) {
    // Memory test passed, continue with the rest of the program
  } else {
    // Memory test failed, handle the error condition
  }
    UART_Configuration();

  // Perform UART test
  uint8_t testData = 0xAA;
  UART_SendData(testData);
  uint8_t receivedData = UART_ReceiveData();

  if (receivedData == testData) {
    // UART test passed
    // Take appropriate actions
  } else {
    // UART test failed
    // Handle the error condition
  }

    I2C_Configuration();

  // Perform I2C test
  uint8_t testData = 0xAA;
  uint8_t receivedData;
  I2C_SendData(0x50, &testData, 1);
  I2C_ReceiveData(0x50, &receivedData, 1);

  if (receivedData == testData) {
    // I2C test passed
    // Take appropriate actions
  } else {
    // I2C test failed
    // Handle the error condition
  }
    I2C_Init();
    // Perform power supply and voltage tests
    perform_power_supply_test();

    // Initialize GPIO for the button pin
    GPIO_Init();

    // Initialize EXTI for the button pin
    EXTI_Init();

    // Perform interrupt test
    perform_interrupt_test();

     // Initialize SysTick timer
    SysTick_Init();

    // Perform system timer test
    perform_system_timer_test();

    uint8_t data[] = {0x12, 0x34, 0x56, 0x78};
    size_t dataLength = sizeof(data) / sizeof(data[0]);

    // Compute checksum
    uint8_t checksum = computeChecksum(data, dataLength);

    // Print computed checksum
    printf("Computed Checksum: 0x%02X\n", checksum);

    // Validate checksum
    if (validateChecksum(data, dataLength, checksum)) {
        printf("Checksum is valid!\n");
    } else {
        printf("Checksum is not valid!\n");
    }

}
