#pragma once
#include "stm32g0xx_hal.h"  // or whichever STM32 series you're using

// ======== VNQ7003 Pin Definitions ========
#define VNQ_CS_GPIO_Port GPIOA
#define VNQ_CS_Pin       GPIO_PIN_4

// ======== SPI Handle Declaration ========
// Declare the SPI handle used for the VNQ7003
extern SPI_HandleTypeDef hspi1;
