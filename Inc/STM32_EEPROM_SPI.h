/* Copyright 2017 Nikita Bulaev
 *
 */


#ifndef _STM32_EEPROM_SPI_H
#define _STM32_EEPROM_SPI_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "cmsis_os.h"

#define EEPROM_SPI   hspi1

#define EEPROM_WREN  0x06  // Write Enable
#define EEPROM_WRDI  0x04  // Write Disable
#define EEPROM_RDSR  0x05  // Read Status Register
#define EEPROM_WRSR  0x01  // Write Status Register
#define EEPROM_READ  0x03  // Read from Memory Array
#define EEPROM_WRITE 0x02  // Write to Memory Array

/**
 * @brief  M95040 SPI EEPROM defines
 */
#define EEPROM_WIP_FLAG        0x01  /*!< Write In Progress (WIP) flag */

#define EEPROM_PAGESIZE           32

#define EEPROM_CS_HIGH()    HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_SET)
#define EEPROM_CS_LOW()     HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_RESET)

typedef enum {
    EEPROM_STATUS_PENDING,
    EEPROM_STATUS_COMPLETE,
    EEPROM_STATUS_ERROR
} EepromOperations;

extern void Error_Handler(void);


EepromOperations EEPROM_SPI_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint8_t EEPROM_SPI_WaitStandbyState(void);

/**
  * @brief  Low layer functions
  */
uint8_t EEPROM_SendByte(uint8_t byte);
void sEE_WriteEnable(void);
void sEE_WriteDisable(void);
void sEE_WriteStatusRegister(uint8_t regval);
uint8_t sEE_ReadStatusRegister(void);

void  EEPROM_SPI_SendInstruction(uint8_t *instruction, uint8_t size);
void  EEPROM_SPI_ReadStatusByte(SPI_HandleTypeDef SPIe, uint8_t *statusByte );

#endif // _STM32_EEPROM_SPI_H
