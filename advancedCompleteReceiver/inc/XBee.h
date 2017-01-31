#ifndef XBEE_LIBRARY
#define XBEE_LIBRARY

//These are the Includes
#include <string.h>
#include <stdbool.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_exti.h>
#include <stm32f0xx_syscfg.h>

#include "SysTickDelay.h"
#include "SPI1.h"
#include "IndicationGPIOs.h"
#include "USART1.h"
/*
 * Xbee defines
 */
#define AT_COMMAND_RESPONSE 0x88
#define RECIEVE_PACKET 0x90
#define MODEM_STATUS 0x8A
#define TRANSMIT_STATUS 0x8B
#define AT_COMMAND_DATA_INDEX 0x05
#define ASCII_DIGIT_OFFSET 0x30

#define TRANSOPT_DISACK 0x00
#define TRANSOPT_DIGIMESH 0xC0

// Defines for xbee FRAME_ID
#define XBEE_TYPE_OF_FRAME_INDEX 0
#define XBEE_FRAME_ID_INDEX 1
#define XBEE_TRANSMIT_STATUS_DELIVERY_INDEX 0x05
#define XBEE_AT_COMMAND_INDEX 2
#define XBEE_AT_COMMAND_STATUS 4
#define XBEE_AT_COMMAND_DATA 5
#define AT_FRAME_ID_REQUEST 0x52
#define AT_FRAME_ID_APPLY   0x50
#define XBEE_FN_ADDRESSH_INDEX 7
#define XBEE_FN_ADDRESSL_INDEX 11

//debugging
//#include "debug.h"

//These are the Defines and global variables
extern bool readingPacket;

#define XBEE_CS_LOW() GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define XBEE_CS_HIGH() GPIO_SetBits(GPIOA,GPIO_Pin_4)

//Setup Xbee
void initializeXbeeATTnPin(void);

//Functions for AT commands, parameters
void xbeeApplyDwordParamter(char* atCommand, uint32_t parameter, uint8_t frameID);
void xbeeApplyParamter(char* atCommand, uint8_t parameter, uint8_t frameID);
void askXbeeParam(char* atCommand, uint8_t frameID);

bool xbeeStartupParamRead(uint8_t _packetErrorLimit, uint8_t* _xbeeBuffer);

//Data transmission functions
void transmitRequest(uint32_t adrHigh, uint32_t adrLow, uint8_t transmitOption, uint8_t frameID, char* data);
void transmitRequestBytes(uint32_t adrHigh, uint32_t adrLow, uint8_t transmitOption, uint8_t frameID, uint8_t* data, uint8_t dataSize);

#endif
