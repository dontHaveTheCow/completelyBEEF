/*
 * STM32 and C libraries
 */
#include <stm32f0xx.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/*
 * DEW| libraries
 */
#include "SPI1.h"
#include "SPI2.h"
#include "SysTickDelay.h"
#include "USART1.h"
#include "USART2.h"
#include "MyStringFunctions.h"
#include "Timer.h"
#include "ADC.h"

#include "XBee.h"
#include "Button.h"
#include "IndicationGPIOs.h"
#include "ADXL362.h"
#include "A2035H.h"
#include "sdCard.h"
#include "accTimer.h"
#include "transmitTimer.h"
/*
 * Module state defines
 */
#define MODULE_NOT_INITIALIZED 0x01
#define MODULE_INITIALIZING 0x02
#define MODULE_IDLE 0x03
#define MODULE_EXPERIMENT_MODE 0x04
#define MODULE_SAFE_TURNOFF 0x05
/*
 * Any other defines
 */
#define RSSI_MESSAGE_0 "1 0"
#define RSSI_MESSAGE_1 "1 1"
#define GPS_MSG_INIT_DELAY 400
#define ACC_MEASUREMENT 0x00
#define RSSI_MEASUREMENT 0x01
#define ERROR_TIMER_COUNT 30
#define STARTUP_ERROR_TIMER_COUNT 5
#define ACC_STATE_CASE 0x00
#define GPS_STATE_CASE 0x01
#define GPS_COORD_CASE 0x02
#define XBEE_DATA_MODE_OFFSET 12
#define XBEE_DATA_TYPE_OFFSET 14
#define TIMER_SYNC_DELAY 106

/*
 * XBEE globals
 */
char xbeeReceiveBuffer[255];
volatile bool xbeeDataUpdated = false;
volatile uint8_t length,errorTimer,cheksum;
bool xbeeReading = false;

uint32_t CoordAddrHigh = 0x0013A200;
uint32_t CoordAddrLow = 0x40E3E13A;
/*
 * Module globals
 */
uint8_t turnOffTimer = 0;
bool SPI1_Busy = false;
uint32_t globalCounter = 0;
bool timerUpdated = false;
int16_t accBuff[5];
uint8_t accBuffValue = 0;
/*
 * GPS globals
 */
char gpsReceiveString[96];
uint8_t gpsReadIterator;
volatile bool gpsDataUpdated = false;
/*
 * Transmitting globals
 */
char xbeeTransmitString[64];
char ts[11] = " ";
char lat[11] = " ";
char latd[2]= " ";
char lon[11]= " ";
char lond[2]= " ";
char fix[2]= "0";
char sats[3]= " ";
char *ptrToNMEA[] = {ts, lat, latd, lon, lond, fix, sats};
char velocityString[6] = " ";
char accString[6];
uint8_t state = 0;

int main(void){
	/*
	 * Local variables for XBEE
	 */
	uint8_t typeOfFrame;
	uint32_t SerialAddrHigh = 0x0013A200;
	uint32_t SerialAddrLow = 0x40E32A94;

	uint32_t receivedAddHigh = 0;
	uint32_t receivedAddLow = 0;
	char xbeeTransmitString[64];
	/*
	 * Local variables for GPS
	 */
	char* ptr;
	char* tmpPtr;
	uint8_t messageIterator;
	bool readingVelocity = true;
	/*
	 * Local variables for Accelerometer
	 */
	//int16_t x = 0;
	/*
	 * Local variables for SD card
	 */
	uint8_t sdBuffer[512];
	uint16_t sdBufferCurrentSymbol = 0;
	uint8_t sector;
	uint32_t mstrDir;
	uint32_t cluster;
	uint32_t filesize;
	uint16_t fatSect, fsInfoSector;
	/*
	 * ADC, Timer and other loco's
	 */
	uint8_t moduleStatus = MODULE_NOT_INITIALIZED;
	uint16_t ADC_value;
	uint8_t errorTimer = 40;
	uint8_t timerWindow = 0xFF;
	uint8_t nodesInNetwork = 0;
	int16_t transmitTimerValue = 0;
	char timerString[16];
	//char timerString[16] = " ";
	char itoaConversionString[8];

	/*
	 * Initializing gpio's
	 */
	initializeEveryRedLed();
	initializeEveryGreenLed();
	setupGpsGpio();
	adcPinConfig();
	initializeXbeeATTnPin();
	/*
	 * Initializing peripherals
	 */
	Usart1_Init(BAUD_9600);
	Usart2_Init(BAUD_4800);
	ConfigureUsart2Interrupt();
	//used for delayMs()
	//not meant for using in interrupt routines
	initialiseSysTick();
	//SPI2 used for accelerometer
	InitialiseSPI2_GPIO();
	InitialiseSPI2();
	//SPI for xbee and sdcard
	InitialiseSPI1_GPIO();
	InitialiseSPI1();
	//ADC is used for battery monitoring
	adcConfig();
	//Timer counter with i=1ms
	Initialize_timer();
	//acc_tim
	Initialize_accTimer();
	accTimer_interrupt_enable();
	//Transmit timer - interrupt disabled by default
	Initialize_transmitTimer();
	transmitTimer_interrupt_enable();

	/*
	 * Setup xbee as it is will be needed anyways
	 * for node to be initialized through network
	 */
	XBEE_CS_LOW();
	while(errorTimer--){
		SPI1_TransRecieve(0x00);
	}
	XBEE_CS_HIGH();
	delayMs(1000);
	XBEE_CS_LOW();
	while(errorTimer--){
		SPI1_TransRecieve(0x00);
	}
	XBEE_CS_HIGH();

	while(1){

		switch (moduleStatus) {
		case MODULE_EXPERIMENT_MODE:
			break;

		case MODULE_NOT_INITIALIZED:
			//Battery blinky blinky
			ADC_value = (ADC_GetConversionValue(ADC1));
			ADC_value = (ADC_value * 330) / 128;
			batteryIndicationStartup(ADC_value);
			break;

		case MODULE_SAFE_TURNOFF:
    		if((state&0x04) >> 2){
    			sdIdleState();
    		}
    		if((state&0x02) >> 1){
    			hibernateGps();
    		}
    		if(state&0x01){
    			/*
    			 * Stop acc
    			 */
    			TIM_Cmd(TIM14,DISABLE);
    		}
    		state = 0;
    		moduleStatus = MODULE_NOT_INITIALIZED;
			break;

		case MODULE_INITIALIZING:
			if(state&0x01){
				errorTimer = 10;
				//SPI2 for ADXL
				initializeADXL362();
				blinkRedLed1();
				while(!return_ADXL_ready() && --errorTimer > 0){
					//wait time for caps to discharge
					delayMs(500);
					initializeADXL362();
					delayMs(500);
					blinkRedLed1();
				}
				if(!errorTimer){
					state &=~(0x01);

					xbeeTransmitString[0] = 'C';
					xbeeTransmitString[1] = ' ';
					xbeeTransmitString[2] = 0x8F;
					xbeeTransmitString[3] = '#';
					xbeeTransmitString[4] = 0x01; //acc not initialized error
					xbeeTransmitString[5] = '\0';
					transmitRequest(SerialAddrHigh, SerialAddrLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
				}
				else{
					TIM_Cmd(TIM14,ENABLE);
				}
			}
			errorTimer = 70;
			if((state&0x02) >> 1){
				turnGpsOn();

				while(!GPIO_ReadInputDataBit(GPS_PORTC,WAKEUP_PIN) && errorTimer > 0){
					turnGpsOn();
					delayMs(1000);
					blinkRedLed2();
					errorTimer--;
					SEND_SERIAL_MSG("Pulling GPS pin...\r\n");
				}

				delayMs(GPS_MSG_INIT_DELAY);
				gps_dissableMessage($GPGSA);
				delayMs(GPS_MSG_INIT_DELAY);
				gps_dissableMessage($GPGSV);
				delayMs(GPS_MSG_INIT_DELAY);
				gps_dissableMessage($GPRMC);
				delayMs(GPS_MSG_INIT_DELAY);
				gps_dissableMessage($GPVTG);
				delayMs(GPS_MSG_INIT_DELAY);
				gps_setRate($GPGGA, 1);

				while(fix[0] == '0' && errorTimer > 0){
					if(gpsDataUpdated){
						errorTimer--;
						gpsDataUpdated = false;
						messageIterator = 0;
						 // Make sure that you are comparing GPGGA message
						 // $PSRF and $GPVTG messages are possible at the startup
						if(strncmp(gpsReceiveString,"$GPGGA", 6) == 0){
							ptr = &gpsReceiveString[7]; //This value could change whether the $ is used or not
							for(; messageIterator < 7; messageIterator ++){
								tmpPtr = ptrToNMEA[messageIterator];
								while(*ptr++ != ','){
									*ptrToNMEA[messageIterator]++ = *(ptr-1);
								}
								ptrToNMEA[messageIterator] = tmpPtr;
							}
						}
		/*				SEND_SERIAL_MSG("Waiting for sats...\r\n");
						SEND_SERIAL_MSG(gpsReceiveString);
						SEND_SERIAL_MSG("\r\n");*/

						blinkRedLed2();
					}
				}
				//if not enough satellites are found, turn off gps
				if(!errorTimer){
					hibernateGps();
					state &= 0xF5;//11110101

					xbeeTransmitString[0] = 'C';
					xbeeTransmitString[1] = ' ';
					xbeeTransmitString[2] = 0x8F;
					xbeeTransmitString[3] = '#';
					xbeeTransmitString[4] = 0x02; //gps not initialized error
					xbeeTransmitString[5] = '\0';
					transmitRequest(SerialAddrHigh, SerialAddrLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

				}
				else{
					//if satellites found -> turn on $GPVTG to monitor velocity
					if(readingVelocity == true){
						delayMs(400);
						gps_setRate($GPVTG,1);
						delayMs(400);
						blinkRedLed2();
						gps_dissableMessage($GPGGA);
					}
					else{
						state |= 0x08;
						state &=~ 0x02;
					}
					SEND_SERIAL_MSG("SATs found...\r\n");
				}
			}
			/*
			 * Initialize SD if chosen
			 */
			if((state&0x04) >> 2){

				errorTimer = 10;
				while(!initializeSD() && errorTimer-- > 1){
					delayMs(300);
					blinkRedLed3();
				}
				if(!errorTimer){
					//If sd card doesnt turn on, dont log anything to it
					state &= 0xFB;

					xbeeTransmitString[0] = 'C';
					xbeeTransmitString[1] = ' ';
					xbeeTransmitString[2] = 0x8F;
					xbeeTransmitString[3] = '#';
					xbeeTransmitString[4] = 0x03; //sd not initialized error
					xbeeTransmitString[5] = '\0';
					transmitRequest(SerialAddrHigh, SerialAddrLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

				}
				else{

					findDetailsOfFAT(sdBuffer,&fatSect,&mstrDir, &fsInfoSector);
					findDetailsOfFile("LOGFILE",sdBuffer,mstrDir,&filesize,&cluster,&sector);
					/*
					 * Program halts during this function
					 */
					findLastClusterOfFile("LOGFILE",sdBuffer, &cluster,fatSect,mstrDir);

					if(filesize < 512)
						filesize = 512;

					appendTextToTheSD("\nNEW LOG", '\n', &sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
				}
			}
			/*
			 * Sync time with a coordinator
			 */
			strcpy(&xbeeTransmitString[0],"C  \0");
			xbeeTransmitString[2] = 0x01;
			transmitRequest(CoordAddrHigh,CoordAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

			/*
			 * Send response to serial node about readiness
			 */
			xbeeTransmitString[0] = 'C';
			xbeeTransmitString[1] = ' ';
			xbeeTransmitString[2] = 0x87;
			xbeeTransmitString[3] = '#';
			xbeeTransmitString[4] = state;
			xbeeTransmitString[5] = '\0';
			transmitRequest(SerialAddrHigh, SerialAddrLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

			if(state == 0x00){
				moduleStatus = 	MODULE_NOT_INITIALIZED;
			}
			else{
				moduleStatus = 	MODULE_IDLE;
			}
			break;

			case MODULE_IDLE:
			//Battery blinky blinky with state
			blinkGreenLeds(state);
			ADC_value = (ADC_GetConversionValue(ADC1));
			ADC_value = (ADC_value * 330) / 128;
			batteryIndicationStartup(ADC_value);
			break;
		}

    	if(xbeeDataUpdated == true){
    		typeOfFrame = xbeeReceiveBuffer[0];
    		switch(typeOfFrame){
				/*
				 * Parse AT command packet
				 */
    			case AT_COMMAND_RESPONSE:
					if(xbeeReceiveBuffer[2] == 'D' && xbeeReceiveBuffer[3] == 'B'){

					}
					if(xbeeReceiveBuffer[2] == 'P' && xbeeReceiveBuffer[3] == 'L'){
						if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_REQUEST){
							if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
								//SEND_SERIAL_MSG("DEBUG#SERIAL POWER LEVEL:");
								//SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA]+ASCII_DIGIT_OFFSET);
								//SEND_SERIAL_MSG("\r\n");
								strcpy(&xbeeTransmitString[0],"C  \0");
								xbeeTransmitString[0] = 'C';
								xbeeTransmitString[1] = ' ';
								xbeeTransmitString[2] = 0x8B;
								xbeeTransmitString[3] = ' ';
								xbeeTransmitString[4] = xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA];
								xbeeTransmitString[5] = ' ';
								xbeeTransmitString[6] = '\0';
								transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);


							}else{
								//SEND_SERIAL_MSG("PL_AT_COMMAND_REQUEST_ERROR\r\n");
							}
						}else if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_APPLY){
							if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
								//SEND_SERIAL_MSG("DEBUG#PL_AT_COMMAND_APPLIED\r\n");
								askXbeeParam("PL",AT_FRAME_ID_REQUEST);

							}else{
								//SEND_SERIAL_MSG("DEBUG#PL_AT_COMMAND_APPLY_ERROR\r\n");
							}
						}
					}
					/*
					 * Other possible AT data
					 */
					break;
			/*
			 * Parse RECEIVE command packet
			 */
			case RECIEVE_PACKET:
				if(xbeeReceiveBuffer[XBEE_DATA_MODE_OFFSET] == 'C'){

					receivedAddHigh = 0;
					receivedAddLow = 0;

					receivedAddHigh |= xbeeReceiveBuffer[1] << 24;
					receivedAddHigh |= xbeeReceiveBuffer[2] << 16;
					receivedAddHigh |= xbeeReceiveBuffer[3] << 8;
					receivedAddHigh |= xbeeReceiveBuffer[4];

					receivedAddLow |= xbeeReceiveBuffer[5] << 24;
					receivedAddLow |= xbeeReceiveBuffer[6] << 16;
					receivedAddLow |= xbeeReceiveBuffer[7] << 8;
					receivedAddLow |= xbeeReceiveBuffer[8];

					switch(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET]){
					case (0x80):
						timerWindow = xbeeReceiveBuffer[16] - 2;
						itoa(timerWindow,timerString,10);
						Usart1_SendString(timerString);
						Usart1_SendString("\r\n");
						nodesInNetwork = xbeeReceiveBuffer[18] - 1;
						itoa(nodesInNetwork,timerString,10);
						Usart1_SendString(timerString);
						Usart1_SendString("\r\n");
						TIM_SetCounter(TIM2,atoi(&xbeeReceiveBuffer[20]) + TIMER_SYNC_DELAY);
						itoa(TIM_GetCounter(TIM2),timerString,10);
						Usart1_SendString(timerString);
						Usart1_SendString("\r\n");

						/*
						 * Send positive response to Gateway
						 */
						break;
					case (0x0F):
						/*
						 * Init. request
						 */
						if(moduleStatus == MODULE_NOT_INITIALIZED){
							moduleStatus = MODULE_INITIALIZING;

							state = atoi(&xbeeReceiveBuffer[16]);
							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x10):
						if(moduleStatus == MODULE_IDLE){
							moduleStatus = MODULE_EXPERIMENT_MODE;

							/*
							 * Start new timer interrupt based on window calculation
							 */
							itoa(TIM_GetCounter(TIM2),timerString,10);
							Usart1_SendString(timerString);
							Usart1_SendString("\r\n");

							transmitTimerValue = TIM_GetCounter(TIM2)%800 - (800/nodesInNetwork)*timerWindow;
							itoa(transmitTimerValue,timerString,10);
							Usart1_SendString(timerString);
							Usart1_SendString("\r\n");

							if(transmitTimerValue < 0)
								transmitTimerValue = 800 + transmitTimerValue;
							itoa(transmitTimerValue,timerString,10);
							Usart1_SendString(timerString);
							Usart1_SendString("\r\n");
							TIM_SetCounter(TIM15,transmitTimerValue);
							TIM_ClearITPendingBit(TIM15, TIM_IT_Update);
							TIM_ITConfig(TIM15, TIM_IT_Update, ENABLE);

							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x11):
						if(moduleStatus == MODULE_EXPERIMENT_MODE){
							moduleStatus = MODULE_IDLE;

							TIM_ClearITPendingBit(TIM15, TIM_IT_Update);
							TIM_ITConfig(TIM15, TIM_IT_Update, DISABLE);
							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x12):
						if(moduleStatus == MODULE_IDLE){
						moduleStatus = MODULE_NOT_INITIALIZED;
						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x15):
						/*
						 * GET_BATTERY
						 */
						ADC_value = (ADC_GetConversionValue(ADC1));
						ADC_value = (ADC_value * 330) / 128;
						itoa(ADC_value,itoaConversionString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x86;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],itoaConversionString);
						transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						break;
					case (0x16):
						/*
						 * GET_GPSCOORD_NODE
						 */
						readingVelocity = false;
						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						break;
					case (0x17):
						//GET POWER
						askXbeeParam("PL",AT_FRAME_ID_REQUEST);
						/*
						 * 0x8B used for response
						 */

						break;
					case (0x18):
						//SET POWER
						xbeeApplyParamter("PL",xbeeReceiveBuffer[16] - ASCII_DIGIT_OFFSET,AT_FRAME_ID_APPLY);
						/*
						 * 0x8B used for response
						 */
						break;

					case (0X1E):

						CoordAddrHigh = receivedAddHigh;
						CoordAddrLow = receivedAddLow;

						SerialAddrLow = 0;

						SerialAddrLow |= xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] << 24;
						SerialAddrLow |= xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+3] << 16;
						SerialAddrLow |= xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+4] << 8;
						SerialAddrLow |= xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+5];

						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(SerialAddrHigh,SerialAddrLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

						break;
					case (0x1F):
						/*
						 * PRINT_TIME_NODE
						 */
						globalCounter = TIM_GetCounter(TIM2);
						itoa(globalCounter,timerString,10);
						Usart1_SendString(timerString);
						Usart1_SendString("\r\n");

						break;
					default:
						break;
					}
				}
				break;
				case MODEM_STATUS:
				SEND_SERIAL_MSG("MODEM STATUS...\r\n");
				if(xbeeReceiveBuffer[1] == 0x00){
					SEND_SERIAL_MSG("HARDWARE RESET...\r\n");
				}
				break;
				case TRANSMIT_STATUS:
				SEND_SERIAL_MSG("TRANSMIT STATUS - ");
				if(xbeeReceiveBuffer[5] == 0x00){
					SEND_SERIAL_MSG("SUCCESS...\r\n");
				}
				else{
					SEND_SERIAL_MSG("FAIL - ");
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[5]);
					SEND_SERIAL_MSG(" \r\n");
				}
				break;
				/*
				 * Parse Any other XBEE packet
				 */
				default:
				SEND_SERIAL_MSG("UNKNOWN PACKET...\r\n");
				break;
    		}
    		xbeeDataUpdated = false;
    	}
    	/*
    	 * Check whether there aren't any unread XBEE data
    	 */
    	else if(xbeeReading == false  && !GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4) && xbeeDataUpdated == false){

    		errorTimer = 0;
    		cheksum = 0;
    		xbeeReading = true;
			XBEE_CS_LOW();

			while(SPI1_TransRecieve(0x00) != 0x7E){	//Wait for start delimiter
				errorTimer++;
				if(errorTimer >ERROR_TIMER_COUNT)			//Exit loop if there is no start delimiter
					break;
			}
			if(errorTimer < ERROR_TIMER_COUNT){
				SPI1_TransRecieve(0x00);
				length = SPI1_TransRecieve(0x00);
				uint8_t i = 0;
				for(; i < length; i ++ ){				//Read data based on packet length
					xbeeReceiveBuffer[i] = SPI1_TransRecieve(0x00);
					cheksum += xbeeReceiveBuffer[i];
				}
				cheksum += SPI1_TransRecieve(0x00);
				if(cheksum == 0xFF){
					xbeeDataUpdated = true;
				}
				//Data is updated if checksum is true
				xbeeReading = false;
				XBEE_CS_HIGH();
			}
			else{
				xbeeReading = false;
			}
    	}
    }
}

/*
 * INTERRUPT ROUTINES
 */

void USART2_IRQHandler(void){
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
		if((gpsReceiveString[gpsReadIterator++] = USART_ReceiveData(USART2)) == '\n'){
			gpsDataUpdated = true;
			gpsReadIterator = 0;
			if(strncmp(gpsReceiveString,"$GPVTG" , 6) == 0){
				gps_parseGPVTG(gpsReceiveString,velocityString);
			}
			else if(strncmp(gpsReceiveString,"$GPGGA" , 6) == 0){
				gps_parseGPGGA(gpsReceiveString,ts,lat,lon,fix,sats);
			}
		}
	}
}

void EXTI4_15_IRQHandler(void)					//External interrupt handlers
{
	if(EXTI_GetITStatus(EXTI_Line4) == SET){	//Handler for Radio ATTn pin interrupt (data ready indicator)

		if(xbeeReading == false && SPI1_Busy == false){
			errorTimer, cheksum = 0;
			xbeeReading = true;
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);
			while(SPI1_TransRecieve(0x00) != 0x7E){	//Wait for start delimiter
				errorTimer++;
				if(errorTimer > ERROR_TIMER_COUNT)			//Exit loop if there is no start delimiter
					break;
			}
			if(errorTimer < ERROR_TIMER_COUNT){
				SPI1_TransRecieve(0x00);
				length = SPI1_TransRecieve(0x00);
				uint8_t i = 0;
				for(; i < length; i ++ ){				//Read data based on packet length
					xbeeReceiveBuffer[i] = SPI1_TransRecieve(0x00);
					cheksum += xbeeReceiveBuffer[i];
				}
				xbeeReceiveBuffer[i] = '\0';
				cheksum += SPI1_TransRecieve(0x00);
				if(cheksum == 0xFF)
					xbeeDataUpdated = true;					//Data is updated if checksum is true
				xbeeReading = false;
				GPIO_SetBits(GPIOA,GPIO_Pin_4);
			}
			else{
				xbeeReading = false;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}


void TIM14_IRQHandler(){

	if(TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);

		accBuff[accBuffValue++] = returnX_axis();
		if(accBuffValue > 4)
			accBuffValue = 0;
	}
}

void TIM15_IRQHandler(){

	if(TIM_GetITStatus(TIM15, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM15, TIM_IT_Update);
		/*
		 * Transmit shit here
		 */
		xbeeTransmitString[0] = 'M';
		xbeeTransmitString[1] = ' ';
		xbeeTransmitString[2] = state;
		xbeeTransmitString[3] = '\0';
		if(state&0x01){
			itoa((accBuff[0] + accBuff[1] + accBuff[2] + accBuff[3] + accBuff[4])/5, accString, 10);
			strcat(xbeeTransmitString,"#");
			strcat(xbeeTransmitString,&accString[0]);
			xorGreenLed(0);
		}
		if((state&0x02) >> 1){
			strcat(xbeeTransmitString,"#");
			strcat(xbeeTransmitString,velocityString);
			xorGreenLed(1);
		}
		if((state&0x08) >> 1){
			strcat(xbeeTransmitString,"#");
			strcat(xbeeTransmitString,lat);
			strcat(xbeeTransmitString,"#");
			strcat(xbeeTransmitString,lon);
			xorGreenLed(2);
		}
		transmitRequest(CoordAddrHigh, CoordAddrLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
	}
}

