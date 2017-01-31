/*
 * STM32 and C libraries
 */
#include <stm32f0xx.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/*
 * DEW| peripheral libraries
 */
#include "SPI1.h"
#include "SPI2.h"
#include "SysTickDelay.h"
#include "USART1.h"
#include "USART2.h"
#include "Timer.h"
#include "ADC.h"

#include "XBee.h"
#include "Button.h"
#include "IndicationGPIOs.h"
#include "ADXL362.h"
#include "A2035H.h"
#include "sdCard.h"
#include "accTimer.h"
/*
 * DEW| software libraries
 */
#include "MyStringFunctions.h"
#include "dynamicNode.h"
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
#define ACC_STATE_CASE 0x01
#define GPS_STATE_CASE 0x02
#define ACCGPS_STATE_CASE 0x03
#define GPS_COORD_CASE 0x04
#define XBEE_DATA_MODE_OFFSET 12
#define XBEE_DATA_TYPE_OFFSET 14

#define NUMBER_OF_NODES 4
#define ACC_BUFFER_SIZE 20

#define SERIAL_ADDR_HIGH 0x0013A200
#define SERIAL_ADDR_LOW 0x40E32A94
/*
 * XBEE globals
 */
char xbeeReceiveBuffer[255];
volatile bool xbeeDataUpdated = false;
volatile uint8_t length,errorTimer,cheksum;
bool xbeeReading = false;
/*
 * Module globals
 */
uint8_t turnOffTimer = 0;
bool SPI1_Busy = false;
//uint32_t globalCounter = 0;
bool timerUpdated = false;
int16_t accBuff[ACC_BUFFER_SIZE];
uint8_t accBuffValue = 0;
bool accBufferFull = false;
/*
 * GPS globals
 */
char gpsReceiveString[96];
uint8_t gpsReadIterator;
volatile bool gpsDataUpdated = false;
char velocityString[6] = " ";

int main(void){
	/*
	 * Initialize addresses of nodes
	 */
	uint32_t receivedAddressHigh = 0;
	uint32_t receivedAddressLow = 0;

	struct node* CoordinatorNode = list_createRoot();
	list_setAddress(CoordinatorNode,0x0013A200,0x40E3E13A);
	CoordinatorNode->addressLow = 0x40E3E13A;
	CoordinatorNode->state = 0;
	CoordinatorNode->velocity = 0.1;

	struct node* gatewayNode  = list_createRoot();
	gatewayNode->addressLow = SERIAL_ADDR_LOW;
	gatewayNode->addressHigh = SERIAL_ADDR_HIGH;
	struct node* lastNode = CoordinatorNode;
	struct node* currNode = lastNode;

	/*
	 * Local variables for XBEE
	 */
	uint8_t typeOfFrame;
	uint8_t commandStatus;
	uint8_t AT_data[4];
	char xbeeTransmitString[128];
	uint16_t rssiDiff = 0;
	uint32_t channelMask = 0x00;
	/*
	 *--- Accelerometer does not
	 *--- have any variables
	 *--- for coordinator
	 */
	uint16_t accDiff;
	/*
	 * Local variables for GPS
	 */
	char* ptr;
	char* tmpPtr;
    char ts[11] = " ";
    char lat[11] = " ";
    char receivedLat[11] = " ";
    char latd[2]= " ";
    char lon[11]= " ";
    char receivedLon[11] = " ";
    char lond[2]= " ";
    char fix[2]= "0";
    char sats[3]= " ";

    char *ptrToNMEA[] = {ts, lat, latd, lon, lond, fix, sats};
	uint8_t messageIterator;
	float velocityDiff;
	bool readingVelocity = true;
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
	uint8_t state = 0;
	uint8_t moduleStatus = MODULE_NOT_INITIALIZED;
	uint16_t ADC_value;
	uint8_t errorTimer = 40;
	int i = 0;//Packet reading iterator
	char timerString[10];
	char stringOfMessurement[32] = "";
	char stringOfACC[16] = "";
	char stringOfGPS[16] = "";
	char stringOfRelative[6] = "";
	uint16_t thresholdACC = 120;
	uint8_t thresholdGPS = 5;
	uint8_t thresholdRSSI = 10;
	uint16_t thresholdTIM = 20; //10*100ms=1s
	char thesholdString[8];
	uint16_t timDiff = 0;
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
	Usart2_Init(BAUD_4800);
	Usart1_Init(BAUD_9600);
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
	Initialize_timer();
	//Timer_interrupt_enable();
	Initialize_accTimer();
	accTimer_interrupt_enable();

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
					transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
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
					state &= 0xFD;

					xbeeTransmitString[0] = 'C';
					xbeeTransmitString[1] = ' ';
					xbeeTransmitString[2] = 0x8F;
					xbeeTransmitString[3] = '#';
					xbeeTransmitString[4] = 0x02; //gps not initialized error
					xbeeTransmitString[5] = '\0';
					transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
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
					transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
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
			 * Send response to serial node about readiness
			 */
			xbeeTransmitString[0] = 'C';
			xbeeTransmitString[1] = ' ';
			xbeeTransmitString[2] = 0x87;
			xbeeTransmitString[3] = '#';
			xbeeTransmitString[4] = state;
			xbeeTransmitString[5] = '\0';
			transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

			moduleStatus = 	MODULE_IDLE;
			break;

			case MODULE_IDLE:
				//Battery blinky blinky with state
				blinkGreenLeds(state);
				ADC_value = (ADC_GetConversionValue(ADC1));
				ADC_value = (ADC_value * 330) / 128;
				batteryIndicationStartup(ADC_value);

				break;

			case MODULE_EXPERIMENT_MODE:

				break;
		}
    	if(xbeeDataUpdated == true){
    		typeOfFrame = xbeeReceiveBuffer[0];
    		switch(typeOfFrame){
				/*
				 * Parse AT command packet
				 */
    			case AT_COMMAND_RESPONSE:

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
							transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);


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
				else if(xbeeReceiveBuffer[2] == 'N' && xbeeReceiveBuffer[3] == 'O'){
					if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_REQUEST){
						if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
							if(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA] != 4){
								xbeeTransmitString[0] = 'C';
								xbeeTransmitString[1] = ' ';
								xbeeTransmitString[2] = 0x8F;
								xbeeTransmitString[3] = '#';
								xbeeTransmitString[4] = 0x04; //Node discovery error
								xbeeTransmitString[5] = '\0';
								transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
							}
							else{
								askXbeeParam("FN",AT_FRAME_ID_REQUEST);
							}
						}
					}else if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_APPLY){
						if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
							//DEBUG#NO_AT_COMMAND_APPLIED
							askXbeeParam("NO",AT_FRAME_ID_REQUEST);
						}
					}
				}
				else if(strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "FN", 2) == 0) {

					if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){

						receivedAddressHigh = (xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX] << 24)
								+(xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX+1] << 16)
								+(xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX+2] << 8)
								+(xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX+3]);

						receivedAddressLow = (xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX] << 24)
								+(xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX+1] << 16)
								+(xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX+2] << 8)
								+(xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX+3]);

						if(receivedAddressLow != SERIAL_ADDR_LOW){

							list_addNode(&lastNode,receivedAddressLow);
							lastNode->rssiMeasurment = xbeeReceiveBuffer[length-1];

							if(lastNode == NULL){
								xbeeTransmitString[0] = 'C';
								xbeeTransmitString[1] = ' ';
								xbeeTransmitString[2] = 0x8F;
								xbeeTransmitString[3] = '#';
								xbeeTransmitString[4] = 0x05; //Node discovery error
								xbeeTransmitString[5] = '\0';
								transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
							}
							else{
								xbeeTransmitString[0] = 'C';
								xbeeTransmitString[1] = ' ';
								xbeeTransmitString[2] = 0x90;
								xbeeTransmitString[3] = '#';
								xbeeTransmitString[4] = lastNode->id; //Inform gateway about found nodes id
								xbeeTransmitString[5] = '\0';
								transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
							}
						}
					}
				}
				else if(xbeeReceiveBuffer[2] == 'C' && xbeeReceiveBuffer[3] == 'M'){
					if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_REQUEST){
						if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){

							//channelMask
							if(length == 9){
								channelMask = (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA] << 24)
										+ (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+1] << 16)
										+ (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+2] << 8)
										+ (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+3]);
							}
							else{
								channelMask = (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA] << 8)
										+ (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+1]);
							}

							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x8C;
							xbeeTransmitString[3] = ' ';
							itoa(channelMask,&xbeeTransmitString[4],10);

							transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

						}else{
							//SEND_SERIAL_MSG("PL_AT_COMMAND_REQUEST_ERROR\r\n");
						}
					}else if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_APPLY){
						if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){

							askXbeeParam("CM",AT_FRAME_ID_REQUEST);
						}else{

						}
					}
				}
				else if(xbeeReceiveBuffer[2] == 'D' && xbeeReceiveBuffer[3] == 'B'){
					if(currNode->avarageRSSIcount == 6){
						currNode->rssiMeasurment = xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX];
						/*
						 * If threshold was exceeded - raise an alarm bit
						 */
						rssiDiff = abs(currNode->avrRSSI - currNode->rssiMeasurment);

						if(rssiDiff > thresholdRSSI){
							currNode->errorByte |= 0x02;
						}
						else{
							/*
							 * If threshold was not exceeded - clear an alarm bit
							 */
							currNode->errorByte &= 0x05;
						}
					}
					/*
					 * From first couple of packets - calculate avarage rssi
					 */
					else if(currNode->avarageRSSIcount++ < 6){
						if(currNode->avarageRSSIcount==1){
							currNode->avrRSSI =  xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX];
						}else{
							currNode->avrRSSI =  (xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX] + currNode->avrRSSI) / 2 ;
						}
					}
					/*
					 * *** ERROR CHECKING ROUTINE ***
					 *
					 * Error is checked after reading the RSSI value
					 * Node's id is stored in AT packets frame_id value
					 *
					 * Last 3 bits of errorByte represents gps,rssi and acc error
					 *
					 * If more then one sensor reads danger value, the alarm routine is called
					 *
					 * Blinking means that received measurement was good
					 * Solid light means error
					 */

					if((currNode->errorByte & 0x01)+((currNode->errorByte >> 1)&0x01)+(currNode->errorByte >> 2) > 1){

						GPIOB->ODR |= (1 << (5+currNode->id));
					}
					else{
						//indicate that one of the five nodes received packet was good
						GPIOB->ODR ^= (1 << (5+currNode->id));
					}

					/*
					 * Sending relative/absolute values
					 */
					if(currNode->transferToGateway == true){
						//Re-send data to serial node
						if(currNode->state == ACC_STATE_CASE){
							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x88;
							xbeeTransmitString[3] = ' ';
							itoa(accDiff,stringOfRelative,10);
							xbeeTransmitString[4] = currNode->id + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[5] = '#';
							xbeeTransmitString[6] = (xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX] / 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[7] = (xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX] % 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[8] = '#';
							xbeeTransmitString[9] = currNode->errorByte + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[10] = '#';
							strcpy(&xbeeTransmitString[11],stringOfRelative);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,stringOfACC);
						}
						else if(currNode->state == GPS_STATE_CASE){
							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x89;
							xbeeTransmitString[3] = ' ';
							ftoa(velocityDiff,stringOfRelative,1);
							xbeeTransmitString[4] = currNode->id + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[5] = '#';
							xbeeTransmitString[6] = (rssiDiff / 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[7] = (rssiDiff % 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[8] = '#';
							xbeeTransmitString[9] = currNode->errorByte + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[10] = '#';
							strcpy(&xbeeTransmitString[11],stringOfRelative);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,stringOfGPS);
						}
						else if(currNode->state == GPS_COORD_CASE){

							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x8A;
							xbeeTransmitString[3] = ' ';
							xbeeTransmitString[4] = currNode->id + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[5] = '#';
							xbeeTransmitString[6] = (currNode->rssiMeasurment / 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[7] = (currNode->rssiMeasurment % 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[8] = '#';
							strcpy(&xbeeTransmitString[9],receivedLat);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,receivedLon);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,lat);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,lon);
						}
						transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
						if(((state&0x04) >> 2)){
							SPI1_Busy = true;
							appendTextToTheSD(xbeeTransmitString, '\n',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
							SPI1_Busy = false;
						}
					}
				}
				/*
				 * Other possible AT data
				 */
				else{
					i = 5;
					for(; i < length; i++){		//Read AT data
						AT_data[i - 5] = xbeeReceiveBuffer[i];
					}
				}
				break;
			/*
			 * Parse RECEIVE command packet
			 */
			case RECIEVE_PACKET:

				i = 1;	//Remember that "Frame type" byte was the first one

				for(; i < 9; i++){	//Read address from received packet

					if(i<5){
						receivedAddressHigh |= xbeeReceiveBuffer[i] << 8*(4-i);
					}
					else{
						receivedAddressLow |= xbeeReceiveBuffer[i] << 8*(8-i);
					}
				}
				/*
				 * Only the lowest part of the address differs
				 */
				itoa(receivedAddressLow,stringOfMessurement,16);
				Usart1_SendString("Received address:");
				Usart1_SendString(stringOfMessurement);
				Usart1_SendString("\r\n");
				itoa(SERIAL_ADDR_LOW,stringOfMessurement,16);
				Usart1_SendString("Serial address:");
				Usart1_SendString(stringOfMessurement);
				Usart1_SendString("\r\n");

				if(receivedAddressLow == SERIAL_ADDR_LOW){
					//Address of a node is saved in nextNode pointer
					//To not mess up list_findNodeByAdd() function
					//Since gatewayNode is not part of the list
					gatewayNode->nextNode = currNode;
					currNode = gatewayNode;
				}
				else{
					//If gatewayNode was used last time, jump back into a list
					if(currNode->addressLow == SERIAL_ADDR_LOW)
						currNode = currNode->nextNode;
					currNode = list_findNodeByAdd(receivedAddressLow,currNode,CoordinatorNode);
					if(currNode == NULL){
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x8F;
						xbeeTransmitString[3] = '#';
						xbeeTransmitString[4] = 0x05; //Node discovery error
						xbeeTransmitString[5] = '\0';
						transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

						/*
						 * This is kinda risky
						 */
						currNode = gatewayNode;
					}
				}
				//After reading source address, comes 2 reserved bytes
				i = i + 2;
				//In this case comandStatus actually is receive options
				commandStatus = xbeeReceiveBuffer[i++];
				/*
				 * Whether the command(C) or measurement(M) was received
				 */
				if(xbeeReceiveBuffer[i] == 'M'){
					/*
					 * "M <state>#<acc>#<gps>"
					 */
					i = i + 2;
					currNode->state = xbeeReceiveBuffer[i++];

/*				n = 0;
				while (xbeeReceiveBuffer[i++] != '\0'){
						stringOfMessurement[n++] = xbeeReceiveBuffer[i];
					}
					stringOfMessurement[n] = xbeeReceiveBuffer[i-1];*/

					if((currNode->state&0x03) == ACCGPS_STATE_CASE){
						//load into stringOfACC
						//load into stringOfGPS
						str_splitter(&xbeeReceiveBuffer[i+1],stringOfACC,stringOfGPS,"#");
					}
					else if((currNode->state&0x01) == ACC_STATE_CASE){
						//load only stringOfACC
						//str_splitter(&xbeeReceiveBuffer[i+1],stringOfGPS,stringOfACC,"#");
						strcpy(stringOfACC,&xbeeReceiveBuffer[i+1]);
					}
					else if((currNode->state&0x02) == GPS_STATE_CASE){
						//load only stringOfGPS
						//str_splitter(&xbeeReceiveBuffer[i],stringOfACC,stringOfGPS,"#");
						strcpy(stringOfGPS,&xbeeReceiveBuffer[i+1]);
					}
					//Measure the time when packet was received
					currNode->packetsTime = TIM_GetCounter(TIM2);
					/*
					 * THIS IS WHERE THE MEASUREMENT COMPAREMENT HAPPENS
					 */
					if((currNode->state&0x01) == ACC_STATE_CASE){

						currNode->accMeasurment = atoi(stringOfACC);
						CoordinatorNode->accMeasurment = (accBuff[0] + accBuff[1] + accBuff[2] + accBuff[3] + accBuff[4])/5;

						accDiff = abs(CoordinatorNode->accMeasurment - currNode->accMeasurment);

						if(accDiff > thresholdACC){
							currNode->errorByte |= 0x01;
							//add sd card accelerometer error code
						}
						else{
							currNode->errorByte &= 0x06;
						}
					}
					if((currNode->state&0x02) == GPS_STATE_CASE){

						currNode->velocity = stof(stringOfGPS);
						if(gpsDataUpdated == true){
							gpsDataUpdated = false;
							CoordinatorNode->velocity = stof(velocityString);
						}
						velocityDiff = CoordinatorNode->velocity - currNode->velocity;

						if(velocityDiff < 0)
							velocityDiff *= -1.0;

						if(velocityDiff > thresholdGPS){
							currNode->errorByte |= 0x04;
							//SEND_SERIAL_MSG("GPS_DANGER\r\n");
							//Error was measured - log it to sd card
						}
						else{
							currNode->errorByte &= 0x03;
						}
					}
					if((currNode->state&0x04) == GPS_COORD_CASE){
						/*
						 * GPS_COORD case
						 */
						str_splitter(stringOfMessurement,receivedLat,receivedLon,"#");
						if(gpsDataUpdated == true){
							gps_parseGPGGA(gpsReceiveString,ts,lat,lon,fix,sats);
							gpsDataUpdated = false;
						}
					}
					/*
					 * When comparision is done, ask for RSSI to complete error, alarm check
					 */
					askXbeeParam("DB",currNode->id+1);

				}
				else if(xbeeReceiveBuffer[i] == 'C'){

					i++;
					switch(xbeeReceiveBuffer[++i]){

					case (0x01):
						//Timer sync. request
						GPIOB->ODR ^= (1 << (5+currNode->id));
						itoa(TIM_GetCounter(TIM2),timerString,10);
		    		    xbeeTransmitString[0] = 'C';
		    		    xbeeTransmitString[1] = ' ';
		    		    xbeeTransmitString[2] = 0x80;
		    		    xbeeTransmitString[3] = ' ';
		    		    xbeeTransmitString[4] = currNode->id; //dummy id
		    		    xbeeTransmitString[5] = ' ';
		    		    strcpy(&xbeeTransmitString[6],&timerString[0]);
		    		    /*
		    		     * Need to add time window
		    		     */
						SPI1_Busy = true;
						transmitRequest(currNode->addressHigh, currNode->addressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
						SPI1_Busy = false;
						break;

					case (0x02):
						/*
						 *SET_EVENT_COORD
						 */
						if(((state&0x04) >> 2)){

							itoa(TIM_GetCounter(TIM2), timerString,10);
							SPI1_Busy = true;
							appendTextToTheSD("EVENT",' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
							appendTextToTheSD(timerString,' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
							SPI1_Busy = false;
							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x03):
						/*
						 *SET_THRACC
						 */
						thresholdACC = atoi(&xbeeReceiveBuffer[16]);
						itoa(thresholdACC,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x83;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);


						break;
					case (0x04):
						/*
						 *SET_THRGPS
						 */
						thresholdGPS = atoi(&xbeeReceiveBuffer[16]);
						itoa(thresholdGPS,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x84;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

						break;
					case (0x05):
						/*
						 *SET_THRRSSI
						 */
						thresholdRSSI = atoi(&xbeeReceiveBuffer[16]);
						itoa(thresholdRSSI,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x82;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

						break;
					case (0x06):
						/*
						 *GET_THRRSSI
						 */
						itoa(thresholdRSSI,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x82;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

						break;
					case (0x07):
						/*
						 *GET_THRACC
						 */
						itoa(thresholdACC,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x83;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

						break;
					case (0x08):
						/*
						 *GET_THRGPS
						 */
						itoa(thresholdGPS,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x84;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

						break;
					case (0x09):
						/*
						 *GET_ABSREL_NODE / GET_ABSREL_ALL
						 */
						if(xbeeReceiveBuffer[16] == '7'){
							list_setTransferForAll(CoordinatorNode,true);
						}
						else{
							currNode = list_findNodeById(atoi(&xbeeReceiveBuffer[16]), currNode, CoordinatorNode);
							if(currNode == NULL){
								xbeeTransmitString[0] = 'C';
								xbeeTransmitString[1] = ' ';
								xbeeTransmitString[2] = 0x8F;
								xbeeTransmitString[3] = '#';
								xbeeTransmitString[4] = 0x04; //Node discovery error
								xbeeTransmitString[5] = '\0';
								transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
							}
							else{
								currNode->transferToGateway = true;
							}
						}
						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

						break;
					case (0x0A):
						/*
						 *STOP_ABSREL_NODE
						 */
						if(xbeeReceiveBuffer[16] == '7'){
							list_setTransferForAll(CoordinatorNode,false);
						}
						else{
							currNode = list_findNodeById(atoi(&xbeeReceiveBuffer[16]), currNode, CoordinatorNode);
							if(currNode == NULL){
								xbeeTransmitString[0] = 'C';
								xbeeTransmitString[1] = ' ';
								xbeeTransmitString[2] = 0x8F;
								xbeeTransmitString[3] = '#';
								xbeeTransmitString[4] = 0x04; //Node discovery error
								xbeeTransmitString[5] = '\0';
								transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
							}
							else{
								currNode->transferToGateway = false;
							}
						}
						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

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
							transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x10):
						if(moduleStatus == MODULE_IDLE){
							moduleStatus = MODULE_EXPERIMENT_MODE;
							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x11):
						if(moduleStatus == MODULE_EXPERIMENT_MODE){
							moduleStatus = MODULE_IDLE;
							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x12):
						if(moduleStatus == MODULE_IDLE){
						moduleStatus = MODULE_SAFE_TURNOFF;
						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
						break;
					case (0x13):
						/*
						 * SET_THRRTIM
						 */
						thresholdTIM = atoi(&xbeeReceiveBuffer[16]);
						itoa(thresholdTIM,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x85;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(currNode->addressHigh, currNode->addressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

						break;
					case (0x14):
						/*
						 *GET_THRTIM
						 */
						itoa(thresholdTIM,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x85;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(currNode->addressHigh, currNode->addressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
						break;
					case (0x15):
						/*
						 * GET_BATTERY
						 */
						ADC_value = (ADC_GetConversionValue(ADC1));
						ADC_value = (ADC_value * 330) / 128;
						itoa(ADC_value,thesholdString,10);
						xbeeTransmitString[0] = 'C';
						xbeeTransmitString[1] = ' ';
						xbeeTransmitString[2] = 0x86;
						xbeeTransmitString[3] = ' ';
						strcpy(&xbeeTransmitString[4],thesholdString);
						transmitRequest(currNode->addressHigh, currNode->addressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
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
						transmitRequest(SERIAL_ADDR_HIGH,SERIAL_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

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
					case (0x19):
						//GET CHANNELS
						askXbeeParam("CM",AT_FRAME_ID_REQUEST);
						/*
						 * 0x8C used for response
						 */
						break;
					case (0x1A):
						//SET CHANNELS
						Usart1_SendString("Received channel info:");
						Usart1_SendString(&xbeeReceiveBuffer[16]);
						channelMask = atoi(&xbeeReceiveBuffer[16]);
						xbeeApplyDwordParamter("CM",channelMask,AT_FRAME_ID_APPLY);

					case (0x1B):
						//DISCOVER_NETWORK
						list_clearNodes(CoordinatorNode);
						lastNode = CoordinatorNode;
						currNode = CoordinatorNode;

						xbeeApplyParamter("NO",0x04,AT_FRAME_ID_APPLY);
						break;

					case (0x1C):
						//GET_NODE_LIST - RESPOND with 0x91
						currNode = CoordinatorNode->nextNode;

						do{
							//Send every nodes address to gateway
							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x91;
							xbeeTransmitString[3] = '#';
							xbeeTransmitString[4] = currNode->id;
							xbeeTransmitString[5] = '#';
							xbeeTransmitString[6] = currNode->addressLow >> 24; //Inform gateway about found nodes id
							xbeeTransmitString[7] = currNode->addressLow >> 16;
							xbeeTransmitString[8] = currNode->addressLow >> 8;
							xbeeTransmitString[9] = currNode->addressLow;
							xbeeTransmitString[10] = '\0';
							transmitRequest(SERIAL_ADDR_HIGH, SERIAL_ADDR_LOW, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

							currNode = currNode->nextNode;
							/*
							 * Fucking delay might be needed here
							 */
							delayMs(100);
						}
						while(currNode != NULL);
						break;
					default:
						break;
					}
				}
				else{
					/*
					 * If other data then commands or measurements were received
					 */
					SEND_SERIAL_MSG("Received data: ");
					SEND_SERIAL_MSG(&xbeeReceiveBuffer[i]);
					SEND_SERIAL_MSG("\r\n");
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

    	/*
    	 * Respond for timer error in 0x8E message
    	 */

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
			if(strncmp(gpsReceiveString,"$GPVTG" , 6) == 0)
				gps_parseGPVTG(gpsReceiveString,velocityString);
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

/*void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		globalCounter++;
		timerUpdated = true;
	}
}*/

void TIM14_IRQHandler()
{
	if(TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);

		accBuff[accBuffValue++] = returnX_axis();
		if(accBuffValue > ACC_BUFFER_SIZE-1){
			accBufferFull = true;
			accBuffValue = 0;
		}

	}
}

