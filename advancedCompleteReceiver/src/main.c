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

#define NUMBER_OF_NODES 4
#define ACC_BUFFER_SIZE 50
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
struct node receiverNode;
bool SPI1_Busy = false;
uint32_t globalCounter = 0;
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
/*
 * Struct for node measurements, id's...
 */
struct node{
	uint32_t adressHigh;
	uint32_t adressLow;
	int16_t measurment[2];	//0 -> ACC 1 -> RSSI 2 -> GPS
	uint8_t avrRSSI;
	uint32_t packetTime;
	uint8_t avarageRSSIcount;
	float velocity;
	/*
	 * if there is an error, bit is set to 1
	 * alarm will be called if there are more then 1 bit set (sum is
	 * 0b00000|gps|rssi|acc|
	 */
	uint8_t errorByte;
	/*
	 * state - 8 bit Config register
	 * |SD|COM|NETWORK|---|---|GPS|RSSI|ACC|
	 * default - 0x00
	 */
	uint8_t state;
};

int main(void){
	/*
	 * Initialize addresses of nodes
	 */
	receiverNode.adressHigh = 0x0013A200;
	receiverNode.adressLow = 0x40E3E13A;
	receiverNode.state = 0;		//0 -> ACC 1 -> RSSI 2 -> GPS
	receiverNode.velocity = 0.1;
	//Nodes
	struct node node[NUMBER_OF_NODES];
	node[0].adressHigh = 0x0013A200;
	node[0].adressLow = 0x409783D9;
	node[0].state = 0;
	node[0].errorByte = 0;
	node[0].avrRSSI = 0;
	node[0].avarageRSSIcount = 0;
	node[0].packetTime = 0;

	node[1].adressHigh = 0x0013A200;
	node[1].adressLow = 0x409783DA;
	node[1].state = 0;
	node[1].errorByte = 0;
	node[1].avrRSSI = 0;
	node[1].avarageRSSIcount = 0;
	node[1].packetTime = 0;

	node[2].adressHigh = 0x0013A200;
	node[2].adressLow = 0x40E3E13D;
	node[2].state = 0;
	node[2].errorByte = 0;
	node[2].avrRSSI = 0;
	node[2].avarageRSSIcount = 0;
	node[2].packetTime = 0;

	//Serial node
	node[3].adressHigh = 0x0013A200;
	node[3].adressLow = 0x40E32A94;
	node[3].state = 0;
	node[3].errorByte = 0;
	node[3].avrRSSI = 0;
	node[3].avarageRSSIcount = 0;
	node[3].packetTime = 0;
	/*
	 * Local variables for XBEE
	 */
	uint8_t typeOfFrame;
	uint8_t commandStatus;
	uint8_t AT_data[4];
	uint8_t frameID;
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
    char velocityString[6] = " ";
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
	int n = 0;//Node finding iterator
	uint8_t tmpNode;//
	char timerString[10];
	char stringOfMessurement[32] = "";
	char stringOfRelative[6] = "";
	uint16_t thresholdACC = 120;
	uint8_t thresholdGPS = 5;
	uint8_t thresholdRSSI = 10;
	uint16_t thresholdTIM = 20; //10*100ms=1s
	bool transferNode[3] = {false,false,false};
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
	Timer_interrupt_enable();
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
					SEND_SERIAL_MSG("GPS timeout...\r\n");
					hibernateGps();
					state &= 0xFD;
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
			transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

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

				//When buffer is full, send data to gateway
				if(accBufferFull == true){
					accBufferFull = false;
					//Send buffer to gateway
					xbeeTransmitString[0] = 'C';
					xbeeTransmitString[1] = ' ';
					xbeeTransmitString[2] = 0x8D;
					xbeeTransmitString[3] = ' ';
					//xbeeTransmitString[4] = (accBuff[0] >> 8);
					//xbeeTransmitString[5] = accBuff[0];
					memcpy(&xbeeTransmitString[4],accBuff,2);
					transmitRequestBytes(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,&xbeeTransmitString[0],6);
				}
				break;
		}
    	if(xbeeDataUpdated == true){
    		typeOfFrame = xbeeReceiveBuffer[0];
    		switch(typeOfFrame){
				/*
				 * Parse AT command packet
				 */
    			case AT_COMMAND_RESPONSE:

				frameID = xbeeReceiveBuffer[1];
    			/*
    			 * Parse RSSI packet
    			 */

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
							transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);


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

							transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

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
					if(node[frameID-1].avarageRSSIcount == 6){
						node[frameID-1].measurment[RSSI_MEASUREMENT] = xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX];
							/*
							 * If threshold was exceeded - raise an alarm bit
							 */
							rssiDiff = abs(node[frameID-1].avrRSSI - node[frameID-1].measurment[RSSI_MEASUREMENT]);

							if(rssiDiff > thresholdRSSI){
								node[frameID-1].errorByte |= 0x02;
							}
							else{
								/*
								 * If threshold was not exceeded - clear an alarm bit
								 */
								node[frameID-1].errorByte &= 0x05;
							}
					}
					/*
					 * From first couple of packets - calculate avarage rssi
					 */
					else if(node[frameID-1].avarageRSSIcount++ < 6){
						if(node[frameID-1].avarageRSSIcount==1){
							node[frameID-1].avrRSSI =  xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX];
						}else{
							node[frameID-1].avrRSSI =  (xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX] + node[frameID-1].avrRSSI) / 2 ;
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

					if((node[frameID-1].errorByte & 0x01)+((node[frameID-1].errorByte >> 1)&0x01)+(node[frameID-1].errorByte >> 2) > 1){

						GPIOB->ODR |= (1 << (5+tmpNode));
					}
					else{
						//indicate that one of the five nodes received packet was good
						GPIOB->ODR ^= (1 << (5+tmpNode));
					}

					/*
					 * Sending relative/absolute values
					 */
					if(transferNode[tmpNode] == true){
						//Re-send data to serial node
						if(node[tmpNode].state == ACC_STATE_CASE){
							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x88;
							xbeeTransmitString[3] = ' ';
							itoa(accDiff,stringOfRelative,10);
							xbeeTransmitString[4] = tmpNode + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[5] = '#';
							xbeeTransmitString[6] = (xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX] / 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[7] = (xbeeReceiveBuffer[AT_COMMAND_DATA_INDEX] % 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[8] = '#';
							xbeeTransmitString[9] = node[frameID-1].errorByte + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[10] = '#';
							strcpy(&xbeeTransmitString[11],stringOfRelative);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,stringOfMessurement);
						}
						else if(node[tmpNode].state == GPS_STATE_CASE){
							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x89;
							xbeeTransmitString[3] = ' ';
							ftoa(velocityDiff,stringOfRelative,1);
							xbeeTransmitString[4] = tmpNode + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[5] = '#';
							xbeeTransmitString[6] = (rssiDiff / 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[7] = (rssiDiff % 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[8] = '#';
							xbeeTransmitString[9] = node[frameID-1].errorByte + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[10] = '#';
							strcpy(&xbeeTransmitString[11],stringOfRelative);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,stringOfMessurement);
						}
						else if(node[tmpNode].state == GPS_COORD_CASE){

							xbeeTransmitString[0] = 'C';
							xbeeTransmitString[1] = ' ';
							xbeeTransmitString[2] = 0x8A;
							xbeeTransmitString[3] = ' ';
							xbeeTransmitString[4] = tmpNode + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[5] = '#';
							xbeeTransmitString[6] = (node[frameID-1].measurment[RSSI_MEASUREMENT] / 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[7] = (node[frameID-1].measurment[RSSI_MEASUREMENT] % 10) + ASCII_DIGIT_OFFSET;
							xbeeTransmitString[8] = '#';
							strcpy(&xbeeTransmitString[9],receivedLat);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,receivedLon);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,lat);
							strcat(xbeeTransmitString,"#");
							strcat(xbeeTransmitString,lon);
						}
						transmitRequest(node[3].adressHigh, node[3].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
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
				uint32_t receivedAddressHigh = 0;
				uint32_t receivedAddressLow = 0;

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
				 * so compare just that
				 */
				for(n = 0; n < NUMBER_OF_NODES; n++){	//Find the matching node for the received address
					if(receivedAddressLow == node[n].adressLow )
						tmpNode = n;
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
					 * Example
					 * M 0 45
					 * M 1 10.2
					 */
					i = i + 2;
					node[tmpNode].state = xbeeReceiveBuffer[i++] - 0x30;	//Calculate the integer from ASCII by subtracting 0x30

					n = 0;
					while (xbeeReceiveBuffer[i++] != '\0'){
						stringOfMessurement[n++] = xbeeReceiveBuffer[i];
					}
					stringOfMessurement[n] = xbeeReceiveBuffer[i-1];

					//Measure the time when packet was received
					node[tmpNode].packetTime = globalCounter;

/*					SEND_SERIAL_BYTE(tmpNode + 0x30);
					SEND_SERIAL_MSG(stringOfMessurement);
					SEND_SERIAL_MSG(":Received\r\n");*/

					switch(node[tmpNode].state){
						case ACC_STATE_CASE:
							node[tmpNode].measurment[ACC_MEASUREMENT] = atoi(stringOfMessurement);
							receiverNode.measurment[ACC_MEASUREMENT] = (accBuff[0] + accBuff[1] + accBuff[2] + accBuff[3] + accBuff[4])/5;

							accDiff = abs(receiverNode.measurment[ACC_MEASUREMENT] - node[tmpNode].measurment[ACC_MEASUREMENT]);

							if(accDiff > thresholdACC){
								node[tmpNode].errorByte |= 0x01;
								//add sd card accelerometer error code
								if(((state&0x04) >> 2)){
									SPI1_Busy = true;
									appendTextToTheSD(stringOfMessurement, ' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
									SPI1_Busy = false;
								}
							}
							else {
								node[tmpNode].errorByte &= 0x06;
								//if sd card is in use, log to it
								if(((state&0x04) >> 2)){
									SPI1_Busy = true;
									appendTextToTheSD(stringOfMessurement, ' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
									SPI1_Busy = false;
								}
							}
							//after receiving acc measurement, ask module for last packets RSSI
							askXbeeParam("DB",tmpNode+1);
							break;

						case GPS_STATE_CASE:
							node[tmpNode].velocity = stof(stringOfMessurement);
							if(gpsDataUpdated == true){
								gpsDataUpdated = false;
								gps_parseGPVTG(gpsReceiveString,velocityString);
								receiverNode.velocity = stof(velocityString);
/*								SEND_SERIAL_MSG(velocityString);
								SEND_SERIAL_MSG(":MyVel\r\n");*/
							}

							velocityDiff = receiverNode.velocity - node[tmpNode].velocity;

							if(velocityDiff < 0)
								velocityDiff *= -1.0;

							if(velocityDiff > thresholdGPS){
								node[tmpNode].errorByte |= 0x04;
								//SEND_SERIAL_MSG("GPS_DANGER\r\n");
								//Error was measured - log it to sd card
								if(((state&0x04) >> 2)){
									SPI1_Busy = true;
									appendTextToTheSD(stringOfMessurement, ' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
									SPI1_Busy = false;
								}
							}
							else{
								node[tmpNode].errorByte &= 0x03;
								//If sd card is in use, log data to it
								if(((state&0x04) >> 2)){
										SPI1_Busy = true;
										appendTextToTheSD(stringOfMessurement, ' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
										SPI1_Busy = false;
									}
							}
							//after receiving gps measurement, ask module for last packets RSSI

							askXbeeParam("DB",tmpNode+1);
							break;

						case GPS_COORD_CASE:

							str_splitter(stringOfMessurement,receivedLat,receivedLon,"#");
							if(gpsDataUpdated == true){
								gps_parseGPGGA(gpsReceiveString,ts,lat,lon,fix,sats);
								gpsDataUpdated = false;
							}
							askXbeeParam("DB",tmpNode+1);
							break;
						}
				}
				else if(xbeeReceiveBuffer[i] == 'C'){
					/*
					 * Example
					 * C 0 -> Request for timestamp
					 * C 1
					 */
					i++;

					switch(xbeeReceiveBuffer[++i]){

					case (0x01):
						//Timer sync. request
						GPIOB->ODR ^= (1 << (5+tmpNode));
						itoa(globalCounter,timerString,10);
		    		    xbeeTransmitString[0] = 'C';
		    		    xbeeTransmitString[1] = ' ';
		    		    xbeeTransmitString[2] = 0x80;
		    		    xbeeTransmitString[3] = ' ';
		    		    strcpy(&xbeeTransmitString[4],&timerString[0]);
						SPI1_Busy = true;
						transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
						SPI1_Busy = false;
						break;

					case (0x02):
						/*
						 *SET_EVENT_COORD
						 */
						if(((state&0x04) >> 2)){

							itoa(globalCounter, timerString,10);
							SPI1_Busy = true;
							appendTextToTheSD("EVENT",' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
							appendTextToTheSD(timerString,' ',&sdBufferCurrentSymbol, sdBuffer, "LOGFILE", &filesize, mstrDir, fatSect, &cluster, &sector);
							SPI1_Busy = false;
							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
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
						transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);


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
						transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

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
						transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

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
						transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

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
						transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

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
						transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

						break;
					case (0x09):
						/*
						 *GET_ABSREL_NODE
						 */
						if(xbeeReceiveBuffer[16] == '7'){
							transferNode[0] = true;
							transferNode[1] = true;
							transferNode[2] = true;
							transferNode[3] = true;
						}
						else{
							transferNode[atoi(&xbeeReceiveBuffer[16])] = true;
						}
						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

						break;
					case (0x0A):
						/*
						 *STOP_ABSREL_NODE
						 */
						if(xbeeReceiveBuffer[16] == '7'){
							transferNode[0] = false;
							transferNode[1] = false;
							transferNode[2] = false;
							transferNode[3] = false;
						}
						else{
							transferNode[atoi(&xbeeReceiveBuffer[16])] = false;
						}
						/*
						 * Positive response
						 */
						strcpy(&xbeeTransmitString[0],"C  \0");
						xbeeTransmitString[2] = 0x81;
						transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

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
							transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
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
							transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
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
							transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
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
						transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
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
						transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);

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
						transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
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
						transmitRequest(node[tmpNode].adressHigh, node[tmpNode].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
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
						transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);

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

						break;
					case (0x1B):
						//START ACC EXPERIMENT
						if(moduleStatus == MODULE_IDLE){
							moduleStatus = MODULE_EXPERIMENT_MODE;
							accBuffValue = 0;
							/*
							 * Positive response
							 */
							strcpy(&xbeeTransmitString[0],"C  \0");
							xbeeTransmitString[2] = 0x81;
							transmitRequest(node[3].adressHigh,node[3].adressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
						}
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

    	if(timerUpdated == true){

    		/*
    		 * We are not interested in checkin serial nodes timeout
    		 */
    		for(i = 0; i < NUMBER_OF_NODES-1; i++){
    			if(node[i].packetTime == 0){
    				continue;
    			}
    			timDiff = globalCounter - node[i].packetTime;
    			if(timDiff > thresholdTIM){
    				/*
    				 * TIM_DNG
    				 */
    				node[i].packetTime = 0;
					strcpy(&xbeeTransmitString[0],"C V#");
					itoa(timDiff,stringOfMessurement,10);
					xbeeTransmitString[4] = tmpNode + ASCII_DIGIT_OFFSET;
					xbeeTransmitString[5] = '#';
					strcpy(&xbeeTransmitString[6],stringOfMessurement);
					transmitRequest(node[3].adressHigh, node[3].adressLow, TRANSOPT_DISACK, 0x00, xbeeTransmitString);
    			}
    		}
    		timerUpdated = false;
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

void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		globalCounter++;
		timerUpdated = true;
	}
}

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

