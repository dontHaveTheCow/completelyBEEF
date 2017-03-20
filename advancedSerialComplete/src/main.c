/*
 * STM32 and C libraries
 */
#include <stm32f0xx.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/*
 * DEW| hardware libraries
 */
#include "SPI1.h"
#include "SysTickDelay.h"
#include "USART1.h"

#include "Timer.h"
#include "XBee.h"
#include "IndicationGPIOs.h"
#include "SPI2.h"
#include "ADXL362.h"
#include "accTimer.h"
#include "transmitTimer.h"
/*
 * DEW| software libraries
 */
#include "MyStringFunctions.h"
#include "dynamicNode.h"

/*
 * XBEE defines
 */
#define ERROR_TIMER_COUNT 30
#define XBEE_DATA_MODE_OFFSET 12
#define XBEE_DATA_TYPE_OFFSET 14
#define TIMER_SYNC_DELAY 92 //ms

#define COORDINATOR_ADDR_HIGH 0x0013A200
#define COORDINATOR_ADDR_LOW 0x40E3E13A

#define SERIAL_ADDR_HIGH 0x0013A200
#define SERIAL_ADDR_LOW 0x40E3E13C
/*
 * SERIAL defines
 */
#define ASCII_DIGIT_OFFSET 0x30
/*
 * Any other defines
 */
#define TOGGLE_REDLED_SERIAL() GPIOB->ODR ^= GPIO_Pin_5
#define TOGGLE_REDLED_XBEE() GPIOB->ODR ^= GPIO_Pin_6
#define SET_REDLED_SERIAL() GPIOB->ODR |= GPIO_Pin_5

#define ACC_BUFFER_SIZE 20
/*
 * Serial globals
 */
char serialBuffer[256];
uint8_t packetLenght = 0;
bool serialUpdated = false;
/*
 * XBEE globals
 */
char xbeeReceiveBuffer[255] = " ";
volatile bool xbeeDataUpdated = false;
volatile uint8_t length,errorTimer,cheksum;
bool SPI1_Busy = false;
bool xbeeReading = false;
/*
 * Module globals
 */
//uint32_t globalCounter = 0;
int16_t accBuff[ACC_BUFFER_SIZE];
uint8_t accBuffValue = 0;

int main(void)
{
	//Nodes
	struct node* CoordinatorNode = list_createRoot();
	list_setAddress(CoordinatorNode,COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW);

	struct node* lastNode = CoordinatorNode;
	struct node* currNode = lastNode;

	//list_addNode(&lastNode,SERIAL_ADDR_LOW);
/*	list_addNode(&lastNode,0x409783D9);
	list_addNode(&lastNode,0x409783DA);
	list_addNode(&lastNode,0x40E3E13D);*/

	//Local variables - serial
	char s_delimiter[2] = "#";
	char key[32] = "";
	char value[64];
	char str_helper[32] = "";
	char timerString[10];
	//Local variables - XBEE
	uint32_t xbeeAddressHigh;
	uint32_t xbeeAddressLow;
	char xbeeAddressHighString[12];
	char xbeeAddressLowString[12];
	char xbeeTransmitString[64];
	char xbeeReceivedRelative[32];
	char xbeeReceivedAbsolute[32];
	uint8_t nodesFound = 0;
	uint32_t channelMask = 0x3FFFFFFF; //for all 30 channels enabled
	uint32_t tmpmask = 0x00;
	uint8_t tmpcounter = 0;
	uint8_t rErrorByte = 0x00;

	uint32_t receivedAddressHigh = 0;
	uint32_t receivedAddressLow = 0;
	uint8_t iterator = 1;
	//Initialize peripherals
	initializeEveryRedLed();
	initializeEveryGreenLed();
	initializeXbeeATTnPin();

	SET_REDLED_SERIAL();

	Usart1_Init(BAUD_9600);
	ConfigureUsart1Interrupt();
	initialiseSysTick();
	InitialiseSPI1_GPIO();
	InitialiseSPI1();
	Configure_SPI1_interrupt();
	Initialize_timer();
	//Timer_interrupt_enable();
	Initialize_transmitTimer();
	transmitTimer_interrupt_enable();

	//acelerometer stuff
	InitialiseSPI2_GPIO();
	InitialiseSPI2();
	Initialize_accTimer();
	accTimer_interrupt_enable();

	//Wait 1 second for XBEE to start UpP...
	delayMs(1000);
	xbeeDataUpdated = xbeeStartupParamRead(ERROR_TIMER_COUNT,(uint8_t*)xbeeReceiveBuffer);
	/*
	 * Module ready
	 */
	SEND_SERIAL_MSG("MSG#SERIAL_MODULE_READY\r\n");
	TOGGLE_REDLED_SERIAL();
	TOGGLE_REDLED_XBEE();

	 while(1){
    	if(serialUpdated){
    		/*
    		 * Split received serial message
    		 */
    		str_splitter(serialBuffer,key,value,s_delimiter);
    		/*
    		 * Process serial
    		 */
    		if(strcmp(key,"SEND_MESSAGE") == 0){
				str_splitter(value,key,str_helper,s_delimiter);
				currNode = list_findNodeById(atoi(key),currNode,CoordinatorNode);
				if(currNode == NULL)
					SEND_SERIAL_MSG("list_findNodeById error \r\n");

    			if(currNode->id < lastNode->id)
    				transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,str_helper);
				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");

    		}
    		else if(strcmp(key,"START_EXPERIMENT") == 0){

    			str_splitter(value,key,str_helper,s_delimiter);
    			uint8_t experimentPackets = atoi(str_helper);
    			uint8_t target = atoi(key);

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x1D;

    			if(target < lastNode->id){
        			for(; experimentPackets > 0; experimentPackets--){
        				delayMs(100);

        				transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
        				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");
        			}
    			}
    		}
    		else if(strcmp(key,"GET_TIME") == 0){

				SEND_SERIAL_MSG("MSG#CURRENT_TIME#");
				itoa(TIM_GetCounter(TIM2),timerString,10);
				SEND_SERIAL_MSG(timerString);
				SEND_SERIAL_MSG("\r\n");
			}
			else if(strcmp(key,"GET_ADDRESS_HIGH") == 0){
				askXbeeParam("SH",AT_FRAME_ID_REQUEST);
			}
			else if(strcmp(key,"GET_ADDRESS_LOW") == 0){
				askXbeeParam("SL",AT_FRAME_ID_REQUEST);
			}
			else if(strcmp(key,"GET_POWER_SERIAL") == 0){
				askXbeeParam("PL",AT_FRAME_ID_REQUEST);
			}
			else if(strcmp(key,"SET_POWER_SERIAL") == 0){
				xbeeApplyParamter("PL",atoi(value),AT_FRAME_ID_APPLY);
			}
			else if(strcmp(key,"GET_CHANNELS_SERIAL") == 0){
				askXbeeParam("CM",AT_FRAME_ID_REQUEST);
			}			else if(strcmp(key,"SET_POWER_SERIAL") == 0){
				xbeeApplyParamter("PL",atoi(value),AT_FRAME_ID_APPLY);
			}
			else if(strcmp(key,"GET_CHANNELS_SERIAL") == 0){
				askXbeeParam("CM",AT_FRAME_ID_REQUEST);
			}
			else if(strcmp(key,"SET_CHANNELS_SERIAL") == 0){

				char* ptr_helper = value;
				uint8_t digitCounter;
				tmpmask = 0x00;

				while(*ptr_helper != '\0'){
					//Comma separates passed channel id
					digitCounter = 0;
					while(*ptr_helper != ','){
						if(*ptr_helper == '\0')
							break;
						digitCounter++;ptr_helper++;
					}
					//jump away from comma
					if(digitCounter == 1){
						tmpmask |= 1 << (*(ptr_helper-1) - ASCII_DIGIT_OFFSET);
					}else{
						tmpmask |= 1 << (((*(ptr_helper-2) - ASCII_DIGIT_OFFSET)*10)
									+ (*(ptr_helper-1) - ASCII_DIGIT_OFFSET));
					}
					if(*ptr_helper++ == '\0')
						break;
				}
				xbeeApplyDwordParamter("CM",tmpmask,AT_FRAME_ID_APPLY);

			}
			else if(strcmp(key,"DISCOVER_NET") == 0){
				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x1B;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_NODE_LIST") == 0){

				list_clearNodes(CoordinatorNode);
				currNode = CoordinatorNode;
				lastNode = CoordinatorNode;

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x1C;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"COORD_NODE_SYNC") == 0){
				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x1D;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"SYNC_TIME") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x01;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"SET_EVENT") == 0){

				SEND_SERIAL_MSG("EVENTS#");
				SEND_SERIAL_MSG(value);
				SEND_SERIAL_BYTE('#');
				itoa(TIM_GetCounter(TIM2),timerString,10);
				SEND_SERIAL_MSG(timerString);
				SEND_SERIAL_MSG("\r\n");
			}
			else if(strcmp(key,"SET_EVENT_COORD") == 0){

				/*
				 * If this CMD doesnt seem to work, this might be because 0x00 equals to null
				 */
				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x02;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"SET_THRACC") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x03;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"SET_THRGPS") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x04;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"SET_THRRSSI") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x05;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_THRRSSI") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x06;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_THRACC") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x07;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_THRGPS") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x08;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_ABSREL") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x09;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_ABSREL_ALL") == 0){

				//Parameter "7" tells to get data from every node
				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x09;
				strcpy(&xbeeTransmitString[4],"7");
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"STOP_ABSREL_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x0A;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"STOP_ABSREL_ALL") == 0){

				//Parameter "7" tells to get data from every node
				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x0A;
				strcpy(&xbeeTransmitString[4],"7");
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_REL_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x0B;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_REL_ALL") == 0){

				//Parameter "7" tells to get data from every node
				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x0B;
				strcpy(&xbeeTransmitString[4],"7");
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_ABS_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x0C;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"GET_ABS_ALL") == 0){

				//Parameter "7" tells to get data from every node
				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x0C;
				strcpy(&xbeeTransmitString[4],"7");

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					if(currNode == CoordinatorNode->nextNode){
						currNode = currNode->nextNode;
					}
					delayMs(2000);
				}
			}
			else if(strcmp(key,"STOP_ABS_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x0D;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"STOP_ABS_ALL") == 0){

				//Parameter "7" tells to get data from every node
				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x0D;

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					if(currNode == CoordinatorNode->nextNode){
						currNode = currNode->nextNode;
					}
					delayMs(2000);
				}
				SEND_SERIAL_MSG("DEBUG#PACKETS#SENT...\r\n");
			}
			else if(strcmp(key,"STOP_REL_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x0E;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"STOP_REL_ALL") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x0E;
				strcpy(&xbeeTransmitString[4],"7");
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"INIT_NODE") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '7';
				xbeeTransmitString[5] = '\0';
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"INIT_ALL") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '7';
				xbeeTransmitString[5] = '\0';

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}

			}
			else if(strcmp(key,"INIT_ACC_NODE") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '1';
				xbeeTransmitString[5] = '\0';
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"INIT_ACC_ALL") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '1';
				xbeeTransmitString[5] = '\0';

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(1000);
				}
				currNode = CoordinatorNode;

			}
			else if(strcmp(key,"INIT_GPS_NODE") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '2';
				xbeeTransmitString[5] = '\0';
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"INIT_GPS_ALL") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '2';
				xbeeTransmitString[5] = '\0';

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"INIT_SD_NODE") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '4';
				xbeeTransmitString[5] = '\0';
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}

			}
			else if(strcmp(key,"INIT_SD_ALL") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '4';
				xbeeTransmitString[5] = '\0';

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"INIT_ACCSD_NODE") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '5';
				xbeeTransmitString[5] = '\0';
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}

			}
			else if(strcmp(key,"INIT_ACCSD_ALL") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '5';
				xbeeTransmitString[5] = '\0';

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"INIT_GPSSD_NODE") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '6';
				xbeeTransmitString[5] = '\0';
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}

			}
			else if(strcmp(key,"INIT_GPSSD_ALL") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x0F;
				xbeeTransmitString[3] = ' ';
				xbeeTransmitString[4] = '6';
				xbeeTransmitString[5] = '\0';

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"START_ALL") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x10;

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"START_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x10;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"IDLE_ALL") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x11;

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"IDLE_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x11;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"STOP_ALL") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x12;

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					//delayMs(2000);
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"STOP_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x12;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"SET_THRTIM") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x13;
				strcpy(&xbeeTransmitString[4],value);
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_THRTIM") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x14;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"GET_BATTERY") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x15;
				transmitRequest(COORDINATOR_ADDR_HIGH,COORDINATOR_ADDR_LOW,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
			}
			else if(strcmp(key,"SET_GPSCOORD_ALL") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x16;

				currNode = CoordinatorNode;
				while(currNode != NULL){
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
					currNode = currNode->nextNode;
					delayMs(2000);
				}
				currNode = CoordinatorNode;
				SEND_SERIAL_MSG("DEBUG#PACKETS#SENT...\r\n");
			}
			else if(strcmp(key,"SET_GPSCOORD_NODE") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x16;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else if(strcmp(key,"GET_POWER") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x17;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");
			}
			else if(strcmp(key,"SET_POWER") == 0){

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x18;
				str_splitter(value,key,str_helper,s_delimiter);
				strcpy(&xbeeTransmitString[4],str_helper);
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");
			}
			else if(strcmp(key,"GET_CHANNELS") == 0){

				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x19;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");
			}
			else if(strcmp(key,"SET_CHANNELS") == 0){

				str_splitter(value,key,str_helper,s_delimiter);
				//Parse str_helper to output dword
				//str_helper -> 1,3,4,5,7
				char* ptr_helper = str_helper;
				uint8_t digitCounter;
				tmpmask = 0x00;

				while(*ptr_helper != '\0'){
					//Comma separates passed channel id
					digitCounter = 0;
					while(*ptr_helper != ','){
						if(*ptr_helper == '\0')
							break;
						digitCounter++;ptr_helper++;
					}
					//jump away from comma
					if(digitCounter == 1){
						tmpmask |= 1 << (*(ptr_helper-1) - ASCII_DIGIT_OFFSET);
					}else{
						tmpmask |= 1 << (((*(ptr_helper-2) - ASCII_DIGIT_OFFSET)*10)
									+ (*(ptr_helper-1) - ASCII_DIGIT_OFFSET));
					}
					if(*ptr_helper++ == '\0')
						break;
				}

				itoa(tmpmask,str_helper,10);

				strcpy(&xbeeTransmitString[0],"C   ");
				xbeeTransmitString[2] = 0x1A;

				strcpy(&xbeeTransmitString[4],str_helper);
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");
			}
			else if(strcmp(key,"PRINT_CMD") == 0){

				SEND_SERIAL_MSG("\nMSG#SEND_MESSAGE#<id>#<message>\r\n");
				SEND_SERIAL_MSG("MSG#START_EXPERIMENT#<packet_count>\r\n");
				SEND_SERIAL_MSG("MSG#GET_TIME\r\n");
				SEND_SERIAL_MSG("MSG#DISCOVER_NET\r\n");
				SEND_SERIAL_MSG("MSG#GET_NODE_LIST\r\n");
				SEND_SERIAL_MSG("MSG#SYNC_TIME\r\n");
				SEND_SERIAL_MSG("MSG#GET_ADDRESS_HIGH\r\n");
				SEND_SERIAL_MSG("MSG#GET_ADDRESS_LOW\r\n");
				SEND_SERIAL_MSG("MSG#GET_POWER_SERIAL\r\n");
				SEND_SERIAL_MSG("MSG#SET_POWER_SERIAL\r\n");
				SEND_SERIAL_MSG("MSG#GET_CHANNELS_SERIAL\r\n");
				SEND_SERIAL_MSG("MSG#SET_CHANNELS_SERIAL\r\n");
				SEND_SERIAL_MSG("MSG#SET_EVENT#\r\n");
				SEND_SERIAL_MSG("MSG#SET_EVENT_COORD\r\n");
				SEND_SERIAL_MSG("MSG#SET_THRACC#\r\n");
				SEND_SERIAL_MSG("MSG#SET_THRGPS#\r\n");
				SEND_SERIAL_MSG("MSG#SET_THRRSSI#\r\n");
				SEND_SERIAL_MSG("MSG#SET_THRTIM#\r\n");
				SEND_SERIAL_MSG("MSG#GET_THRACC#\r\n");
				SEND_SERIAL_MSG("MSG#GET_THRGPS#\r\n");
				SEND_SERIAL_MSG("MSG#GET_THRRSSI#\r\n");
				SEND_SERIAL_MSG("MSG#GET_THRTIM#\r\n");
				SEND_SERIAL_MSG("MSG#GET_BATTERY#\r\n");
				SEND_SERIAL_MSG("MSG#GET_ABSREL#\r\n");
				SEND_SERIAL_MSG("MSG#GET_ABSREL_ALL#\r\n");
				SEND_SERIAL_MSG("MSG#STOP_ABSREL_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#GET_REL_ALL#\r\n");
				SEND_SERIAL_MSG("MSG#GET_ABS_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#GET_ABS_ALL#\r\n");
				SEND_SERIAL_MSG("MSG#STOP_REL_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#STOP_REL_ALL#\r\n");
				SEND_SERIAL_MSG("MSG#STOP_ABS_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#STOP_ABS_ALL#\r\n");
				SEND_SERIAL_MSG("MSG#<--INIT_COMMANDS-->");
				SEND_SERIAL_MSG("MSG#INIT_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#INIT_ALL\r\n");
				SEND_SERIAL_MSG("MSG#INIT_ACC_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#INIT_ACC_ALL\r\n");
				SEND_SERIAL_MSG("MSG#INIT_GPS_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#INIT_GPS_ALL\r\n");
				SEND_SERIAL_MSG("MSG#INIT_SD_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#INIT_GPS_ALL\r\n");
				SEND_SERIAL_MSG("MSG#INIT_ACCSD_NODE\r\n");
				SEND_SERIAL_MSG("MSG#INIT_ACCSD_NODE\r\n");
				SEND_SERIAL_MSG("MSG#INIT_GPSSD_NODE\r\n");
				SEND_SERIAL_MSG("MSG#INIT_GPSSD_ALL\r\n");
				SEND_SERIAL_MSG("MSG#START_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#START_ALL\r\n");
				SEND_SERIAL_MSG("MSG#IDLE_ALL\r\n");
				SEND_SERIAL_MSG("MSG#INIT_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#STOP_ALL\r\n");
				SEND_SERIAL_MSG("MSG#STOP_NODE\r\n");
				SEND_SERIAL_MSG("MSG#SET_GPSCOORD_ALL\r\n");
				SEND_SERIAL_MSG("MSG#SET_GPSCOORD_NODE#\r\n");
				SEND_SERIAL_MSG("MSG#GET_POWER#\r\n");
				SEND_SERIAL_MSG("MSG#SET_POWER#<id>#<>\r\n");
				SEND_SERIAL_MSG("MSG#GET_CHANNELS\r\n");
				SEND_SERIAL_MSG("MSG#SET_CHANNELS#<id>#<ch>\r\n");
			}
			else if(strcmp(key,"PRINT_ACC_CMD") == 0){
				SEND_SERIAL_MSG("\nMSG#<ACC_COMMANDS>\r\n");
				SEND_SERIAL_MSG("MSG#LOCAL_INIT_ACC\r\n");
				SEND_SERIAL_MSG("MSG#LOCAL_STOP_ACC\r\n");
				SEND_SERIAL_MSG("MSG#LOCAL_GET_ACC\r\n");
				SEND_SERIAL_MSG("MSG#START_ACC_EXPERIMENT\r\n");
				SEND_SERIAL_MSG("MSG#SYNC_START_ACC\r\n");
			}
			else if(strcmp(key,"LOCAL_INIT_ACC") == 0){

				errorTimer = 10;
				initializeADXL362();
				while(!return_ADXL_ready() && --errorTimer > 0){
					//wait time for caps to discharge
					delayMs(500);
					initializeADXL362();
					delayMs(500);
					SEND_SERIAL_MSG("DEBUG#TRYING_TO_INIT_ACC...\r\n");
				}

				if(!errorTimer){
					SEND_SERIAL_MSG("DEBUG#ACC_INIT_FAILED!!!\r\n");
				}
				else{
					SEND_SERIAL_MSG("DEBUG#ACC_READY!!!\r\n");

				TIM_Cmd(TIM14,ENABLE);
				SEND_SERIAL_MSG("DEBUG#STARTING_ACC_TIMER!!!\r\n");
				}
			}
			else if(strcmp(key,"LOCAL_STOP_ACC") == 0){

				TIM_Cmd(TIM14,DISABLE);

			}
			else if(strcmp(key,"LOCAL_GET_ACC") == 0){
				TIM_Cmd(TIM14,ENABLE);
			}
			else if(strcmp(key,"START_ACC_EXPERIMENT") == 0){
				strcpy(&xbeeTransmitString[0],"C  \0");
				xbeeTransmitString[2] = 0x1B;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");
			}
			else if(strcmp(key,"SYNC_START_ACC") == 0){

				strcpy(&xbeeTransmitString[0],"C         \0");
				xbeeTransmitString[2] = 0x1C;
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
				SEND_SERIAL_MSG("DEBUG#PACKET#SENT...\r\n");
				delayMs(93);
				TIM_Cmd(TIM14,ENABLE);
			}
			else if(strcmp(key,"PRINT_DEBUG_CMD") == 0){
				SEND_SERIAL_MSG("\nMSG#<DEBUG_COMMANDS>\r\n");
				SEND_SERIAL_MSG("MSG#START_TEST_TIM2\r\n");
				SEND_SERIAL_MSG("MSG#STOP_TEST_TIM2\r\n");
				SEND_SERIAL_MSG("MSG#PRINT_TIM2\r\n");
				SEND_SERIAL_MSG("MSG#PRINT_TIM14\r\n");
				SEND_SERIAL_MSG("MSG#PRINT_TIM15\r\n");
				SEND_SERIAL_MSG("MSG#START_TIM15\r\n");
				SEND_SERIAL_MSG("MSG#STOP_TIM15\r\n");
				SEND_SERIAL_MSG("MSG#FIND_NEIGHBORS\r\n");
				SEND_SERIAL_MSG("MSG#NETWORK_DISCOVER\r\n");
				SEND_SERIAL_MSG("MSG#PUSH_LIST\r\n");
				SEND_SERIAL_MSG("MSG#SET_DISC_OPT#<4 for rssi>\r\n");
				SEND_SERIAL_MSG("MSG#GET_DISC_OPT#\r\n");
				SEND_SERIAL_MSG("MSG#PRINT_NODE_LIST#\r\n");
				SEND_SERIAL_MSG("MSG#PRINT_TIME_NODE#\r\n");
			}
			else if(strcmp(key,"START_TEST_TIM2") == 0){
				TIM_Cmd(TIM2,ENABLE);

			}
			else if(strcmp(key,"STOP_TEST_TIM2") == 0){
				TIM_Cmd(TIM2,DISABLE);
			}
			else if(strcmp(key,"PRINT_TIM2") == 0){
				itoa(TIM_GetCounter(TIM2),timerString,10);
				SEND_SERIAL_MSG("Timer2 value ");
				SEND_SERIAL_MSG(timerString);
				SEND_SERIAL_MSG("\r\n");
			}
			else if(strcmp(key,"PRINT_TIM14") == 0){
				itoa(TIM_GetCounter(TIM14),timerString,10);
				SEND_SERIAL_MSG("Timer14 value ");
				SEND_SERIAL_MSG(timerString);
				SEND_SERIAL_MSG("\r\n");
			}
			else if(strcmp(key,"PRINT_TIM15") == 0){
				itoa(TIM_GetCounter(TIM15),timerString,10);
				SEND_SERIAL_MSG("Timer15 value ");
				SEND_SERIAL_MSG(timerString);
				SEND_SERIAL_MSG("\r\n");
			}
			else if(strcmp(key,"START_TIM15") == 0){
				TIM_SetCounter(TIM15,0);
				TIM_ClearITPendingBit(TIM15, TIM_IT_Update);
				TIM_ITConfig(TIM15, TIM_IT_Update, ENABLE);
				SEND_SERIAL_MSG("Timer15 enabled\r\n");
			}
			else if(strcmp(key,"STOP_TIM15") == 0){
				TIM_ITConfig(TIM15, TIM_IT_Update, DISABLE);
				SEND_SERIAL_MSG("Timer15 value ");
				SEND_SERIAL_MSG("Timer15 disabled\r\n");
			}
			else if(strcmp(key,"FIND_NEIGHBORS") == 0){
				askXbeeParam("FN",AT_FRAME_ID_REQUEST);

				SEND_SERIAL_MSG("MSG#Discovering network...\r\n");
				SEND_SERIAL_MSG("MSG#Id\tAddress\r\n");
			}
			else if(strcmp(key,"NETWORK_DISCOVER") == 0){
				askXbeeParam("ND",AT_FRAME_ID_REQUEST);
			}
			else if(strcmp(key,"PUSH_LIST") == 0){

				list_clearNodes(CoordinatorNode);
				currNode = CoordinatorNode;
				lastNode = CoordinatorNode;

				xbeeApplyParamter("NO",0x04,AT_FRAME_ID_APPLY);
				SEND_SERIAL_MSG("MSG#Discovering network...\r\n");
				askXbeeParam("FN",AT_FRAME_ID_REQUEST);
			}
			else if(strcmp(key,"PRINT_LIST") == 0){

				currNode = CoordinatorNode;

				while(currNode != NULL){

					SEND_SERIAL_MSG("MSG#");
					SEND_SERIAL_BYTE(currNode->id / 10 + 0x30);
					SEND_SERIAL_BYTE(currNode->id % 10 + 0x30);
					SEND_SERIAL_BYTE('\t');
					itoa(currNode->addressLow,xbeeAddressLowString, 16);
					SEND_SERIAL_MSG(xbeeAddressLowString);
					SEND_SERIAL_MSG("\r\n");

					currNode = currNode -> nextNode;
				}
				currNode = CoordinatorNode;
			}
			else if(strcmp(key,"DELETE_LIST") == 0){

				list_clearNodes(CoordinatorNode->nextNode);
				currNode = CoordinatorNode->nextNode;
				lastNode = CoordinatorNode->nextNode;
				SEND_SERIAL_MSG("Deleting...\r\n");
				SEND_SERIAL_MSG("Done!!!\r\n");
			}
			else if(strcmp(key,"LIST_BY_ADD") == 0){

				if(currNode == NULL)
					currNode = CoordinatorNode;
				currNode = list_findNodeByAdd(atoi(value),currNode,CoordinatorNode);

				if(currNode == NULL){
					SEND_SERIAL_MSG("MSG#Node_not_found");
					currNode = CoordinatorNode;
				}
				else{
					SEND_SERIAL_MSG("MSG#Id:");
					SEND_SERIAL_BYTE(currNode->id / 10 + 0x30);
					SEND_SERIAL_BYTE(currNode->id % 10 + 0x30);
					SEND_SERIAL_MSG("\r\n");
				}
			}
			else if(strcmp(key,"LIST_BY_ID") == 0){

				if(currNode == NULL)
					currNode = CoordinatorNode;
				currNode = list_findNodeById(atoi(value),currNode,CoordinatorNode);

				if(currNode == NULL){
					SEND_SERIAL_MSG("MSG#Node_not_found");
					currNode = CoordinatorNode;
				}
				else{
					SEND_SERIAL_MSG("MSG#Address:");
					itoa(currNode->addressLow,xbeeAddressLowString, 16);
					SEND_SERIAL_MSG(xbeeAddressLowString);
					SEND_SERIAL_MSG("\r\n");
				}
			}
			else if(strcmp(key,"SET_DISC_OPT") == 0){
				xbeeApplyParamter("NO",atoi(value),AT_FRAME_ID_APPLY);
			}
			else if(strcmp(key,"GET_DISC_OPT") == 0){
				askXbeeParam("NO",AT_FRAME_ID_REQUEST);
			}
			else if(strcmp(key,"PRINT_NODE_LIST") == 0){

				SEND_SERIAL_MSG("MSG#Id\tAddress\r\n");
				currNode = CoordinatorNode;

				do{
					SEND_SERIAL_MSG("MSG#");
					SEND_SERIAL_BYTE(currNode->id / 10 + 0x30);
					SEND_SERIAL_BYTE(currNode->id % 10 + 0x30);
					SEND_SERIAL_BYTE('\t');
					itoa(currNode->addressLow,xbeeAddressLowString, 16);
					SEND_SERIAL_MSG(xbeeAddressLowString);
					SEND_SERIAL_MSG("\r\n");

					currNode = currNode -> nextNode;
				}while(currNode != NULL);
			}
			else if(strcmp(key,"PRINT_TIME_NODE") == 0){

				xbeeTransmitString[0] = 'C';
				xbeeTransmitString[1] = ' ';
				xbeeTransmitString[2] = 0x1F;
				xbeeTransmitString[3] = '\0';
				currNode = list_findNodeById(atoi(value), currNode, CoordinatorNode);
				if(currNode == NULL){
					SEND_SERIAL_MSG("list_findNodeById error \r\n");
					currNode = CoordinatorNode;
				}
				else{
					transmitRequest(currNode->addressHigh,currNode->addressLow,TRANSOPT_DISACK, 0x00,xbeeTransmitString);
				}
			}
			else{
				// VERYWRONG DATA
				SEND_SERIAL_MSG("MSG#WRONG_INPUT_DATA\r\n");
				SEND_SERIAL_MSG("MSG#>>>PRINT_CMD<<<\r\n");
				SEND_SERIAL_MSG("MSG#>>>PRINT_ACC_CMD<<<\r\n");
				SEND_SERIAL_MSG("MSG#>>>PRINT_DEBUG_CMD<<<\r\n");
			}
    		serialUpdated = false;
    	}
    	if(xbeeDataUpdated == true){

		switch(xbeeReceiveBuffer[XBEE_TYPE_OF_FRAME_INDEX]){
		case XBEE_AT_COMMAND_RESPONSE:

		if(strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "SH", 2) == 0) {
			if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
				SEND_SERIAL_MSG("DEBUG#ADDRESS_HIGH#");
				xbeeAddressHigh = (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA] << 24)
						+(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+1] << 16)
						+(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+2] << 8)
						+(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+3]);
				itoa(xbeeAddressHigh,xbeeAddressHighString, 10);
				SEND_SERIAL_MSG(xbeeAddressHighString);
				SEND_SERIAL_MSG("\r\n");
			}else{
				SEND_SERIAL_MSG("DEBUG#SH_AT_COMMAND_REQUEST_ERROR\r\n");
			}
		}else if(strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "SL", 2) == 0) {

			if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
				SEND_SERIAL_MSG("DEBUG#ADDR_LOW#");
				xbeeAddressLow = (xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA] << 24)
						+(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+1] << 16)
						+(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+2] << 8)
						+(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA+3]);
				itoa(xbeeAddressLow,xbeeAddressLowString, 10);
				SEND_SERIAL_MSG(xbeeAddressLowString);
				SEND_SERIAL_MSG("\r\n");
			}
			else{
				SEND_SERIAL_MSG("DEBUG#SL_AT_COMMAND_REQUEST_ERROR\r\n");
			}
		}else if(strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "FN", 2) == 0) {

			if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){

				xbeeAddressHigh = 0;
				xbeeAddressLow = 0;

				xbeeAddressHigh = (xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX] << 24)
						+(xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX+1] << 16)
						+(xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX+2] << 8)
						+(xbeeReceiveBuffer[XBEE_FN_ADDRESSH_INDEX+3]);

				xbeeAddressLow = (xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX] << 24)
						+(xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX+1] << 16)
						+(xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX+2] << 8)
						+(xbeeReceiveBuffer[XBEE_FN_ADDRESSL_INDEX+3]);


				if(xbeeAddressLow != COORDINATOR_ADDR_LOW){
					list_addNode(&lastNode,xbeeAddressLow);
					lastNode->rssi = xbeeReceiveBuffer[length-1];
					SEND_SERIAL_MSG("MSG#");
					SEND_SERIAL_BYTE(lastNode->id / 10 + 0x30);
					SEND_SERIAL_BYTE(lastNode->id % 10 + 0x30);
					SEND_SERIAL_BYTE('\t');
					SEND_SERIAL_BYTE(lastNode->rssi / 10 + 0x30);
					SEND_SERIAL_BYTE(lastNode->rssi % 10 + 0x30);
					SEND_SERIAL_BYTE('\t');
					itoa(xbeeAddressLow,xbeeAddressLowString, 16);
					SEND_SERIAL_MSG(xbeeAddressLowString);
					SEND_SERIAL_MSG("\r\n");
				}
				else{
					CoordinatorNode->rssi = xbeeReceiveBuffer[length-1];
/*					SEND_SERIAL_MSG("MSG#");
					SEND_SERIAL_BYTE(CoordinatorNode->id / 10 + 0x30);
					SEND_SERIAL_BYTE(CoordinatorNode->id % 10 + 0x30);
					SEND_SERIAL_BYTE('\t');
					SEND_SERIAL_BYTE(CoordinatorNode->rssi / 10 + 0x30);
					SEND_SERIAL_BYTE(CoordinatorNode->rssi % 10 + 0x30);
					SEND_SERIAL_BYTE('\t');
					itoa(CoordinatorNode->addressLow,xbeeAddressLowString, 10);
					SEND_SERIAL_MSG(xbeeAddressLowString);
					SEND_SERIAL_MSG(" - Coordinator");
					SEND_SERIAL_MSG("\r\n");*/
				}
			}
			else{
				SEND_SERIAL_MSG("DEBUG#FN_AT_COMMAND_REQUEST_ERROR\r\n");
			}
		}else if(strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "ND", 2) == 0) {

			if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){

				//FIND_NEIGHBORS
				uint8_t counter = 0;

				while(counter++ < length){
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[counter]);
				}

				SEND_SERIAL_MSG("\r\n");
			}
			else{
				SEND_SERIAL_MSG("DEBUG#ND_AT_COMMAND_REQUEST_ERROR\r\n");
			}
		}else if (strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "PL", 2) == 0) {
			if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_REQUEST){
				if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
					SEND_SERIAL_MSG("DEBUG#SERIAL POWER LEVEL:");
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA]+ASCII_DIGIT_OFFSET);
					SEND_SERIAL_MSG("\r\n");

				}else{
					SEND_SERIAL_MSG("PL_AT_COMMAND_REQUEST_ERROR\r\n");
				}
			}else if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_APPLY){
				if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
					SEND_SERIAL_MSG("DEBUG#PL_AT_COMMAND_APPLIED\r\n");
					askXbeeParam("PL",AT_FRAME_ID_REQUEST);

				}else{
					SEND_SERIAL_MSG("DEBUG#PL_AT_COMMAND_APPLY_ERROR\r\n");
				}
			}
		}else if (strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "NO", 2) == 0) {
			if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_REQUEST){
				if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
					SEND_SERIAL_MSG("DEBUG#NO_OPTIONS:");
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_AT_COMMAND_DATA]+ASCII_DIGIT_OFFSET);
					SEND_SERIAL_MSG("\r\n");

				}else{
					SEND_SERIAL_MSG("NO_AT_COMMAND_REQUEST_ERROR\r\n");
				}
			}else if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_APPLY){
				if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
					SEND_SERIAL_MSG("DEBUG#NO_AT_COMMAND_APPLIED\r\n");
					askXbeeParam("NO",AT_FRAME_ID_REQUEST);

				}else{
					SEND_SERIAL_MSG("DEBUG#NO_AT_COMMAND_APPLY_ERROR\r\n");
				}
			}
		}else if (strncmp((char*)&xbeeReceiveBuffer[XBEE_AT_COMMAND_INDEX], "CM", 2) == 0) {
			if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_REQUEST){
				if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
					channelMask = 0x00;
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

					tmpcounter = 0;
					SEND_SERIAL_MSG("DEBUG#Channels_enabled");
					for( ;tmpcounter < 30; tmpcounter++){
						if(channelMask & (1 << tmpcounter)){
							SEND_SERIAL_BYTE('_');
							SEND_SERIAL_BYTE(tmpcounter/10 + ASCII_DIGIT_OFFSET);
							SEND_SERIAL_BYTE(tmpcounter%10 + ASCII_DIGIT_OFFSET);
						}
					}
					SEND_SERIAL_MSG("\r\n");
				}else{
					SEND_SERIAL_MSG("CM_AT_COMMAND_REQUEST_ERROR\r\n");
				}
			}else if(xbeeReceiveBuffer[XBEE_FRAME_ID_INDEX] == AT_FRAME_ID_APPLY){
				if(xbeeReceiveBuffer[XBEE_AT_COMMAND_STATUS] == 0){
					SEND_SERIAL_MSG("DEBUG#CM_AT_COMMAND_APPLIED\r\n");
					askXbeeParam("CM",AT_FRAME_ID_REQUEST);

				}else{
					SEND_SERIAL_MSG("DEBUG#CM_AT_COMMAND_APPLY_ERROR\r\n");
				}
			}
		}
		else {
			SEND_SERIAL_MSG("DEBUG#UNEXPECTED_AT_COMMAND_RESPONSE XBEE PACKET\r\n");
		}
		break;
		case XBEE_RECEIVE_PACKET:

			receivedAddressHigh = 0x00;
			receivedAddressLow = 0x00;

			for(iterator = 1; iterator < 9; iterator++){	//Read address from received packet

				if(iterator<5){
					receivedAddressHigh |= xbeeReceiveBuffer[iterator] << 8*(4-iterator);
				}
				else{
					receivedAddressLow |= xbeeReceiveBuffer[iterator] << 8*(8-iterator);
				}
			}
			if(currNode == NULL){
				Usart1_SendString("MSG#currNode is NULL, giving a Coordinator reference...\r\n");
				currNode = CoordinatorNode;
			}
			currNode = list_findNodeByAdd(receivedAddressLow,currNode,CoordinatorNode);
			if(currNode == NULL){
				SEND_SERIAL_MSG("MSG#list_findNodeByAdd error, giving a Coordinator reference... \r\n");

				currNode = CoordinatorNode;

				xbeeReceiveBuffer[XBEE_DATA_MODE_OFFSET] = 'E';
			}

			if(xbeeReceiveBuffer[XBEE_DATA_MODE_OFFSET] == 'M'){
				/*
				 * Serial node should not receive any measurements
				 */
			}
			else if(xbeeReceiveBuffer[XBEE_DATA_MODE_OFFSET] == 'E'){

				Usart1_SendString("MSG#Unlisted receptment, not parsing packet...\r\n");
			}
			else if(xbeeReceiveBuffer[XBEE_DATA_MODE_OFFSET] == 'C'){
				/*
				 * Example
				 * C 0 -> Request for time stamp
				 * C 1
				 */
				switch(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET]){

				case (0x80):
					SEND_SERIAL_MSG("MSG#TIMER#SYNCHRONIZED...\r\n");
					TIM_SetCounter(TIM2,atoi(&xbeeReceiveBuffer[20]) + TIMER_SYNC_DELAY);
					SEND_SERIAL_MSG("MSG#SERIAL_ID#");
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[16]/10 + 0x30);
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[16]%10 + 0x30);
					SEND_SERIAL_BYTE('\r');
					SEND_SERIAL_BYTE('\n');
					SEND_SERIAL_MSG("MSG#NODES_TOTAL#");
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[18]/10 + 0x30);
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[18]%10 + 0x30);
					SEND_SERIAL_BYTE('\r');
					SEND_SERIAL_BYTE('\n');
					SEND_SERIAL_MSG("MSG#CURRENT_TIME#");
					itoa(TIM_GetCounter(TIM2),timerString,10);
					SEND_SERIAL_MSG(timerString);
					SEND_SERIAL_MSG("\r\n");
					break;
				case (0x81):
					//Command positive response
					SEND_SERIAL_BYTE(currNode->id + ASCII_DIGIT_OFFSET);
					SEND_SERIAL_MSG("#NODE_CMD_ACCEPTED#");
					SEND_SERIAL_MSG("\r\n");
					break;
				case (0x82):
					SEND_SERIAL_MSG("MSG#RSSI_THRESHOLD#");
					SEND_SERIAL_MSG(&xbeeReceiveBuffer[16]);
					SEND_SERIAL_MSG("\r\n");
					break;
				case (0x83):
					SEND_SERIAL_MSG("MSG#ACC_THRESHOLD#");
					SEND_SERIAL_MSG(&xbeeReceiveBuffer[16]);
					SEND_SERIAL_MSG("\r\n");
					break;
				case (0x84):
					SEND_SERIAL_MSG("MSG#GPS_THRESHOLD#");
					SEND_SERIAL_MSG(&xbeeReceiveBuffer[16]);
					SEND_SERIAL_MSG("\r\n");
					break;
				case (0x85):
					SEND_SERIAL_MSG("MSG#TIM_THRESHOLD#");
					SEND_SERIAL_MSG(&xbeeReceiveBuffer[16]);
					SEND_SERIAL_MSG("\r\n");
					break;
				case (0x86):
					SEND_SERIAL_MSG("MSG#BATTERY#");
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[16]);
					SEND_SERIAL_BYTE('.');
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[17]);
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[18]);
					SEND_SERIAL_MSG("\r\n");
					break;
				case (0x87):
					//Readiness response
					//"C N#<state><\n>

					SEND_SERIAL_BYTE(currNode->id + ASCII_DIGIT_OFFSET);
					SEND_SERIAL_MSG("#NODE_");
					if((xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2])&0x01){
						SEND_SERIAL_MSG("ACC#");
					}
					if(((xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]) >> 1)&0x01){
						SEND_SERIAL_MSG("GPS#");
					}
					if(((xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]) >> 2)&0x01){
						SEND_SERIAL_MSG("SD#");
					}
					if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x00){
						SEND_SERIAL_MSG("NOT_INITIALIZED\r\n");
					}
					else{
						SEND_SERIAL_MSG("IDLE_AND_READY\r\n");
					}
					break;
				case (0x88):
					//Transfered accelerometer and RSSI measurement
					//"C E#3#30#7#1224"
					rErrorByte = xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+7] - 0x30;
					//2o3 routine
					if(rErrorByte > 4 || rErrorByte == 3){
						SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);
						SEND_SERIAL_MSG("#ALARM#\r\n");
					}

					//id
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);
					SEND_SERIAL_BYTE('#');

					if(rErrorByte & 0x01){
						//If ACC error
						SEND_SERIAL_MSG("ACC_DNG#");
					}
					else{
						SEND_SERIAL_MSG("ACC_REL#");
					}

					/*
					 * Split absolute and relative measurement
					 */

					str_splitter(&xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+8],xbeeReceivedRelative,xbeeReceivedAbsolute,s_delimiter);
					SEND_SERIAL_MSG(xbeeReceivedRelative);
					SEND_SERIAL_MSG("\r\n");

					/*
					 * Print absolute coordinator measurement
					 */

					//id
					SEND_SERIAL_BYTE('0');
					SEND_SERIAL_BYTE('#');
					SEND_SERIAL_MSG("ACC_ABS#");
					SEND_SERIAL_MSG(xbeeReceivedAbsolute);
					SEND_SERIAL_MSG("\r\n");

					/*
					 * Print RSSI measurement
					 */

					//id
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);
					SEND_SERIAL_BYTE('#');
					if((rErrorByte >> 1)&0x01){
						//If RSSI error
						SEND_SERIAL_MSG("RSSI_DNG#");
					}
					else{
						SEND_SERIAL_MSG("RSSI_REL#");
					}
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+4]);
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+5]);
					SEND_SERIAL_MSG("\r\n");
					break;

				case (0x89):
					//Transfered velocity and RSSI measurement
					//"C F#3#30#7#5.8"
					rErrorByte = xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+7] - 0x30;

					//id
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);
					SEND_SERIAL_BYTE('#');

					if((rErrorByte >> 2)&0x01){
						//If GPS error
						SEND_SERIAL_MSG("GPS_DNG");
					}
					else{
						SEND_SERIAL_MSG("GPS_REL");
					}
					str_splitter(&xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+8],xbeeReceivedRelative,xbeeReceivedAbsolute,s_delimiter);
					SEND_SERIAL_MSG(xbeeReceivedRelative);
					SEND_SERIAL_MSG("\r\n");
					/*
					 * Print absolute measurement
					 */
					//id
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);
					SEND_SERIAL_BYTE('#');
					SEND_SERIAL_MSG("GPS_ABS");
					SEND_SERIAL_MSG(xbeeReceivedAbsolute);
					SEND_SERIAL_MSG("\r\n");

					//id
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);
					SEND_SERIAL_BYTE('#');
					if((rErrorByte >> 1)&0x01){
						//If RSSI error
						SEND_SERIAL_MSG("RSSI_DNG#");
					}
					else{
						SEND_SERIAL_MSG("RSSI_REL#");
					}
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+4]);
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+5]);
					SEND_SERIAL_MSG("\r\n");

					/*
					 * Send absolute GPS
					 */


					/*
					 * Send absolute RSSI
					 */

					break;
				case (0x8A):
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);
					SEND_SERIAL_BYTE('#');
					SEND_SERIAL_MSG("COORD_NODE_LAT_LONG#");
					SEND_SERIAL_MSG(&xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+4]);
					SEND_SERIAL_MSG("\r\n");
				break;
				case (0x8B):
					SEND_SERIAL_MSG("MSG#NODE#");
					//id
					SEND_SERIAL_BYTE(currNode->id + ASCII_DIGIT_OFFSET);
					SEND_SERIAL_BYTE('#');
					SEND_SERIAL_MSG("POWER_LEVEL#");
					SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] + ASCII_DIGIT_OFFSET);
					SEND_SERIAL_MSG("\r\n");
				break;
				case (0x8C):
					tmpmask = atoi(&xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]);

					SEND_SERIAL_MSG("MSG#NODE#");
					//id
					SEND_SERIAL_BYTE(currNode->id + ASCII_DIGIT_OFFSET);
					SEND_SERIAL_BYTE('#');
					SEND_SERIAL_MSG("CHANNELS_USED#");
					tmpcounter = 0;

					for( ;tmpcounter < 30; tmpcounter++){
						if(tmpmask & (1 << tmpcounter)){
							SEND_SERIAL_BYTE(tmpcounter/10 + ASCII_DIGIT_OFFSET);
							SEND_SERIAL_BYTE(tmpcounter%10 + ASCII_DIGIT_OFFSET);
							SEND_SERIAL_BYTE(' ');
						}
					}
					SEND_SERIAL_MSG("\r\n");
				break;
				case (0x8D):	

					/*
					 * ACC EXPERIMENT RESPONSE
					 */

					SEND_SERIAL_MSG("Huge chunk of bytes received: ");

					uint8_t sampleCounter;
					char tmpAccString[6];

					for(sampleCounter = 0; sampleCounter<1;sampleCounter++){

						itoa( (xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2+sampleCounter] )
								+ (xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+3+sampleCounter] << 8), tmpAccString, 10);
						SEND_SERIAL_MSG(tmpAccString);
						SEND_SERIAL_MSG("\r\n");
					}
				break;
				case (0x8E):
					SEND_SERIAL_BYTE(lastNode->id + 0x30);
					SEND_SERIAL_BYTE('#');
					SEND_SERIAL_MSG("TIM_DNG#");
					SEND_SERIAL_MSG("\r\n");
				break;
				case (0x8F):
					SEND_SERIAL_BYTE(currNode->id + ASCII_DIGIT_OFFSET);
					SEND_SERIAL_BYTE('#');
					SEND_SERIAL_MSG("ERROR#");
					if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x01){
						/*
						 * ACC_INIT_ERROR
						 */
						SEND_SERIAL_MSG("ACC_INIT");
					}
					else if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x02){
						/*
						 * GPS_INIT_ERROR
						 */
						SEND_SERIAL_MSG("GPS_INIT");
					}
					else if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x03){
						/*
						 * SD_INIT_ERROR
						 */
						SEND_SERIAL_MSG("SD_INIT");
					}else if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x04){
						/*
						 * NODE_DISCOVERY_ERROR
						 */
						SEND_SERIAL_MSG("NODE_DISCOVERY");
					}else if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x05){
						/*
						 * LIST_FUNCTION_ERROR
						 */
						SEND_SERIAL_MSG("LIST_FUNCTION()");
					}
					else if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x06){
						/*
						 * LIST_FUNCTION_ERROR
						 */
						SEND_SERIAL_MSG("LIST_SORTING()");
					}
					else if(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] == 0x07){
						/*
						 * LIST_FUNCTION_ERROR
						 */
						SEND_SERIAL_MSG("NO_NODES_FOUND");
					}

					SEND_SERIAL_MSG("\r\n");
				break;
				case (0x90):
					//SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] + 0x30);
					SEND_SERIAL_MSG("MSG#DISC#");
					SEND_SERIAL_BYTE((xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+4] / 10) + 0x30);
					SEND_SERIAL_BYTE((xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+4] % 10) + 0x30);
					SEND_SERIAL_BYTE('#');
					xbeeAddressLow = 0;
					xbeeAddressLow = (xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+6] << 24)
							+(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+7] << 16)
							+(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+8] << 8)
							+(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+9]);
					itoa(xbeeAddressLow,xbeeAddressLowString, 16);
					SEND_SERIAL_MSG(xbeeAddressLowString);
					SEND_SERIAL_BYTE('\r');
					SEND_SERIAL_BYTE('\n');

					nodesFound = xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2];

				break;

				case (0x91):
					//NETWORK LIST RESPONSE

					xbeeAddressLow = (xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+4] << 24)
							+(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+5] << 16)
							+(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+6] << 8)
							+(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+7]);

					currNode = list_addNode(&lastNode,xbeeAddressLow);

					if(currNode->id != xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2]){
						//Print list population error
						SEND_SERIAL_MSG("DEBUG#LIST_RECEIPTMENT_ERROR\r\n");
						SEND_SERIAL_MSG("DEBUG#id mismatch:");
						SEND_SERIAL_BYTE(currNode->id / 10 +0x30);
						SEND_SERIAL_BYTE(currNode->id % 10 +0x30);
						SEND_SERIAL_MSG(" and ");
						SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] / 10 +0x30);
						SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] % 10 +0x30);
						SEND_SERIAL_MSG("\r\n");
					}
					else if(currNode->id == nodesFound)  {
						//If all nodes are added, print list
						//SEND_SERIAL_MSG("MSG#Id#Address\r\n");
						currNode = CoordinatorNode;

						do{
							SEND_SERIAL_BYTE(currNode->id + 0x30);
							SEND_SERIAL_BYTE('#');
							SEND_SERIAL_MSG("ADDR#");
							itoa(currNode->addressLow,xbeeAddressLowString, 16);
							SEND_SERIAL_MSG(xbeeAddressLowString);
							SEND_SERIAL_MSG("\r\n");

							currNode = currNode -> nextNode;
						}while(currNode != NULL);

						currNode = CoordinatorNode;
					}
				break;

				case (0x92):
					SEND_SERIAL_MSG("MSG#DONE,TOTALLY#");
					SEND_SERIAL_BYTE((xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] / 10) + 0x30);
					SEND_SERIAL_BYTE((xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2] % 10) + 0x30);

					nodesFound = xbeeReceiveBuffer[XBEE_DATA_TYPE_OFFSET+2];
				break;
				default:
					SEND_SERIAL_MSG("MSG#UNEXPECTED_XBEE_COMMAND...\r\n");
					break;
				}
			}
			break;
			case XBEE_MODEM_STATUS:
			SEND_SERIAL_MSG("DEBUG#MODEM STATUS#");
			if(xbeeReceiveBuffer[1] == 0x00){
				SEND_SERIAL_MSG("HARDWARE RESET...\r\n");
			}
			else{
				SEND_SERIAL_MSG("UNEXPECTED RESET...\r\n");
			}
			break;
			case XBEE_TRANSMIT_STATUS:
			SEND_SERIAL_MSG("DEBUG#TRANSMIT STATUS");
			if(xbeeReceiveBuffer[5] == 0x00){
				SEND_SERIAL_MSG("#SUCCESS...\r\n");
			}
			else{
				SEND_SERIAL_MSG("#FAIL-");
				SEND_SERIAL_BYTE(xbeeReceiveBuffer[5]);
				SEND_SERIAL_MSG(" \r\n");
			}
			break;
			/*
			 * Parse Any other XBEE packet
			 */
			default:
				SEND_SERIAL_MSG("DEBUG#UNEXPECTED XBEE PACKET(dec):");
				SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_TYPE_OF_FRAME_INDEX]/100 + ASCII_DIGIT_OFFSET);
				SEND_SERIAL_BYTE((xbeeReceiveBuffer[XBEE_TYPE_OF_FRAME_INDEX]%100)/10 + ASCII_DIGIT_OFFSET);
				SEND_SERIAL_BYTE(xbeeReceiveBuffer[XBEE_TYPE_OF_FRAME_INDEX]%10 + ASCII_DIGIT_OFFSET);
				SEND_SERIAL_MSG("\r\n");
			break;
			}
    		xbeeDataUpdated = false;
    	}
    	/*
    	 * Check whether there were no unread XBEE data
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
				//printf("Lenght: %d\n", length);
				uint8_t i = 0;
				for(; i < length; i ++ ){				//Read data based on packet length
					xbeeReceiveBuffer[i] = SPI1_TransRecieve(0x00);
					cheksum += xbeeReceiveBuffer[i];
				}
				cheksum += SPI1_TransRecieve(0x00);
				if(cheksum == 0xFF){
					xbeeDataUpdated = true;
				}
				//printf("Checksum:%d\n",cheksum);
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

// Serial data interrupt handler
void USART1_IRQHandler(void){
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){

		Usart1_Send(serialBuffer[packetLenght++] = USART_ReceiveData(USART1));
		//serialBuffer[packetLenght] = USART_ReceiveData(USART1);
		if(serialBuffer[packetLenght-1] == '\r' || serialBuffer[packetLenght-1] == '\n'){
			serialBuffer[packetLenght-1] = '\0';
			TOGGLE_REDLED_SERIAL();
			serialUpdated = true;
			packetLenght = 0;
			Usart1_Send('\r');
			Usart1_Send('\n');
		}
	}
}

void EXTI4_15_IRQHandler(void){

	if(EXTI_GetITStatus(EXTI_Line4) == SET){	//Handler for Radio ATTn pin interrupt (data ready indicator)

		TOGGLE_REDLED_XBEE();
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

void TIM15_IRQHandler()
{
	if(TIM_GetITStatus(TIM15, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM15, TIM_IT_Update);
		SEND_SERIAL_MSG("800ms\r\n");
	}
}
void TIM14_IRQHandler()
{
	if(TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);

		accBuff[accBuffValue++] = returnZ_axis();

		char accString[6];
		itoa(accBuff[accBuffValue-1], accString, 10);
		SEND_SERIAL_MSG(accString);
		SEND_SERIAL_MSG("\r\n");

		if(accBuffValue > ACC_BUFFER_SIZE-1){
			accBuffValue = 0;
		}

	}
}
