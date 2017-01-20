#include "stm32f0xx.h"
#include "IndicationGPIOs.h"
#include "USART1.h"
#include "SysTickDelay.h"

#include "stdlib.h"
#include "string.h"

struct node{
	uint32_t addressHigh;
	uint32_t addressLow;
	int16_t measurment[2];	//0 -> ACC 1 -> RSSI 2 -> GPS
	uint8_t avrRSSI;
	uint32_t packetsTime;
	uint8_t avarageRSSIcount;
	float velocity;
	uint8_t errorByte;
	uint8_t state;

	uint8_t id;
	struct node *nextNode;
};

struct node* list_createRoot(void);
struct node* list_addNode(struct node* head);
struct node* list_findNode(uint32_t addressL, struct node* curr, struct node* root);

void list_setAddress(struct node* currNode, uint32_t high, uint32_t low);


int main(void)
{
	initializeEveryRedLed();
	initializeEveryGreenLed();
	initialiseSysTick();
	Usart1_Init(BAUD_9600);

	char txString[16];

	Usart1_SendString("Size of a list element:");
	Usart1_Send(sizeof(struct node) / 10 + 0x30);
	Usart1_Send(sizeof(struct node) % 10 + 0x30);

	struct node* rootNode = list_createRoot();
	struct node* headNode = rootNode;
	struct node* currNode = rootNode;
	currNode = list_addNode(headNode);

	Usart1_SendString("Root id:");
	Usart1_Send(rootNode->id + 0x30);
	Usart1_SendString("\r\nNext id:");
	Usart1_Send(currNode->id + 0x30);

	list_setAddress(currNode, 0x0013A200, 0x409783D9);
	list_setAddress(rootNode, 0x0013A200, 0x40E3E13D);

	Usart1_SendString("\r\nId of a address 0x409783D9:0x");
	itoa(list_findNode(0x409783D9,currNode,rootNode)->id,txString,16);
	Usart1_SendString(txString);

	for(;;){
		delay_400ms();
		blinkAllRed();
	}
}

struct node* list_createRoot(void){

	struct node* ptrToRoot = (struct node*) malloc(sizeof(struct node));

	if(ptrToRoot == NULL)
		return NULL;
	ptrToRoot->nextNode = NULL;
	ptrToRoot->id = 0;

	return ptrToRoot;
}

struct node* list_addNode(struct node* head){

	if(head == NULL){
		return list_createRoot();
	}
	struct node* ptrToNext = (struct node*) malloc(sizeof(struct node));

	if(ptrToNext == NULL){
		return NULL;
	}
	ptrToNext->nextNode = NULL;
	ptrToNext->id = head->id + 1;

	head->nextNode = ptrToNext;
	head = ptrToNext;

	return head;
}

struct node* list_findNode(uint32_t addressL, struct node* curr, struct node* root){

	struct node* tmp = curr;

	while(curr->nextNode != tmp){

		if(curr->addressLow == addressL){
			return curr;
		}
		else if(curr->nextNode == NULL){
			//If current node reaches the end of a list
			curr = root;
		}
		else{
			curr = curr->nextNode;
		}
	}
	return NULL;
}


void list_setAddress(struct node* currNode, uint32_t high, uint32_t low){

	currNode->addressHigh = high;
	currNode->addressLow = low;
}
