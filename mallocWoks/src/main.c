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
struct node* list_addNode(struct node** ptrToLast, uint32_t addressL);
struct node* list_findNode(uint32_t addressL, struct node* curr,struct node* root);
void list_printList(struct node* root);

void list_setAddress(struct node* currNode, uint32_t high, uint32_t low);


int main(void)
{
	initializeEveryRedLed();
	initializeEveryGreenLed();
	initialiseSysTick();
	Usart1_Init(BAUD_9600);

	struct node* rootNode = list_createRoot();
	struct node* lastNode = rootNode;
	struct node* currNode;

	list_setAddress(rootNode, 0x0013A200, 0x409783D9);
	list_addNode(&lastNode, 0x409783DA);
	list_addNode(&lastNode, 0x409783DB);
	currNode = list_addNode(&lastNode, 0x409783DC);
	list_addNode(&lastNode, 0x409783DD);

	Usart1_SendString("------printing list----------\r\n");
	list_printList(rootNode);
	Usart1_SendString("------searching node----------\r\n");
	currNode = list_findNode(0x409783DF,currNode,rootNode);
	Usart1_SendString("Node ");
	Usart1_Send(currNode->id + 0x30);
	Usart1_SendString(" with address:0x409783DB\r\n");


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

struct node* list_addNode(struct node** ptrToLast, uint32_t addressL){

	if(*ptrToLast == NULL){
		return list_createRoot();
	}

	(*ptrToLast)->nextNode = malloc(sizeof(struct node));

	if((*ptrToLast)->nextNode == NULL){
		return NULL;
	}

	(*ptrToLast)->nextNode->addressLow = addressL;
	(*ptrToLast)->nextNode->id = (*ptrToLast)->id+1;
	(*ptrToLast)->nextNode->nextNode = NULL;
	(*ptrToLast) = (*ptrToLast)->nextNode;

	return (*ptrToLast);
}

struct node* list_findNode(uint32_t addressL, struct node* curr,struct node* root){

	struct node* tmp = curr;

	do{
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
	while(curr != tmp);

	return NULL;
}

void list_printList(struct node* root){

	struct node* tmp = root;
	char txString[16];

	Usart1_SendString("Id\'s \t Address\r\n");
	while(tmp != NULL){
		Usart1_Send(tmp->id + 0x30);
		Usart1_Send('\t');
		itoa(tmp->addressLow,txString, 16);
		Usart1_SendString(txString);
		Usart1_SendString("\r\n");
		tmp = tmp->nextNode;
	}
}


void list_setAddress(struct node* currNode, uint32_t high, uint32_t low){

	currNode->addressHigh = high;
	currNode->addressLow = low;
}
