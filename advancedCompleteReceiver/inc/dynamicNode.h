#ifndef DYNAMICNODE_H_
#define DYNAMICNODE_H_

#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"

struct node{
	uint32_t addressHigh;
	uint32_t addressLow;
	int16_t accMeasurment;
	uint8_t rssiMeasurment;
	uint8_t avrRSSI;
	uint32_t packetsTime;
	uint8_t avarageRSSIcount;
	float velocity;
	uint8_t errorByte;
	uint8_t state;

	uint8_t id;
	bool transferToGateway;
	struct node *nextNode;
};

struct node* list_createRoot(void);
struct node* list_addNode(struct node** ptrToLast, uint32_t addressL);
void list_setAddress(struct node* currNode, uint32_t high, uint32_t low);

struct node* list_findNodeByAdd(uint32_t addressL, struct node* curr,struct node* root);
struct node* list_findNodeById(uint8_t id, struct node* curr,struct node* root);

void list_setTransferForAll(struct node* root, bool yesOrNo);
//void list_printList(struct node* root);

#endif /* DYNAMICNODE_H_ */
