#ifndef DYNAMICNODE_H_
#define DYNAMICNODE_H_

#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"

struct node{
	uint32_t addressHigh;
	uint32_t addressLow;
	uint8_t rssi;

	uint8_t id;

	struct node *nextNode;
};

struct node* list_createRoot(void);
struct node* list_addNode(struct node** ptrToLast, uint32_t addressL);
void list_clearNodes(struct node* root);
void list_setAddress(struct node* currNode, uint32_t high, uint32_t low);

struct node* list_findNodeByAdd(uint32_t addressL, struct node* curr,struct node* root);
struct node* list_findNodeById(uint8_t id, struct node* curr,struct node* root);

//void list_printList(struct node* root);

#endif /* DYNAMICNODE_H_ */
