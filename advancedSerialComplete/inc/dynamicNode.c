#include "dynamicNode.h"

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
	//Other values are assigned by default
	(*ptrToLast)->nextNode->addressHigh = 0x0013A200;
	(*ptrToLast)->nextNode->id = (*ptrToLast)->id+1;

	(*ptrToLast)->nextNode->nextNode = NULL;
	(*ptrToLast) = (*ptrToLast)->nextNode;

	return (*ptrToLast);
}




void list_setAddress(struct node* currNode, uint32_t high, uint32_t low){

	currNode->addressHigh = high;
	currNode->addressLow = low;
}

struct node* list_findNodeByAdd(uint32_t addressL, struct node* curr,struct node* root){

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

struct node* list_findNodeById(uint8_t id, struct node* curr,struct node* root){

	struct node* tmp = curr;

	do{
		if(curr->id == id){
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

/*void list_printList(struct node* root){

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
}*/
