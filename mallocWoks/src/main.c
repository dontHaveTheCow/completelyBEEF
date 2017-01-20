#include "stm32f0xx.h"
#include "IndicationGPIOs.h"
#include "USART1.h"

#include "stdlib.h"
#include "string.h"

int main(void)
{
	initializeEveryRedLed();
	initializeEveryGreenLed();
	Usart1_Init(BAUD_9600);

	char *str;
	str = (char *) malloc(15);
	strcpy(str,"malloced string");
	Usart1_SendString(str);

	str = (char *) realloc(str,32);
	itoa(&str,str,16);
	Usart1_SendString("\r\nAddress:0x");
	Usart1_SendString(str);

	free(str);

	for(;;);
}
