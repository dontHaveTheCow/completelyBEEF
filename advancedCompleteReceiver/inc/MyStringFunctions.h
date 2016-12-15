#ifndef STRING_LIBRARY
#define STRING_LIBRARY

#include "string.h"
#include <stdio.h>

void reverse(char s[]);
/*void itoa(int n, char s[]);*/
float stof(const char* s);
void rreverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);
int pow(int base, int afterpoint);

void str_splitter(char* buffer, char* _key, char* _value, char* delimiter);

#endif
