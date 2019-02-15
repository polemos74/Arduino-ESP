#include <TM1637Display.h>


#ifdef ESP8266
#define CLK 5
#define DIO 4
#else
#define CLK 3
#define DIO 4
#endif

TM1637Display display(CLK, DIO);

uint8_t data[] = { 0x40, 0x40, 0x40, 0x40 };

int numberToDisplay;