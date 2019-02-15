#include <SevenSegmentTM1637.h>
#include "SevenSegmentExtended.h"


#ifdef ESP8266
#define CLK 5
#define DIO 4
#else
#define CLK 3
#define DIO 4
#endif

SevenSegmentExtended display(CLK, DIO);

int numberToDisplay;
int displayData;
