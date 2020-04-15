#ifndef __OLED_H
#define __OLED_H
//#include "fonts.h"
//#define contrast 0x81
#define contrastLow 0x810F
#define contrastHigh 0x81FF
	void turnOn();
	void sendCMD(uint8_t *cmd, uint16_t size);
	void sendDATA(uint8_t *data, uint16_t size);
	void sendString(char *string, uint8_t header);
	void clearScreen();
	void updateScreen(char* hr, char* spo2, char* distance, char* user);
#endif
