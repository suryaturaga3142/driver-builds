/*
 * my_keypad.h
 *
 *  Created on: Jun 17, 2025
 *      Author: surya
 */

#ifndef INC_MY_KEYPAD_H_
#define INC_MY_KEYPAD_H_

// Square Keypad
#define KEYPAD_ROWS 4

typedef struct {
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
} MyGPIO;

typedef struct {
	MyGPIO * drivers; // Headers 5 - 8: GPIO Output Pins
	MyGPIO * readers; // Headers 1 - 4: GPIO Pulldown Input Pins
	GPIO_PinState buttons[KEYPAD_ROWS][KEYPAD_ROWS]; //Results of keypad
	int num; //High priority switch number
} KeyPad;

void myKeyPad_Init (KeyPad * keypad, MyGPIO drivers[], MyGPIO readers[]);

void myKeyPad_Read (KeyPad * keypad);

void myKeyPad_DeInit (KeyPad * keypad);

#endif /* INC_MY_KEYPAD_H_ */
