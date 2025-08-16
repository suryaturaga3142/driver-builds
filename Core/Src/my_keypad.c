/*
 * my_keypad.c
 *
 *  Created on: Jun 17, 2025
 *      Author: surya
 */

#include "stm32f7xx_hal.h"
#include "my_keypad.h"

void myKeyPad_Init (KeyPad * keypad, MyGPIO drivers[], MyGPIO readers[]) {
	keypad->drivers = drivers;
	keypad->readers = readers;
	for (int i = 0; i < KEYPAD_ROWS; i++) {
		HAL_GPIO_WritePin(keypad->drivers[i].GPIOx, keypad->drivers[i].GPIO_Pin, GPIO_PIN_RESET);
	}
	keypad->num = 0;
	return;
}

void myKeyPad_Read (KeyPad * keypad) {
	keypad->num = 0;
	for (int i = 0; i < KEYPAD_ROWS; i++) {
		HAL_GPIO_WritePin(keypad->drivers[i].GPIOx, keypad->drivers[i].GPIO_Pin, GPIO_PIN_SET);
		for (int j = 0; j < KEYPAD_ROWS; j++) {
			GPIO_PinState pb = HAL_GPIO_ReadPin(keypad->readers[j].GPIOx, keypad->readers[j].GPIO_Pin);
			keypad->buttons[KEYPAD_ROWS - 1 - j][i] = pb;
			int cur_num = 1 + KEYPAD_ROWS * (KEYPAD_ROWS - 1 - j) + i;
			keypad->num = pb && cur_num > keypad->num ? cur_num : keypad->num;
		}
		HAL_GPIO_WritePin(keypad->drivers[i].GPIOx, keypad->drivers[i].GPIO_Pin, GPIO_PIN_RESET);
	}
	return;
}

void myKeyPad_DeInit (KeyPad * keypad) {
	for (int i = 0; i < KEYPAD_ROWS; i++) {
		for (int j = 0; j < KEYPAD_ROWS; j++) {
			keypad->buttons[i][j] = GPIO_PIN_RESET;
		}
	}
	keypad->drivers = NULL;
	keypad->readers = NULL;
	keypad->num = 0;
	return;
}
