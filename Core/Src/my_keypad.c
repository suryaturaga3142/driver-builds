/*
 * my_keypad.c
 *
 *  Created on: Jun 17, 2025
 *      Author: surya
 */

#include "main.h"
#include <stdio.h>

void myKeyPad_Init (KeyPad * keypad, MyGPIO drivers[], MyGPIO readers[]) {
	keypad->drivers = drivers;
	keypad->readers = readers;
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(keypad->drivers[i].GPIOx, keypad->drivers[1].GPIO_Pin, RESET);
	}
	keypad->num = 0;
	return;
}

void myKeyPad_Read (KeyPad * keypad) {
	keypad->num = 0;
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(keypad->drivers[i].GPIOx, keypad->drivers[i].GPIO_Pin, SET);
		for (int j = 0; j < 4; j++) {
			GPIO_PinState pb = HAL_GPIO_ReadPin(keypad->readers[j].GPIOx, keypad->readers[j].GPIO_Pin);
			keypad->buttons[3 - j][i] = pb;
			int cur_num = 1 + 4 * (3 - j) + i;
			keypad->num = pb && cur_num > keypad->num ? cur_num : keypad->num;
		}
		HAL_GPIO_WritePin(keypad->drivers[i].GPIOx, keypad->drivers[i].GPIO_Pin, RESET);
	}
	return;
}
