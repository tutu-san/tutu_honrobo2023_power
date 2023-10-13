#include "main.h"
#include "power_ctrl.h"
#include "stdbool.h"

extern uint8_t SW_state;

enum {
	REMOTE_OFF, SW_OFF, ALL_ON
};

void POWER_ON(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET); //リレーON
}

void POWER_OFF(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET); //リレーOFF;
}

void CURRENT_RESET(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET); //過電流検知リセット
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET); //過電流検知セット
}

bool GET_GPIO_STATE(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	if (GPIOx == GPIOA) {
		return (((GPIOA->ODR) &  GPIO_Pin) != 0);
	} else if (GPIOx == GPIOB) {
		return (((GPIOB->ODR) &  GPIO_Pin) != 0);
	} else {
		return false;
	}

}

