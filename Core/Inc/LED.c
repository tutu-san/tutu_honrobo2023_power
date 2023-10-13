//使用法
//LED.cをSrcフォルダ,LED.hをIncフォルダに入れてmain.cで#include "LED.h"
//303kを使うときは#define
#define F303K

#include "main.h"
#include "LED.h"

//ここでピンを設定

#ifdef F303K

GPIO_TypeDef *LED_R_PORT = GPIOB;
uint16_t LED_R_PIN = GPIO_PIN_1;
GPIO_TypeDef *LED_G_PORT = GPIOA;
uint16_t LED_G_PIN = GPIO_PIN_15;
GPIO_TypeDef *LED_B_PORT = GPIOB;
uint16_t LED_B_PIN = GPIO_PIN_0;

#else

GPIO_TypeDef* LED_R_PORT = GPIOC;
uint16_t LED_R_PIN = GPIO_PIN_14;
GPIO_TypeDef* LED_G_PORT = GPIOC;
uint16_t LED_G_PIN = GPIO_PIN_13;
GPIO_TypeDef* LED_B_PORT = GPIOC;
uint16_t LED_B_PIN = GPIO_PIN_15;

#endif

//赤
void SET_LED_R(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, SET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, RESET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, RESET);
}

//緑
void SET_LED_G(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, RESET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, SET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, RESET);
}

//青
void SET_LED_B(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, RESET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, RESET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, SET);
}

//黄
void SET_LED_Y(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, SET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, SET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, RESET);
}

//紫
void SET_LED_P(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, SET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, RESET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, SET);
}

//水
void SET_LED_S(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, RESET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, SET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, SET);
}

//白
void SET_LED_W(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, SET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, SET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, SET);
}

//消灯
void SET_LED_OFF(void) {
	HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, RESET);
	HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, RESET);
	HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, RESET);
}
