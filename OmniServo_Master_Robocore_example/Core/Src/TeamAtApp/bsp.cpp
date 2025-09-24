///*
// * bsp.cpp
// *
// *  Created on: Oct 9, 2024
// *      Author: teamat
// */
//
//#include <bsp.h>
//
//#include "gpio.h"
//#include "main.h"
//
//managedButton buttonList[NB_BUTTONS] = {
//	 			{IN_BUTTON_1_Pin, (uint32_t *)IN_BUTTON_1_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE},// /*| DOUBLECLICK_ENABLE*/ | LONGCLICK_ENABLE | HOLD_ENABLE},
//	 			{IN_BUTTON_2_Pin, (uint32_t *)IN_BUTTON_2_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE},// /*| DOUBLECLICK_ENABLE*/ /*| LONGCLICK_ENABLE */| HOLD_ENABLE},
//	 			{IN_BUTTON_3_Pin, (uint32_t *)IN_BUTTON_3_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE},// /*| DOUBLECLICK_ENABLE*/ | LONGCLICK_ENABLE | HOLD_ENABLE},
//	 			{IN_BUTTON_4_Pin, (uint32_t *)IN_BUTTON_4_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE},// /*| DOUBLECLICK_ENABLE*/ | LONGCLICK_ENABLE | HOLD_ENABLE}
//				{IN_BUTTON_5_Pin, (uint32_t *)IN_BUTTON_5_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE},
//				{IN_BUTTON_6_Pin, (uint32_t *)IN_BUTTON_6_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE},
//				{IN_BUTTON_7_Pin, (uint32_t *)IN_BUTTON_7_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE},
//	 	};
//
//
//bsp::bsp() {
//	// TODO Auto-generated constructor stub
//
//}
//
//bsp::~bsp() {
//	// TODO Auto-generated destructor stub
//}
//
//
//void bsp::comPortLedControl(uint8_t portId, uint8_t portState)
//{
//	switch(portId)
//	{
//		case 0:
//			HAL_GPIO_WritePin(OUT_LED_4_GPIO_Port, OUT_LED_4_Pin, (GPIO_PinState)portState);
//			break;
//
//		case 1:
//			HAL_GPIO_WritePin(OUT_LED_3_GPIO_Port, OUT_LED_3_Pin, (GPIO_PinState)portState);
//			break;
//
//		case 2:
//			HAL_GPIO_WritePin(OUT_LED_2_GPIO_Port, OUT_LED_2_Pin, (GPIO_PinState)portState);
//			break;
//
//		default:
//			break;
//	}
//}
//
//void bsp::comPortPowerControl(uint8_t portId, uint8_t portState)
//{
//	switch(portId)
//	{
//		case 0:
//			HAL_GPIO_WritePin(OUT_MOTOR_1_EN_GPIO_Port, OUT_MOTOR_1_EN_Pin, (GPIO_PinState)portState);
//			break;
//
//		case 1:
//			HAL_GPIO_WritePin(OUT_MOTOR_2_EN_GPIO_Port, OUT_MOTOR_2_EN_Pin, (GPIO_PinState)portState);
//			break;
//
//		case 2:
//			HAL_GPIO_WritePin(OUT_MOTOR_3_EN_GPIO_Port, OUT_MOTOR_3_EN_Pin, (GPIO_PinState)portState);
//			break;
//
//		default:
//			break;
//	}
//
//}
//
//
//ButtonPollingManager buttonManager;
//bsp board;
//
//
//
//
//
//
//
