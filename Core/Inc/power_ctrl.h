#ifndef power_ctrl

#define power_ctrl

#include "stdbool.h"

void POWER_OFF(void);
void POWER_ON(void);
void CURRENT_RESET(void);
bool GET_GPIO_STATE(GPIO_TypeDef*, uint16_t);

#endif
