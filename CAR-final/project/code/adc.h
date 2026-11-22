#ifndef CODE_ADC_H_
#define CODE_ADC_H_
#include "code1.h"

#define ADC_power_supply                (ADC0_CH21_P07_5) //ADCÒý½Å
#define BUZZER_PIN                      (P19_4)           //·äÃùÆ÷Òý½Å

extern float voltage_power_supply;

void power_supply_init(void);

void buzzer_init(void);

void buzzer_on(void);

#endif /* CODE_ADC_H_ */