/* 
 * File:   PWM.h
 * Author: Diego
 *
 * Created on 15 de abril de 2023, 10:59 PM
 */

#ifndef PWM_H
#define	PWM_H
#include <xc.h>
int chn1, chn2;
float periodo_ms;
int chn;

void PWM_init (int chn1,int chn2,float periodo_ms);
void PWM_duty (int chn,float duty);
#endif	/* PWM_H */

