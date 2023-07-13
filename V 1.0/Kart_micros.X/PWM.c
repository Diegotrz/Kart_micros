/*
 * File:   PWM.c
 * Author: Diego
 *
 * Created on 15 de abril de 2023, 10:58 PM
 */


#include <xc.h>
#include "PWM.h"
void PWM_init (int chn1,int chn2, float periodo_ms){
    // Configuración del PWM
    TRISCbits.TRISC1 = chn2; //RC1/CCP2 como salida
    TRISCbits.TRISC2 = chn1;    //RC2/CCP1 como entrada
    
    PR2 = periodo_ms;              // Config del periodo
    CCP1CONbits.P1M =0;     // Config modo PWM
    CCP1CONbits.CCP1M =0b1100; 
    
    CCPR1L = 0x0f; //Ciclo de trabajo inicial
    CCP1CONbits.DC1B= 0;    
    //TMR2
    PIR1bits.TMR2IF =0; //Apagamos la bandera
    T2CONbits.T2CKPS = 0b11; //Prescaler 1:16
    T2CONbits.TMR2ON = 1;
    //PWM EN CP2

    CCP2CONbits.CCP2M = 0b1100; //PWM mode
    CCPR2L = 0x0f;          //inicio de ciclo de trabajo
    CCP2CONbits.DC2B0 = 0;
    CCP2CONbits.DC2B1 = 0;
    
    
    while (PIR1bits.TMR2IF == 0); // Esperamos un ciclo del TMR2
    PIR1bits.TMR2IF = 0;
    

    
    
    
    
    
    
    
                                                      }
  void PWM_duty (int chn, float duty)  {
      if (chn==0)
          CCPR1L = (ADRESH>>1)+128; 
      
      else
          CCPR2L = (ADRESH>>1)+128; 
      
      
  }                                                  
                                           