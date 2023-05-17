/*
 * File:   PWmanual.c
 * Author: Diego
 *
 * Created on 16 de abril de 2023, 12:41 PM
 */


#include <xc.h>
#include "PWmanual.h"
int i;
void PWmanual_init (char portsl, int numport){
    //TRISCbits.TRISC4 = numport;
    TRISCbits.TRISC3 = numport;
    //portsc = numport;
}
void PWmanual_func (float valad){
    if (PIR1bits.TMR2IF){
    //tmr0
        for (i=0;i<=valad;i++){
            if (i== valad){
            TRISCbits.TRISC3 ++;
        
            }
        }
       PIR1bits.TMR2IF = 0; 
    }
    
}