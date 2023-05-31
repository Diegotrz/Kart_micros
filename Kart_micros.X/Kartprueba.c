    /*
 * File:   postlab7.c
 * Author: Diego
 *
 * Created on 15 de abril de 2023, 10:58 PM
 */


// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdint.h>
#include "PWM.h"
#include "PWmanual.h"
#include <pic16f887.h>
#define _XTAL_FREQ 4000000
/*
 *Constantes
 */
int i;
float val;
/*
 *Variables
 */

/*
 * Prototipos de funciones
 */
void setup(void);
/*
 *Interrupción
 */
void __interrupt() isr (void)
{
   if(PIR1bits.ADIF){
        //Interrupción
       if (ADCON0bits.CHS ==1){
             PWM_duty(0,ADRESH);
              
      
        }
        else if (ADCON0bits.CHS ==0){
            PWM_duty(1 ,ADRESH);
             
        }
      else if (ADCON0bits.CHS == 2)
            val = ADRESH;
            PIR1bits.ADIF =0;
         
  
       
    }
     
   
    //Interrupción del TMR2
    PWmanual_func (val);
        
}
/*
 *---------------Main-------------
 */
void main (void)
{
    setup();
    PWM_init (0,0,255);
   PWmanual_init (TRISCbits.TRISC4,0);
    ADCON0bits.GO =1;
    i=0;
    while(1)
    {
      
       if (ADCON0bits.GO ==0){
           for (i=0;i<=2;i++){
           ADCON0bits.CHS = i;
           ADCON0bits.GO =1;
        
           }
   
            
       }
    }
}
    
    

/*
 * Funciones
 */
void setup(void){
    ANSEL = 0b00000011;
    ANSELH = 0;
    
    TRISA = 0xFF;
    TRISB = 0b11111111;
    OPTION_REGbits.nRBPU =  0;
    WPUB = 0b1111;
    PORTB = 0;
    //PORTC = 0;
    PORTD = 0;
    PORTE = 0;
   
   
    
    
    // Configuración del oscilador
    OSCCONbits.IRCF =   0b0111; //8MHz
    OSCCONbits.SCS = 1;
    
    // Configuración del ADC
    ADCON1bits.ADFM = 0; //Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    
    ADCON0bits.ADCS = 0b01; //FOSC/32
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON= 1;
    __delay_us(50);
    
    //Configuración de las interrupciones
    //Configuración para la interrupción del ADC      
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    //Configuración para la interrupción de los botones
    INTCONbits.RBIE = 0;
    INTCONbits.RBIF = 1;
    //Configuración para las interrupciones globales
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}