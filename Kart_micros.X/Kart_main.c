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
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdint.h>
#include "PWM.h"
#include <pic16f887.h>
#include "USARTmodl.h"
#include "PWmanual.h"
#define _XTAL_FREQ 4000000
/*
 *Constantes
 */
int i;
float val;
/*
 *Variables
 */
unsigned int valadr,valpot;
/*
 * Prototipos de funciones
 */
void setup(void);
void preguntas (void);
unsigned char readEEPROM(unsigned char  address)
{
   
  EEADR = address; //Address to be read
  EECON1bits.EEPGD = 0;//Selecting EEPROM Data Memory
  EECON1bits.RD = 1; //Initialise read cycle
  return EEDATA; //Returning data
}

void writeEEPROM(unsigned char  address, unsigned char  dataEE)
{ 
  unsigned char INTCON_SAVE;//To save INTCON register value
  EEADR = address; //Address to write
  EEDATA = dataEE; //Data to write
  EECON1bits.EEPGD = 0; //Selecting EEPROM Data Memory
  EECON1bits.WREN = 1; //Enable writing of EEPROM
  INTCON_SAVE=INTCON;//Backup INCON interupt register
  INTCON=0; //Diables the interrupt
  EECON2=0x55; //Required sequence for write to internal EEPROM
  EECON2=0xAA; //Required sequence for write to internal EEPROM
  EECON1bits.WR = 1; //Initialise write cycle
  INTCON = INTCON_SAVE;//Enables Interrupt
  EECON1bits.WREN = 0; //To disable write
  while(PIR2bits.EEIF == 0)//Checking for complition of write operation
  {
    NOP(); //do nothing
  }
  PIR2bits.EEIF = 0; //Clearing EEIF bit
}


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
             valpot = ADRESH;
        }
      else if (ADCON0bits.CHS == 2)
            val = ADRESH;
            PIR1bits.ADIF =0;
       
    }
   if (INTCONbits.RBIF ){
       
        //INTCONbits.RBIF = 0;
        /*
        if (!PORTBbits.RB0){
            while (!RB0);
            SLEEP();
        }
         **/
        if (!PORTBbits.RB0){
            while (!RB0);
                PORTE ++;   
        
            
                         
        }
    if (!PORTBbits.RB1){
            while (!RB1){
                //valadr = 10;
                writeEEPROM(valadr, valpot);
                PORTD = readEEPROM(valadr);
                
                         }
        }
    
    }
   PWmanual_func (val);
}
//------------------------------------------Funcion para lectura del UART----------------
 char uart_read(){
 if(PIR1bits.RCIF== 0){
     if (RCSTAbits.OERR){
         RCSTAbits.CREN =0;
         NOP();
         RCSTAbits.CREN =1;
 }
     return RCREG;
 }
 else
     return 0;
 }
//-----------------------------------------------------------------------------------------------------------------------------------------


/*
 *---------------Main-------------
 */
void main (void)
{
    setup();
    OSCCON = 0x70;    // set internal oscillator to 8MHz
     PWM_init (0,0,255);
    ADCON0bits.GO =1;
    UART_Init(9600);  // initialize UART module with 9600 baud
    PWmanual_init (TRISCbits.TRISC4,0);
  __delay_ms(2000);  // wait 2 seconds
 
  UART_Print("1.Leer potenciometro\r\n");  // UART print
 
  __delay_ms(1000);  // wait 1 second
 
  UART_Print(message);  // UART print message
 
  __delay_ms(1000);  // wait 1 second
 
  UART_Print("\r\n");  // start new line
 ADCON0bits.GO =1;
 char text[9];
    i=0;
    
    while(1)
    {
      
       if (ADCON0bits.GO ==0){
           for (i=0;i<=2;i++){
           ADCON0bits.CHS = i;
           ADCON0bits.GO =1;
        
           }
   
            
       }
       
       //--------------------------------------------------------------------
        switch (uart_read()){
          case '1': 
             
              
               UART_Print ("\r\n");
            sprintf(text, "%03u\r\n", 10);
            UART_Print(text);
   
  
              preguntas();
             RCREG ='0';
             
             break;
           case '2': 
               __delay_us(9200000);
               UART_Print ("\r\n");
               UART_Print(uart_read());
               UART_Print ("\r\n");
               preguntas();
               RCREG ='0';
               
               break;
          
      }
      //Enviar datos al terminal
    if ( UART_Data_Ready() )  // if a character available
    {
      uint8_t c = UART_GetC();  // read from UART and store in 'c'
      UART_PutC(c);  // send 'c' via UART (return the received character back)
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
    TRISD = 0;
    OPTION_REGbits.nRBPU =  0;
    WPUB = 0b1111;
    PORTB = 0;
    PORTC = 0;
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
void preguntas(void)
{
    UART_Print ("1.Leer potenciometro\r\n");
    UART_Print (message);
}
