#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>

#include "i2c_master_noint.h"
#include "ssd1306.h"
#include "font.h"

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

void WriteUART1(const char * string);
void ReadUART1(char * message, int maxLength);
void UARTStartup();
void i2c_write(unsigned char addr, unsigned char reg, unsigned char val);
unsigned char i2c_read(unsigned char addr, unsigned char reg);
void blink();
void ssd1306_drawChar(unsigned char x, unsigned char y, unsigned char l);
void ssd1306_drawStr(unsigned char x, unsigned char y, char * message);

float coreTime = 0.0;

char msg[26];

int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    //TRIS = pinMode(input (1) / output (0))
    //LAT = digitalWrite(high (1) / low (0))
    //PORT = digitalRead
    //TRISBbits.TRISB4 = 1; //pinMode(B4, INPUT);
    TRISAbits.TRISA4 = 0;
    
    UARTStartup();
    
    i2c_master_setup();
    
    ssd1306_setup();
    
    __builtin_enable_interrupts();
    
//    char msg[15] = {'H','e','l','l','o',' ','W','o','r','l','d','!','\r','\n','\0'};

    while (1) {
        
//        unsigned char l = 'a';
//        ssd1306_drawChar(64,16,l);
        _CP0_SET_COUNT(0);
        sprintf(msg, "ABCDEFGHIJKLMNOPQRSTUVWXY");
        ssd1306_drawStr(0,0,msg);
        sprintf(msg, "abcdefghijklmnopqrstuvwxy");
        ssd1306_drawStr(0,8,msg);
        sprintf(msg, "0123456789!@#$%^&*()-=_+{");
        ssd1306_drawStr(0,16,msg);
        sprintf(msg, "FPS: %f", coreTime);
        ssd1306_drawStr(0,24,msg);
        //blink();
        ssd1306_update();
        coreTime = (float)24000000/(float)_CP0_GET_COUNT();
        
    }
}

// Write a character array using UART3
void WriteUART1(const char * string) {
  while (*string != '\0') {
    while (U1STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U1TXREG = *string;
    ++string;
  }
}

// Read from UART3
// block other functions until you get a '\r' or '\n'
// send the pointer to your char array and the number of elements in the array
void ReadUART1(char * message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) { // if data is available
      data = U1RXREG;      // read the data
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        ++num_bytes;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

void UARTStartup(){
    //TX = B3
    RPB3Rbits.RPB3R = 0b0001; // Set B3 to U1TX
    //RX = A2 
    U1RXRbits.U1RXR = 0b0000; // Set A2 to U1RX
    

    // turn on UART1 without an interrupt
    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((48000000 / 230400) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;

    // enable the uart
    U1MODEbits.ON = 1;
}

void i2c_write(unsigned char addr, unsigned char reg, unsigned char val){
    i2c_master_start();
    i2c_master_send(addr << 1);
    i2c_master_send(reg);
    i2c_master_send(val);
    i2c_master_stop();
}

unsigned char i2c_read(unsigned char addr, unsigned char reg){
    unsigned char val;
    i2c_master_start();
    i2c_master_send(addr << 1);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send((addr << 1) | 0b00000001);
    val = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return val;
}

void blink(){
    LATAbits.LATA4 = 1;
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT() < 120000){} //5ms delay
    LATAbits.LATA4 = 0;
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT() < 120000){} //5ms delay
}

void ssd1306_drawChar(unsigned char x, unsigned char y, unsigned char l) {
    
    if(x > 127){
        x = 127;
    }
    
    if(y > 31){
        y = 31;
    }
    
    if(l >= 0x20 && l <= 0x7f){
        l = l - 0x20; 
        int i;
        for(i = 0; i < 5; i++){
            int j;
            for(j = 0; j < 8; j++){
                ssd1306_drawPixel(x+i, y+j, 0b1 & (ASCII[l][i] >> j));
            }
        }
    }
    
}

void ssd1306_drawStr(unsigned char x, unsigned char y, char * message){
    
    if(x > 31){
        x = 31;
    }
    
    if(y > 127){
        y = 127;
    }
    
    int pos = 0;
    while(message[pos] != '\0'){
        
        ssd1306_drawChar(x + 5*pos, y, message[pos]); //used 6 instead of 5 so there's space between characters
        
        pos = pos + 1;
    }
    
}