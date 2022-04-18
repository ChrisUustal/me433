#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

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
void initSPI();
unsigned char spi_io(unsigned char o);

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
    TRISBbits.TRISB4 = 1; //pinMode(B4, INPUT);
    
    //UARTStartup();
    initSPI();
    
    __builtin_enable_interrupts();
    
    unsigned char sine_arr[100] = {
        128, 136, 144, 152, 160, 168, 175, 183, 190, 197, 
        203, 210, 216, 221, 227, 232, 236, 240, 244, 247,
        250, 252, 254, 255, 255, 255, 255, 255, 254, 252,
        250, 247, 244, 240, 236, 232, 227, 221, 216, 210, 
        203, 197, 190, 183, 175, 168, 160, 152, 144, 136,
        128, 120, 112, 104, 96, 88, 81, 74, 66, 59, 
        53, 46, 40, 35, 29, 24, 20, 16, 12, 9, 
        6, 4, 2, 1, 0, 0, 0, 1, 2, 4,
        6, 9, 12, 16, 20, 24, 29, 35, 40, 46, 
        53, 59, 66, 74, 81, 88, 96, 104, 112, 120
    };
    
    unsigned char tri_arr[200] = {
        0,3,     5,8,     10,13,   15,18,   20,23,  
        26,26,   31,31,   36,36,   41,41,   46,46,
        51,51,   56,56,   61,61,   67,67,   72,72,  
        77,77,   82,82,   87,87,   92,92,   97,97,
        102,102, 108,108, 113,113, 118,118, 123,123, 
        128,128, 133,133, 138,138, 143,143, 148,148,
        154,154, 159,159, 164,164, 169,169, 174,174,
        179,179, 184,184, 189,189, 195,195, 200,200, 
        205,205, 210,210, 215,215, 220,220, 225,225, 
        230,230, 236,236, 241,241, 246,246, 251,251,
        251, 251, 246, 246, 241, 241, 236, 236, 230, 230,
        225, 225, 220, 220, 215, 215, 210, 210, 205, 205,
        200, 200, 195, 195, 189, 189, 184, 184, 179, 179,
        174, 174, 169, 169, 164, 164, 159, 159, 154, 154,
        148, 148, 143, 143, 138, 138, 133, 133, 128, 128,
        123, 123, 118, 118, 113, 113, 108, 108, 102, 102,
        97, 97, 92, 92, 87, 87, 82, 82, 77, 77, 
        72, 72, 67, 67, 61, 61, 56, 56, 51, 51, 
        46, 46, 41, 41, 36, 36, 31, 31, 26, 26, 
        20, 20, 15, 15, 10, 10, 5, 5, 0, 0
    };
    
    unsigned char pos1 = 0;
    unsigned char pos2 = 0;
    
    unsigned char c1 = 0b0;
    unsigned char c2 = 0b1;
    
    unsigned short p1 = (c1 << 15);
    unsigned short p2 = (c2 << 15);
    
    p1 = p1 | (0b111 << 12);
    p2 = p2 | (0b111 << 12);
    
    p1 = p1 | (sine_arr[pos1] << 4);
    p2 = p2 | (tri_arr[pos2] << 4);

    while (1) {
        
        if(pos1 < 99){
            pos1 = pos1 + 1;
        } else {
            pos1 = 0;
        }
        
        if(pos2 < 199){
            pos2 = pos2 + 1;
        } else {
            pos2 = 0;
        }
        
        p1 = p1 & 0xF000;
        p1 = p1 | (sine_arr[pos1] << 4);
        
        p2 = p2 & 0xF000;
        p2 = p2 | (tri_arr[pos2] << 4);
        
        //CS low
        LATAbits.LATA0 = 0;
        //Write upper byte
        spi_io(p1 >> 8);
        //Write lower byte
        spi_io(p1);
        //CS high
        LATAbits.LATA0 = 1;
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT() < 24000){} //1ms delay
        
         //CS low
        LATAbits.LATA0 = 0;
        //Write upper byte
        spi_io(p2 >> 8);
        //Write lower byte
        spi_io(p2);
        //CS high
        LATAbits.LATA0 = 1;
        
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT() < 96000){} //4ms delay

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
    //RX = B2
    U1RXRbits.U1RXR = 0b0100; // Set B2 to U1RX
    

    // turn on UART3 without an interrupt
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

// initialize SPI1
void initSPI() {
    // Pin B14 has to be SCK1
    // Turn of analog pins
    
    // Make an output pin for CS (A0)
    //RPA0Rbits.RPA0R = 0b0011;
    TRISAbits.TRISA0 = 0; 
    LATAbits.LATA0 = 1;
    // Set SDO1 (A1)
    RPA1Rbits.RPA1R = 0b0011;
    // Set SDI1
    SDI1Rbits.SDI1R = 0b0001;

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 10; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
}


// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}
