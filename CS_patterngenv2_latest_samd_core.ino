/*
control software for pattern generator v2 for shutter in C2 plane of TEM
can be used for compressive sensing, dose control, alternating vortex filter, 
and many other fun experiments

connect pin 2,34 together on board with a wire !!!!!


1.6.16 samd core is needed to fix a few bugs!!!
make sure you have it

 */

//TODO
//send flash buffer over USB to download new patterns (do this page by page in a writepagefromusb routine with some handshaking): DONE (but still ~10 min at 250kbaud)
//blank shutter in flyback time in lineISR when in line sync...do this in HW
//when starting sync jump to random address within a single sector of the flash (but alternatively allow to define exactly where you want to go)
//****try to use NMI_Handler for faster interrupt, possibly on pin2 not needing to connect to pin4 (not really needed, things work well down to 1us now): advise from the pro's: never ever use NMI as it creates many weird downsides in the bootloader...so DONT DO IT
//rollover of sector memory after reading a lot of images is not correct at the moment, e.g. pattern 1 ends up on patter 0 after many frames-FIX THIS


# define SPIPIN11 //to use SPI on pin 11,12,13

//use updated ArduinoSamd_core 1.6.16 ! for corrected (and faster) version of interrupt handling + corrected 48 Mhz
#include <pins_arduino.h>



#include <SPI.h>
#include <SerialFlash.h> //note updated version that does not init the spi when starting!
//special setting to put SPI on pins 11-13 (there should be a cleaner way inside the samd core to get this without hacking...invesitigate)
#include "wiring_private.h" // pinPeripheral() function
SPIClass mySPI (&sercom1, 12, 13, 11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3); //to change to sercom1 on pins 11,12,13 needed as the arduino zero pro has the spi standard on the spi connector via sercom4




  
#define SPICONFIG   SPISettings(12000000, MSBFIRST, SPI_MODE1) //this is max clock frequency for SPI according to issue #147


inline void digitalWriteDirect(int PIN, boolean val){
  if(val)  PORT->Group[g_APinDescription[PIN].ulPort].OUTSET.reg = (1ul << g_APinDescription[PIN].ulPin);
  else     PORT->Group[g_APinDescription[PIN].ulPort].OUTCLR.reg = (1ul << g_APinDescription[PIN].ulPin);
}

//pin mapping
#define DATAOUT 11//MOSI
#define DATAIN  12//MISO 
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss
int COMP=35; //compensation for nr of clockcycles it takes for the timer to retrigger on external interupt, 3-4-2017 tested on scope COMP=7 is perfect match

const int FlashChipSelect = SLAVESELECT; // digital pin for flash chip CS pin
#define EXT 8// switch between arduino control and external control via external clock signal
#define PIX 9//pixel clock generated
#define IRQ 4//line sync interupt on pin 2 CAREFUL for Arduino M0 Pro pin 4 and 2 are swapped!!!!!
#define IRQBIS 3//and another copy of the line sync to create a proper ISR indendendent of the HW ISR needed for starting and stopping the time
///connect pin 2,3,4 together for interrupt reasons

//RGB led on pin 5,6,7 driven by inverter so with inverted logic
#define RED 7
#define YELLOW 6
#define GREEN 5

#define CLKOUT 1 //output of reference clock for testing only pin 1 is PA10 (so even on the mux)

const int PSIZE=256; //flash page size
unsigned long segsize=0xFFFF; //16Mbyte segments / page length  (total chip is 64Mbyte)
int pattern=3; //which pattern is selected

//synchronisation is on falling edge, if wanted otherwise change jumper on board for hardware inversion

bool line=true; //line sync (true) or pixel sync (false)
bool stopped=false; //pixel clock generation stopped when line sync is high

//Definitions of timers and delays
//one tick is 1/48Mhz=20.83ns
//24 ticks =1us
//int PIXTIME=24; //defines the dwell time for ~1us (should work fine when PIXTIME >COMP, otherwise the first pixel might get lost)
//int PIXTIME=479; //defines the dwell time for 20us 
int PIXTIME=239; //defines the dwell time for 10us 
//int PIXTIME=120; //defines the dwell time for 5us 
//int PIXTIME=23; //defines the dwell time for 1us 
#define NOP __asm__ __volatile__ ("nop\n\t")    //No operating process (1 clock cycle doing nothing)
 
//// the setup function runs once when you press reset or power the board
void setup() {
  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));
 // randSeed();      //Intialization of the Mozzi random seed using the T sensor of the Arduino (see Mozzi help)
  
  //setup the serial link over USB
  //Serial.begin(115200);
  Serial.begin(2000000);
  
  Serial.write("Pattern Generator V2 ready for action!\n");
  Serial.write("O opens the shutter\n"); 
  Serial.write("C closes the shutter\n");
  Serial.write("S start patterned shuttering\n");

  Serial.write("P choose pattern\n");
  Serial.write("L sync setup\n");
  Serial.write("T set dwell time\n");
  Serial.write("M set COMP time\n");
  Serial.write("F start fractional dose STEM (synced PWM STEM)\n");
  Serial.write("D enter fraction dose TEM mode (free-running PWM TEM)\n");
  
  Serial.write("W to write new random pattern\n"); 
  Serial.write("R to dump random pattern on USB\n"); 
  Serial.write("(O, C, W, T, or R switch back to internal sync)!\n");
  
   // initialize the pins
  pinMode(DATAOUT, OUTPUT);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(EXT, OUTPUT);
  pinMode(PIX, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  
  pinMode(IRQ,INPUT);
  pinMode(IRQBIS,INPUT);
  pinMode(2,INPUT); //to be sure
  pinMode(DATAIN,INPUT);
       
  // init state of output pins
  digitalWrite(EXT,HIGH); //listen to Arduino 
  digitalWrite(YELLOW,HIGH); //all leds off
  digitalWrite(RED,HIGH); //all leds off
  digitalWrite(GREEN,HIGH); //all leds off
  digitalWrite(PIX,HIGH);  

  //greet the world with some blinking of leds      
  ledtest();

  //init the SPI 
  mySPI.beginTransaction(SPICONFIG);  
  mySPI.begin(); //has to come AFTER beginstransaction and BEFORE pinperipheral!!!! very important
  // Assign pins 11, 12, 13 to SERCOM functionality
  pinPeripheral(12, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  SerialFlash.begin(mySPI,FlashChipSelect); //changed the serialflash library to accept my SPI without initiation a SPI.begin on it (this would erase the pin assigments above)

  getinfo(); //test SPI by dumping the info on the flash memory chip
  // set up GCLK4 clock distribution
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Take full 48Mhz speed
                    GCLK_GENDIV_ID(1);            // 
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  REG_GCLK_GENCTRL = GCLK_GENCTRL_OE |  
                     GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4) ;        // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

pinMode(CLKOUT,OUTPUT);
/*
  //feed GCLK4 to output pin for measurements ONLY do this when debugging, otherwise generates heat and RF for no reason
  
  // Enable the port multiplexer on digital pin 6
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  // Switch the port multiplexer to peripheral H (GCLK_IO[4])
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXE_H;
*/
 
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC1 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC1 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  //and to evsys ch0 (this is essential to make it work!)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to EVSYS
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_EVSYS_0;
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  //and to evsys ch1 (this is essential to make it work!)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to EVSYS
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_EVSYS_1;
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

 //creates more jitter, depends probably on quality of input signal
 // does not really speed up the 6us response on ext interrupt
 //and to EIC (interupt controller)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to EiC
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_EIC;
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
}

//
// the loop function runs over and over again forever
void loop() {
   
   while (Serial.available() > 0) {
     char mychar=Serial.read();
      if (mychar == 'W') {
        digitalWrite(EXT,HIGH);//SPI control
        Serial.write("W\n");  
        Serial.write("Write mode, this will erase all data in pattern (takes ~15 min)\n");
        Serial.write("are you REALLY sure? (Y/N)\n");  
        while (Serial.available() == 0) {
          //wait
        }
        char mychar=Serial.read();
        if (mychar == 'Y') {
          Serial.write("Y\nEntering write mode\n");  
          writesegment();
          }
       Serial.write("Write mode abandoned, nothing erased.\n");      
      }
      if (mychar == 'R') {
        digitalWrite(EXT,HIGH);//SPI control
        Serial.write("R\n");
        dumppattern();
      }
      if (mychar == 'S') {
        Serial.write("S");
        //send SPI commands to flash to prepare for full readout starting from lowest adress
        //the readout is performed from then on with the external clock
        digitalWrite(YELLOW,LOW);
        digitalWrite(RED,HIGH);
        digitalWrite(GREEN,HIGH);
        prepareext();
      }
      if (mychar == 'O') {
        Serial.write("O");
        //put the shutter in unblanked mode
        openshutter();
        Serial.write("\nShutter open\n");
        digitalWrite(YELLOW,HIGH);
        digitalWrite(RED,HIGH);
        digitalWrite(GREEN,LOW);
      }
      if (mychar == 'C') {
        Serial.write("C");
        //put the shutter in blanked mode
        closeshutter();
        Serial.write("\nShutter closed\n");
        digitalWrite(YELLOW,HIGH);
        digitalWrite(RED,LOW);
        digitalWrite(GREEN,HIGH);
      }
       if (mychar == 'T') {
        //change dwell time
        int value=0;
        char junk = ' ';
        Serial.write("T\n Enter dwell time in nr of 48Mhz clockcycles [239=10us,24=1us], end with ENTER\n");
        while (Serial.available() == 0) ;  // Wait here until input buffer has a character
        {
          value = Serial.parseFloat();        // new command in 1.0 forward
          Serial.print("Dwell time clock cycles = "); Serial.println(value, DEC);
          while (Serial.available() > 0)  // .parseFloat() can leave non-numeric characters
            { junk = Serial.read() ; }      // clear the keyboard buffer
        }
        PIXTIME=value;
        openshutter();
      }

      if (mychar == 'M') {
        //change COMP time
        int value=0;
        char junk = ' ';
        Serial.write("M\n Enter COMP time in nr of 48Mhz clockcycles [7 is no shift, 35 works well for 10us 2K], end with ENTER\n");
        while (Serial.available() == 0) ;  // Wait here until input buffer has a character
        {
          value = Serial.parseFloat();        // new command in 1.0 forward
          Serial.print("Dwell time clock cycles = "); Serial.println(value, DEC);
          while (Serial.available() > 0)  // .parseFloat() can leave non-numeric characters
            { junk = Serial.read() ; }      // clear the keyboard buffer
        }
        COMP=value;
        openshutter();
      }
      
      if (mychar == 'F') {
        //change dwell time
        int value=0;
        char junk = ' ';
        Serial.write("F\n Enter fraction of dwell time that beam is on, end with ENTER\n");
        while (Serial.available() == 0) ;  // Wait here until input buffer has a character
        {
          value = Serial.parseFloat();        // new command in 1.0 forward
          Serial.print("Fraction = "); Serial.println(value, DEC);
          while (Serial.available() > 0)  // .parseFloat() can leave non-numeric characters
            { junk = Serial.read() ; }      // clear the keyboard buffer
        }
        startstempwm(value); //do it
      }

      
      if (mychar == 'P') {
        Serial.write("P\n");
        Serial.write("Select pattern 0,1,2,3 or C for checkerboard\n");  
        while (Serial.available() == 0) {
          //wait
        }
        char mychar=Serial.read();
        if (mychar == '0') {
          Serial.write("0\nPattern 0 selected\n");  
          pattern=0;
          }
        if (mychar == '1') {
         Serial.write("1\nPattern 1 selected\n");  
          pattern=1;
          }
        if (mychar == '2') {
          Serial.write("2\nPattern 2 selected\n");  
          pattern=2;
          }
        if (mychar == '3') {
          Serial.write("3\nPattern 3 selected\n");  
          pattern=3;
          }
        if (mychar == 'C') {
          Serial.write("C\nCheckerboard selected\n");  
          pattern=-1;
          }
      }
      if (mychar == 'L') {
        Serial.write("L\nLine or pixel sync (L/P)");
        while (Serial.available() == 0); //wait
        mychar=Serial.read();
        if (mychar == 'L') {
          //Line sync
          Serial.write("L\nLine sync\n");  
          line=true;
          }
        if (mychar == 'P') {
          //Pixel sync
          Serial.write("P\nPixel sync\n");  
          line=false;
          }
      }    
      if (mychar == 'D') {
        int value=0;
        char junk = ' ';
        Serial.write("D\nEnter value 0-65532 (freq~100Hz) to control 16 bit PWM, end with ENTER\n");
        while (Serial.available() == 0) ;  // Wait here until input buffer has a character
        {
          value = Serial.parseFloat();        // new command in 1.0 forward
          Serial.print("a = "); Serial.println(value, DEC);
          while (Serial.available() > 0)  // .parseFloat() can leave non-numeric characters
            { junk = Serial.read() ; }      // clear the keyboard buffer
        }
        startpwm(value);
      }
     
   }
      
}
void send4byte(long address){
   //32 bit address, assume device is selected and ready to receive the address
  mySPI.transfer((char)(address>>24));   //send MSByte address first
  mySPI.transfer((char)(address>>16));   //send MSByte address first
  mySPI.transfer((char)(address>>8));   //sintermediate byte address first
  mySPI.transfer((char)(address));      //send LSByte address
  }
 
void startstempwm(int fraction){
  closeshutter(); //prepares the output to link to DATAIN pin as OUTPUT
  
  noInterrupts(); // disable all interrupts
  //use event channel to retrigger the TCC connected to EXTINT
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;     // Switch on the event system peripheral
  
  // connect DATAIN pin to TCC0 output via port mux
  PORT->Group[g_APinDescription[DATAIN].ulPort].PINCFG[g_APinDescription[DATAIN].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[DATAIN].ulPort].PMUX[g_APinDescription[DATAIN].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
  
  attachInterrupt(IRQ, dummyISR, HIGH);         //feed IRQ to EVsys and figure out rising or falling edge inside the ISR 
  attachInterrupt(IRQBIS, lineISRpwm, RISING);  //when line goes high, reload the timer with new value
  // normal PWM operation
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Don't Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_NPWM;      // Setup normal PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);             // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the period of the pulse
  REG_TCC0_PER = PIXTIME*2+1;      // Set the frequency of the PWM on TCC0 to 2xpixtime+1
  while(TCC0->SYNCBUSY.bit.PER);

  int offtime=round((double(2.0*PIXTIME+1.0)*(100.0-double(fraction)))/100.0);
  // The CCx register value corresponds to the pulsewidth, TCC0 outputs on WO[3] for PA19 (=pin 12)
  REG_TCC0_CC3 = offtime;       // variable duty cycle
  while(TCC0->SYNCBUSY.bit.CC3);

//  REG_TCC0_DRVCTRL |= TCC_DRVCTRL_NRV1 |TCC_DRVCTRL_NRE1 ; //NRV1 determines that in the stop state the output is high, this leads to one pixel too much each line and a partial screwing up of the first pixel

  REG_TCC0_DRVCTRL |= TCC_DRVCTRL_NRE1 ; //not having NRV1 determines that in the stop state the output is low                                                                              
  
  //Event ch0 on interupt change  
  REG_EVSYS_USER =    EVSYS_USER_CHANNEL(1) |                               // Attach TCC0_EV0 (receiver) to channel 0 (n + 1)
                      EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0) ;               
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_FALLING_EDGE|
                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_14) |  // Set event generator (sender) as external interrupt 14 (which connects to pin 2 = pa14
                      EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0

  REG_EVSYS_USER =    EVSYS_USER_CHANNEL(2) |                               // Attach TCC0_EV0 (receiver) to channel 0 (n + 1)
                      EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_1) ;               
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_RISING_EDGE|
                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_14) |  // Set event generator (sender) as external interrupt 14 (which connects to pin 2 = pa14
                      EVSYS_CHANNEL_CHANNEL(1);                          // Attach the generator (sender) to channel 0

  REG_TCC0_EVCTRL |= TCC_EVCTRL_TCEI0 |   TCC_EVCTRL_TCEI1|             // Enable the TCC0 event input EV0
                     TCC_EVCTRL_EVACT0_START|    //start on falling edge
                     TCC_EVCTRL_EVACT1_STOP;     //stop of rising edge
                     //TCC_EVCTRL_TCINV0;
  while (TCC0->SYNCBUSY.bit.ENABLE);                  // Wait for synchronization 
 
  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO14;      // Enable event output on external interrupt 14=pin2 pa14
  
  //enable the counter, note that in event mode as here, the timer only really starts when start event is received
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization 
  interrupts(); //let's go 
  }
 
void startpwm(int value){
  closeshutter(); //prepares the output to link to DATAIN pin as OUTPUT
  // Output 100Hz PWM on timer TCC0 digital pin D11
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for digital pin 12 (D12): timer TCC0 output
  PORT->Group[g_APinDescription[DATAIN].ulPort].PINCFG[g_APinDescription[DATAIN].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[DATAIN].ulPort].PMUX[g_APinDescription[DATAIN].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
  
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
  REG_TCC0_PER = 65532;      // Set the frequency of the PWM on TCC0 to 100Hz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB3 = value;       // TCC0 CCB3 - center the servo on D12
  while(TCC0->SYNCBUSY.bit.CCB3);

  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    // Divide GCLK4 by 2
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization 
  }
  
void prepareext(){
  if (pattern==-1){
    Serial.write("Checkerboard not implemented yet\n");
    //need to make something with interupt and toggle of dataout pin
    return;
    }
  unsigned long address=getstartaddress();
  unselect();
  delay(1);
  select();
  mySPI.transfer((char)0x03); //transmit read opcode
  send4byte(address);
  //now we are in read mode, the external clock will take over to read out everything at its own pace, address is automatically rolled over
  digitalWrite(EXT,LOW);//hand over to external control, PIX now controls the readout of the flash 
  //closeshutter();
  stopsync();
  //set up the PIX generator
   if (line==true){
    //start generating PIX pulse from linesync
    setuplinesync();
  }
  else{
    //or create PIX from pixel sync
    setuppixelsync();
  }
  
  }

void   setuppixelsync(){
  Serial.write("Pixel sync through software (alternative is jumper, then sync is hardwired- much faster for now)\n");  
  noInterrupts(); // disable all interrupts
  detachInterrupt(IRQ);
  detachInterrupt(IRQBIS);
  attachInterrupt(IRQ, pixelint, FALLING);
  digitalWrite(PIX,HIGH);
  interrupts(); //lets go
  }

void pixelint(){
  //but this is  slow up to 2us
  //interupt handler when irq is pixel sync
  //create short sck pulse, enough to advance the clock on tick
  //noInterrupts(); //I would have thought this should have been taken care of in the calling of the interrupt?
  //digitalWrite(PIX,LOW);
  //digitalWrite(PIX,HIGH);
  digitalWriteDirect(PIX, LOW); //fast version of digitalwrite, takes about 250ns
  digitalWriteDirect(PIX, HIGH); 
  //interrupts();
}

void lineISRpwm(){
        noInterrupts(); //I would have thought this should have been taken care of in the calling of the interrupt?         
        REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_STOP; //stop the timer, this puts output in the state defined by nonrcovfault in REG_TCC1_DRVCTRL
        while(TCC0->SYNCBUSY.bit.CTRLB);     
        REG_TCC0_COUNT = COMP;      // clear counter so we are ready when we receive a RETRIGGER/START event, comp compensates for nr of clockcycles it takes for the event system
        while(TCC0->SYNCBUSY.bit.COUNT);
        interrupts(); 
        //digitalWrite(CLKOUT,HIGH); //say hello to outside world
        //digitalWrite(CLKOUT,LOW); //say hello to outside world  
}
void lineISR(){
        noInterrupts(); //I would have thought this should have been taken care of in the calling of the interrupt?         
        REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_STOP; //stop the timer, this puts output in the state defined by nonrcovfault in REG_TCC1_DRVCTRL
        while(TCC1->SYNCBUSY.bit.CTRLB);     
        
        //it takes up to ~2us before the timer is stopped...(I improved Winterrupts.c in the samdcore but there should be more room for improvement)
        //but this is not a big deal as there will be no extra falling edges
        //either the counter was below PIXTIME and the output was low...then it will come high after this delay
        //when counter was >PIXTIME the output was high and it will not change when driving it high again: not causing an unwanted trigger on falling edge of PIX signal
        
        //put shutter in blanked state for the flyback time
        // TODO, but take care not to break the ongoing read sequence of the SPI flash
        //and not to drive into a conflict on who owns the SPI bus (flash wants to write data)
        //alternative is to make sure the last pixel in a row is always a zero?
        //
  
        //best option seems to be to connect the HOLD of the Flash to the line sync
        //...but it doesn't work, doesn't seem to go in high z state
  
        //need to make a HW solution for speed.
        
        REG_TCC1_COUNT = COMP;      // clear counter so we are ready when we receive a RETRIGGER/START event, comp compensates for nr of clockcycles it takes for the event system
        //to retrigger the counter
        while(TCC1->SYNCBUSY.bit.COUNT);
        interrupts();   
       // digitalWrite(CLKOUT,HIGH); //say hello to outside world
       // digitalWrite(CLKOUT,LOW); //say hello to outside world
}

void dummyISR(){
  //do nothing, needed to have one ISR for the EVSys (pin4) and one for an actual ISR that triggers on rising edge and stops the counter 
  }

void setuplinesync(){
  noInterrupts(); // disable all interrupts
  //use event channel to retrigger the TCC connected to EXTINT
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;     // Switch on the event system peripheral
  
  // connect PIX pin to TCC1 output via port mux
  PORT->Group[g_APinDescription[PIX].ulPort].PINCFG[g_APinDescription[PIX].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[PIX].ulPort].PMUX[g_APinDescription[PIX].ulPin >> 1].reg = PORT_PMUX_PMUXO_E;

  attachInterrupt(IRQ, dummyISR, HIGH);          //feed IRQ to EVsys and figure out rising or falling edge inside the ISR 
  attachInterrupt(IRQBIS, lineISR, RISING); //when line goes high, reload the timer with new value
  // normal PWM operation
  REG_TCC1_WAVE |= TCC_WAVE_POL(0xF) |         // Don't Reverse the output polarity on all TCC1 outputs
                    TCC_WAVE_WAVEGEN_NPWM;    // Setup normal PWM on TCC1
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the period of the pulse
  REG_TCC1_PER = PIXTIME*2+1;      // Set the frequency of the PWM on TCC1 to 2xpixtime+1
  while(TCC1->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth
  REG_TCC1_CC1 = PIXTIME;       // 50% duty cycle
  while(TCC1->SYNCBUSY.bit.CC1);

//  REG_TCC1_DRVCTRL |= TCC_DRVCTRL_NRV1 |TCC_DRVCTRL_NRE1 ; //NRV1 determines that in the stop state the output is high, this leads to one pixel too much each line and a partial screwing up of the first pixel


  REG_TCC1_DRVCTRL |=  TCC_DRVCTRL_NRE1 ; //not having NRV1 determines that in the stop state the output is low
                                          //this makes that the N+1 pixel is clocked out at the end of the line sync
                                          //this means no delay at beginning of line (first pix is good)
                                          //and tolerates up to max 1/2 pixel time error at the end of a line before the correct syncing is lost
                                          //for the very first pixel, requires the it is clocked out in the setup of linesync 

   //create 1 clock to prepare the first pixel for when the line sync goes low to start a sequence of pixels
   digitalWriteDirect(PIX, HIGH); //fast version of digitalwrite, takes about 250ns
   digitalWriteDirect(PIX, LOW); 
                                          
  
  //Event ch0 on interupt change  
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                               // Attach TCC1_EV0 (receiver) to channel 0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_0) ;               
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_FALLING_EDGE|
                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_14) |  // Set event generator (sender) as external interrupt 14 (which connects to pin 2 = pa14
                      EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0

  REG_EVSYS_USER = EVSYS_USER_CHANNEL(2) |                               // Attach TCC1_EV0 (receiver) to channel 0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1) ;               
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_RISING_EDGE|
                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_14) |  // Set event generator (sender) as external interrupt 14 (which connects to pin 2 = pa14
                      EVSYS_CHANNEL_CHANNEL(1);                          // Attach the generator (sender) to channel 0

  REG_TCC1_EVCTRL |= TCC_EVCTRL_TCEI0 |   TCC_EVCTRL_TCEI1|             // Enable the TCC1 event input EV0
                    TCC_EVCTRL_EVACT0_START|    //start on falling edge
                    TCC_EVCTRL_EVACT1_STOP;     //stop of rising edge
                    //TCC_EVCTRL_TCINV0;
  while (TCC1->SYNCBUSY.bit.ENABLE);                  // Wait for synchronization 
 
  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO14;      // Enable event output on external interrupt 14=pin2 pa14
  
  
  //enable the counter, note that in event mode as here, the timer only really starts when start event is received
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;             // Enable the TCC1 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization 
  interrupts(); //let's go
  }
  
void stopsync(){
  REG_PM_APBCMASK &= ~PM_APBCMASK_EVSYS;     // Switch off the event system peripheral
  detachInterrupt(IRQ);
  detachInterrupt(IRQBIS);
  //stop an disable the timers if needed
  REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_RETRIGGER; //retrigger just in case it was already stopped by an event, otherwise the next stop doesn't do anytinh 
  while(TCC1->SYNCBUSY.bit.CTRLB);
  REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_STOP; //stop the timer, this puts output in the state defined by nonrcovfault in REG_TCC1_DRVCTRL
  while(TCC1->SYNCBUSY.bit.CTRLB);
  TCC1->CTRLA.reg &= ~TCC_CTRLA_ENABLE; //counter off
  while(TCC1->SYNCBUSY.bit.ENABLE);
  REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_STOP; //stop the timer, this puts output in the state defined by nonrcovfault in REG_TCC0_DRVCTRL
  while(TCC0->SYNCBUSY.bit.CTRLB)
  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE; //counter off
  while(TCC0->SYNCBUSY.bit.ENABLE);
  digitalWrite(PIX,LOW); //when the port returns to the normal output function, make sure the shutter is closed
  PORT->Group[g_APinDescription[PIX].ulPort].PINCFG[g_APinDescription[PIX].ulPin].bit.PMUXEN = 0;//make sure we take control back from PWM controller to normal output pin
  REG_EIC_EVCTRL &=~EIC_EVCTRL_EXTINTEO14;
  }

void select(){
  pinMode(DATAIN,INPUT); 
  digitalWrite(EXT,HIGH); //make sure the board is listening to arduino and not to external pulses
  digitalWrite(SLAVESELECT,LOW); //select the flash memory 
}
void unselect(){
 pinMode(DATAIN,INPUT); //seems to be needed to be able to get the DATAIN pin to output later in this function...probably due to link with sercom...black magic
 digitalWrite(SLAVESELECT,HIGH); //unselect the flash memory 
 delay(1);
 pinMode(DATAIN,OUTPUT); 
 digitalWrite(DATAIN,LOW); 
 digitalWrite(EXT,LOW); //allow DATAIN to reach output
}

void openshutter(){
  stopsync();
  unselect();
  digitalWrite(DATAIN,LOW); //now datain is an output as the flash is in high z mode
  }
void closeshutter(){
  stopsync();
  unselect();
  digitalWrite(DATAIN,HIGH); //now datain is an output as the flash is in high z mode
  }

void ledtest(){
  //Serial.write("Led test\n");
 const int Ntest=10;
  const int deltime=30;
  for (int I=0;I<Ntest;I++)
  {
     digitalWrite(YELLOW,LOW);
     delay(deltime);
     digitalWrite(YELLOW,HIGH);
     delay(deltime);
     digitalWrite(RED,LOW);
     delay(deltime);
     digitalWrite(RED,HIGH);
     delay(deltime);
     digitalWrite(GREEN,LOW);
     delay(deltime);
     digitalWrite(GREEN,HIGH);
     delay(deltime);
  }
}

unsigned long getstartaddress(){
  if (pattern==0){
    return 0x0000000;//first segment, flash does rollover in each segment
    }
  if (pattern==1){
    return 0x1000000;
    }
  if (pattern==2){
    return 0x2000000;
    }   
  if (pattern==3){
    return 0x3000000;
    }    
  }


void dumppattern(){ 
  unsigned char buf[PSIZE];
  char tbs[256];
  unsigned long address=0;
  if (pattern==-1){
    //checkerboard
    Serial.write("Checkerboard selected, nothing to dump\n");  
    return;
    }

  Serial.write("Dumping memory by page, press any key to continue to next page, X to stop\n");   
  address=getstartaddress();  
  unsigned long startaddress=address;  
  bool go_on=true;
  while (go_on){
    SerialFlash.read(address, buf, PSIZE);
    //dump on the screen
    sprintf(tbs, "%032X:\n", address);
    Serial.write(tbs);
    Serial.write(buf,PSIZE);
    Serial.print("\n");  
    while (Serial.available() == 0) {
          //wait
    }
    char mychar=Serial.read();
    if (mychar == 'X') {
          go_on=false;
    }
    address=address+PSIZE;
    if ((address-startaddress)>=segsize*PSIZE){  //should this not be segsize*PSIZE? was just segsize before
          go_on=false;
    }
  }
  
  
//  for (int I=0;I<PSIZE;I++){
//    
//    //the normal serial.print(v,BIN) leaves the leading zeros off
//    byte data=buf[I];
//    for (int j=0;j<8;j++){
//        if (data>127){
//          Serial.print("1");
//          }
//          else
//          {
//            Serial.print("0");
//            }
//        data=data<<1;
//        
//      }
//  }
//  Serial.write("\n"); 
}    

void getinfo(){
  unsigned char buf[256], sig[256], buf2[8];
  unsigned long address, count, chipsize, blocksize;
  unsigned long usec;
  bool first;

  // Read the chip identification
  Serial.println();
  Serial.println("Read Chip Identification:");
  SerialFlash.readID(buf);
  Serial.print("  JEDEC ID:     ");
  Serial.print(buf[0], HEX);
  Serial.print(" ");
  Serial.print(buf[1], HEX);
  Serial.print(" ");
  Serial.println(buf[2], HEX);
  Serial.print("  Part Nummber: ");
  Serial.println(id2chip(buf));
  Serial.print("  Memory Size:  ");
  chipsize = SerialFlash.capacity(buf);
  Serial.print(chipsize);
  Serial.println(" bytes");
  if (chipsize == 0) return;
  Serial.print("  Block Size:   ");
  blocksize = SerialFlash.blockSize();
  Serial.print(blocksize);
  Serial.println(" bytes");
  }

void writesegment(){
  unsigned char buf[PSIZE];
  unsigned long address, count, chipsize;
  const unsigned long blocksize=SerialFlash.blockSize();
  //const int length=1;
  while (SerialFlash.ready() == false) {
        NOP;
  }
    
  //need to erase before writing
  Serial.write("Erasing pattern (takes 2 min) \n");
  address=getstartaddress();
  bool state=false;
  unsigned long startaddress=address;  
  for (int j=0;j<256;j++){
  //for (int j=0;j<20;j++){
    SerialFlash.eraseBlock(address);
    address=address+blocksize;
    digitalWrite(RED,state); //toggle red led for status indication
    state=!state;
    while (SerialFlash.ready() == false) {
    }
    Serial.write("Block erased\n");
  }  
  
  Serial.write("Pattern erased\n");
  //now write page by page from what we receive over the serial port
  address=getstartaddress(); 
  unsigned long patternsize=0xFFFFFF;
  Serial.write("Waiting for page data\n");
  while ((address-startaddress)<patternsize){ //segment size   
    for (int j=0;j<PSIZE;j++){
      while (Serial.available() == 0) {
      }
      buf[j]=Serial.read(); //fill page buffer
    }

    //write page
    SerialFlash.write(address, buf, PSIZE);
    digitalWrite(RED,state); //toggle red led for status indication
    state=!state;
    while (SerialFlash.ready() == false) {
      }
    Serial.write("Page written\n");
    address=address+PSIZE;
  }
  digitalWrite(RED,LOW); 
  Serial.println("\nPattern written\n");
  }
void writepattern(){
  unsigned char buf[PSIZE];
  char tbs[260];
  unsigned long address, count, chipsize;
  const unsigned long blocksize=SerialFlash.blockSize();
  //const int length=1;
 
  //need to erase before writing
  Serial.write("Erasing (will take about 8 minutes-red led flashing slowly):\n");
  SerialFlash.eraseAll();
  int progress=0;
  while (SerialFlash.ready() == false){
    delay(100);
    digitalWrite(RED,LOW);
    delay(100);
    digitalWrite(RED,HIGH);
    progress++;
    if (progress==100){
      Serial.write(".");//make a reasonable amount of dots for 8 minutes
      progress=0;
    }
  }
  Serial.println("\nErase completed\n");

  address=0;
  int th=0;
  Serial.write("Writing (will take even longer (red led flashing fast))\n");
  
  for (int segment=0;segment<4;segment++){
    sprintf(tbs, "Writing segment %d\n", segment);
    Serial.write(tbs);
    switch (segment){
      case 0:
        th=10;
        break;
      case 1:
        th=25;
        break;
      case 2:
        th=50;
        break;
      case 3:
        th=-1; //checkerboard pattern (in fact a waste of memory space,but at least it is uniform)
      }
    progress=0;
    for (unsigned long segaddress=0; segaddress<segsize;segaddress++){
      preparepage(buf,th); //fill buffer with required pattern   
     
      SerialFlash.write(address, buf, PSIZE);
      progress++;
      if (progress==500){
        Serial.write(".");//make a reasonable amount of dots while writing
        progress=0;
      }
      address=address+PSIZE;
      digitalWrite(RED,!digitalRead(RED)); //toggle led
      while (SerialFlash.ready() == false) {
        delay(1);
      }
    }
    Serial.write("\n");
  }
  Serial.write("Done writing\n");
  digitalWrite(RED,HIGH);
}    

void preparepage(unsigned char* buf,int th){
  //make sure buf is initialised at least for PSIZE bytes
  for (int I=0;I<PSIZE;I++)
  { 
    //construct a byte made of bits with a certain chance of being 1 given by threshold
    byte val=0;
    if (th<0){
        //do checkerboard pattern
        val=0xAA;
        }
    else{
    for (int j=0;j<8;j++){
      val=val<<1;//shift to left to next bit      
      byte r=random(100);        //Mozzi based pseudo-random generator or hardware random gen from Zero
      if (r>th){
        val=val+1;
        }  
      }   
    }
    buf[I]=(unsigned char)val;
  }
}

const char * id2chip(const unsigned char *id)
{
  if (id[0] == 0xEF) {
    // Winbond
    if (id[1] == 0x40) {
      if (id[2] == 0x14) return "W25Q80BV";
      if (id[2] == 0x15) return "W25Q16DV";
      if (id[2] == 0x17) return "W25Q64FV";
      if (id[2] == 0x18) return "W25Q128FV";
      if (id[2] == 0x19) return "W25Q256FV";
    }
  }
  if (id[0] == 0x01) {
    // Spansion
    if (id[1] == 0x02) {
      if (id[2] == 0x16) return "S25FL064A";
      if (id[2] == 0x19) return "S25FL256S";
      if (id[2] == 0x20) return "S25FL512S";
    }
    if (id[1] == 0x20) {
      if (id[2] == 0x18) return "S25FL127S";
    }
  }
  if (id[0] == 0xC2) {
    // Macronix
    if (id[1] == 0x20) {
      if (id[2] == 0x18) return "MX25L12805D";
    }
  }
  if (id[0] == 0x20) {
    // Micron
    if (id[1] == 0xBA) {
      if (id[2] == 0x20) return "N25Q512A";
      if (id[2] == 0x21) return "N25Q00AA";
    }
    if (id[1] == 0xBB) {
      if (id[2] == 0x22) return "MT25QL02GC";
    }
  }
  if (id[0] == 0xBF) {
    // SST
    if (id[1] == 0x25) {
      if (id[2] == 0x02) return "SST25WF010";
      if (id[2] == 0x03) return "SST25WF020";
      if (id[2] == 0x04) return "SST25WF040";
      if (id[2] == 0x41) return "SST25VF016B";
      if (id[2] == 0x4A) return "SST25VF032";
    }
    if (id[1] == 0x25) {
      if (id[2] == 0x01) return "SST26VF016";
      if (id[2] == 0x02) return "SST26VF032";
      if (id[2] == 0x43) return "SST26VF064";
    }
  }
  return "(unknown chip)";
}
