/*********************************************************************************/
/*********************** EMULATOR OF THE SIGNALS RECEIVED ************************/
/********************** BY THE ELEMENTS OF AN ARRAY ANTENNA **********************/
/************************ IN LEO SATELLITES COMMUNICATION ************************/ 
/*********************************************************************************/

//#define TIME_SCANNING       // define to scan over time, not define to scan over theta

#ifdef TIME_SCANNING        // if time scanning, declare t as global variable
double t = 0;               // Time [s]
#else                       // if theta scanning, declare theta as global variable
double theta = -1.5708;     // Angle relative to broadside [rad], starting angle = -90deg
#endif

#define TIME_ACCEL  50      // define TIME_ACCEL 1 to have time of simulation correlated to real-life time

#include <SPI.h>

#define INTERRUPT_PERIOD    100000   // in microseconds, 1000 gives 1KHz

#define DDS_CS  PA4    //SPI_1 Chip Select pin is PA4
#define DDS_P0  PB12
#define DDS_SD1 PA0
#define DDS_SD3 PA1
#define DDS_RST PA2
#define DDS_UP  PA3
#define DDS_PDC PB14   // Power-Down Control

#define READ_DDS    0x80
#define WRITE_DDS   0x00

#define F_S         25000000  // fs = 25MHz
#define PLL         20

byte data;
HardwareTimer timer(2);   // Timer 2 for interruption

void setup() {
  /* SETUP UART */
  Serial.begin(115200, SERIAL_8N1);

  /* SETUP DIO PINS */
  pinMode(DDS_P0, OUTPUT);
  digitalWrite(DDS_P0, LOW); // Channel Frequency Tuning Word 0 (Register 0x04) is chosen

  pinMode(DDS_SD1, OUTPUT);
  digitalWrite(DDS_SD1, LOW); // If SD1 not used, set to 0

  pinMode(DDS_SD3, OUTPUT);
  digitalWrite(DDS_SD3, LOW); // If SD3 not used, must be set to 0

  pinMode(DDS_RST, OUTPUT);
  digitalWrite(DDS_RST, LOW); // RESET

  pinMode(DDS_UP, OUTPUT);
  digitalWrite(DDS_UP, LOW); // IO_UPDATE

  pinMode(DDS_PDC, OUTPUT);
  digitalWrite(DDS_PDC, LOW); // Power-Down Control

  pinMode(DDS_CS, OUTPUT);
  digitalWrite(DDS_CS, HIGH); // CHIP_SELECT

  /* SETUP SPI COMMUNICATION */
  SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_1 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // 72/2 = 36 // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)

  /* SETUP DDS */
  resetDDS();       // Forces internal registers to default state
  initializeDDS();  // Initialize register CSR, FR1, FR2 and CFR
  pulseUpdate();    // IO_UPDATE

  /* SETUP SIGNALS MODEL */
  delay(1000);  // so the terminal catches the start-up messages
  
  derive_variables();

  /* SETUP TIMER INTERRUPT */
  timer.pause();  // Pause the timer while we're configuring it
  timer.setPeriod(INTERRUPT_PERIOD); // Set up period in microseconds

  // Set up an interrupt on channel 1
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);               // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(handler_int
  );       // Interruption handler function: handler_int (change to testDDS or testFreq, at signals_model.ino, for testing the DDS)

  timer.refresh();  // Refresh the timer's count, prescale, and overflow
  timer.resume();   // Start the timer counting
}


void loop( )
{
  //
}


void handler_int( )
{
  uint32_t d, f;
  double phi_0, phi_1, phi_2, phi_3, ampl; 
  
  #ifdef TIME_SCANNING
  double theta;
  extern double t_end;
  if( t > t_end ) t = 0.0;

  d = distance_satellite_antenna_t( t );
  theta = angle_relative_to_broadside ( t, d );

  #else
  if( theta > 1.5708 ) theta = -1.5708;
  d = distance_satellite_antenna_theta ( theta );
  
  #endif
  
  f = frequency( theta );
  phi_0 = phase( 1, theta );
  phi_1 = phase( 2, theta );
  phi_2 = phase( 3, theta );
  phi_3 = phase( 4, theta );
  ampl = amplitude( d );
  
  /* PRINT VARIABLES (CAN'T DO IT IF TIME_SCANNING TOO LOW */
//  Serial.print( String( (String)"t = " + t + "\r" ) );
//  Serial.print( String( (String)"d = " + d + "\r" ) );
  Serial.print( String( (String)"theta = " + theta * 57.296 + "\r" ) );
//  Serial.print( String( (String)"f = " + f + "\r" ) );
//  Serial.print( String( (String)"phi_0 = " + phi_0 + "\r" ) );
//  Serial.print( String( (String)"phi_1 = " + phi_1 + "\r" ) );
//  Serial.print( String( (String)"phi_2 = " + phi_2 + "\r" ) );
//  Serial.print( String( (String)"phi_3 = " + phi_3 + "\r" ) );
//  Serial.print( String( (String)"ampl = " + ampl + "\r\r" ) );
//  Serial.print( String( (String)"[ " + t + "    " + d + "    " + theta + "    " + f + "    " + phi_0 + "    " + phi_1 + "    " + phi_2 + "    " + phi_3 + "    " + ampl + "   ]\r\r" ) );

  /* MODIFY ALL CHANNEL FREQUENCY AND AMPLITUDE */
  channelSel(4);             //select a channel (0,1,2,3) to write to.  4 selects all channels
  writeFreq(f);
  writeAmplitude(ampl);

  /* MODIFY CHANNEL 2 SIGNAL */
  channelSel(0);
  writePhase(phi_0);

  channelSel(1);
  writePhase(phi_1);

  channelSel(2);
  writePhase(phi_2);

  channelSel(3);
  writePhase(phi_3);

  /* UPDATE CHANGES */
  pulseUpdate();        //update output to the new frequency

  /* UPDATE TIME/THETA COUNTER */
  #ifdef TIME_SCANNING
  t = t + (double)INTERRUPT_PERIOD/1000000.0 * TIME_ACCEL;   // t will correspond to real-life time (accelerated by TIME_ACCEL)
//  t = t + t_end/16.0;                                      // will scan the pass over 16 discreet points

  #else
  theta = theta + 0.00872665;   //steps of 0.5deg, for 1.0deg, use 0.0174533;
  
  #endif
  

}
