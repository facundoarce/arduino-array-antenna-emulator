/*********************************************************************************/
/***************** SETUP AND COMMUNICATION WITH DDS ANALOG AD9959 ****************/ 
/*********************************************************************************/

/**
    Using the first SPI port (SPI_1)
    SS    <-->  PA4 <-->  BOARD_SPI1_NSS_PIN
    SCK   <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
    MISO  <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
    MOSI  <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN
**/

#include <SPI.h>

#define DDS_CS  PA4    //SPI_1 Chip Select pin is PA4
#define DDS_P0  PB12
#define DDS_SD1 PA0
#define DDS_SD3 PA1
#define DDS_RST PA2
#define DDS_UP  PA3
#define DDS_PDC PB14   // Power-Down Control

#define READ_DDS    0x80
#define WRITE_DDS   0x00

#define F_S         25000000  // System clock fs = 25MHz
#define PLL         20

uint32_t freq_test = 0;
byte data;

/********************************* TEST FUNCTIONS ********************************/ 

void testSPI()
{
  digitalWrite(DDS_CS, LOW); // manually take CSN low for SPI_1 transmission

  SPI.transfer(READ_DDS | 0x00);
  data = SPI.transfer(0x00);

  digitalWrite(DDS_CS, HIGH); // manually take CSN high between spi transmissions

  Serial.print(data);
  Serial.print("\r");
}

void testDDS()    // Test all the parameters (frequency, phase, amplitude) of DDS
{
  uint32_t freq;
  float phase, amplitude;

//  channelSel(0);
//  freq = 100000000;
//  writeFreq(freq);
//  amplitude = 1.0;
//  writeAmplitude(amplitude);
//  
//  channelSel(1);
//  freq = 100000000;
//  writeFreq(freq);
//  amplitude = 1.0;
//  writeAmplitude(amplitude);
//
//  channelSel(2);
//  freq = 100000000;
//  writeFreq(freq);
//  amplitude = 1.0;
//  writeAmplitude(amplitude);
//  phase = 120.0;
//  writePhase(phase);
//
//  channelSel(3);
//  freq = 100000000;
//  writeFreq(freq);
//  amplitude = 1.0;
//  writeAmplitude(amplitude);
//  phase = 240.0;
//  writePhase(phase);

  channelSel(4);    // all channels
  freq = 150000000;
  writeFreq(freq);
//  amplitude = 1.0;
//  writeAmplitude(amplitude);
//  phase = 240.0;
//  writePhase(phase);

  pulseUpdate();        //update output

}

void testFreq()     // Increase frequency with steps of 10MHz to analize filter characteristics of DDS
{
  channelSel(4);    // all channels
  freq_test = freq_test + 10000000;
  if ( freq_test > 250000000 ) freq_test = 0;
  writeFreq(freq_test);

  pulseUpdate();    //update output to the new frequency
}

/********************************* ACTUAL FUNCTIONS ********************************/ 

void setupDDS()
{
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
}

void resetDDS() //master resets the DDS
{
  digitalWrite(DDS_RST, HIGH);
  digitalWrite(DDS_RST, LOW);
}

void resetCommunication() //resets DDS internal communication (using before new command adds protection against previous bad writes)
{
  digitalWrite(DDS_CS, HIGH);
  digitalWrite(DDS_SD3, HIGH);
  digitalWrite(DDS_SD3, LOW);
  digitalWrite(DDS_CS, LOW);
}

void spiWrite(byte data[], int numBytes)  //writes an array of bytes to the DDS board. Takes an array of bytes to write, the length of the array, and SPI settings.
{
  resetCommunication();
  for (int i = 0; i < numBytes; i++) //for all bytes in byte array
  {
    SPI.transfer(data[i]);//write byte
  }
}

void initializeDDS() //writes all of the static settings to the DDS
{
  resetDDS();

  digitalWrite(DDS_CS, LOW);
  delay(1);

  //CSR Channel Select Register
  {
    byte valsToWrite[] = {B00000000, B11110010}; // 4 channels activated, single bit - 3 wires com, MSB
    spiWrite(valsToWrite, 2);
  }

  //FR1 Function Register 1
  {
    byte valsToWrite[] = {
      B00000001,  // Address 0x01
      1 << 7 | PLL << 2,   // VCO gain [23], PLL divider [22:18]
      B00000000,
      B00000000
    };
    spiWrite(valsToWrite, 4);
  }

  digitalWrite(DDS_CS, HIGH);
}

void pulseUpdate() //pulses an update to renew the frequency
{
  digitalWrite(DDS_UP, HIGH);
  digitalWrite(DDS_UP, LOW);
}

void channelSel(int chNO) //select a channel (0,1,2,3) to write to.  4 selects all channels
{
  byte chans[] = {B00010000, B00100000, B01000000, B10000000, B11110000};
  {
    byte valsToWrite[] = {B00000000, chans[chNO] | B00000010};
    spiWrite(valsToWrite, 2);
  }
}

void writeFreq(uint32_t frequency) //writes a new frequency to the selected channel
{
  uint32_t ftw = ( unsigned long )( ( float )frequency / ( F_S * PLL ) * 4294967296 );
  if ( ftw > 4294967295 ) ftw = 4294967295;   // maximum value 2^32 - 1 = 4294967295

  //CFTW0 Channel Frequency Tuning Word 0
  byte writeCFTW0_1 = ( byte )( ( ftw >> 24 ) & 0xff );
  byte writeCFTW0_2 = ( byte )( ( ftw >> 16 ) & 0xff );
  byte writeCFTW0_3 = ( byte )( ( ftw >> 8 ) & 0xff );
  byte writeCFTW0_4 = ( byte )( ftw & 0xff );
  byte valsToWrite[] = {B00000100, writeCFTW0_1, writeCFTW0_2, writeCFTW0_3, writeCFTW0_4};
  spiWrite(valsToWrite, 5);
}

void writePhase(float phase) //writes a new phase offset to the selected channel
{
  while ( phase >= 360.0 ) phase -= 360.0;
  
  uint16_t phow = ( uint16_t )( phase * 16384 / 360 );
  if ( phow > 16383 ) phow = 16383;   // maximum value 2^14 - 1 = 16383

  //CPOW0 Channel Phase Offset Word 0
  byte writeCPOW0_1 = ( byte )( ( phow >> 8 ) & 0xff );
  byte writeCPOW0_2 = ( byte )( phow & 0xff );
  byte valsToWrite[] = {B00000101, writeCPOW0_1, writeCPOW0_2};
  spiWrite(valsToWrite, 3);
}

void writeAmplitude(float amplitude) //writes a new amplitude scale factor to the selected channel
{
  uint16_t asf = ( uint16_t )( amplitude * 1024 );
  if ( asf > 1023 ) asf = 1023;   // maximum value 2^10 - 1 = 1023

  //ACR Amplitude Control Register
  byte writeACR_1 = B00000000;
  byte writeACR_2 = ( B00010000 ) | ( byte )( ( asf >> 8 ) & 0xff ); // 1 << 3 Amplitude multiplier enable [12]
  byte writeACR_3 = ( byte )( asf & 0xff );
  byte valsToWrite[] = {B00000110, writeACR_1, writeACR_2, writeACR_3};
  spiWrite(valsToWrite, 4);
}
