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

uint32_t freq_test = 0;

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

void channelSel(int chNO) //sets channel to write to
{
  byte chans[] = {B0010000, B00100000, B01000000, B10000000, B11110000};
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
