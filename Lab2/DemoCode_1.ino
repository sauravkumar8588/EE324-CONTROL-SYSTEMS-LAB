/* Include the SPI library for the arduino boards */
#include <SPI.h>
 
/* Serial rates for UART */
#define BAUDRATE        115200
 
/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70
 
/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09
 
/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14
 
/* SPI pins */
#define ENC_0            3
#define ENC_1            2
#define SPI_MOSI        51
#define SPI_MISO        50
#define SPI_SCLK        52
 
/*motor PINS*/
#define motor_plus_pin 9
#define motor_minus_pin 8
 
float control_input;
 
float k[] = {35.48, -1415.60, 29.27, -114.57};
// {31.62, -1398.06, 27.48, -118.45}; first good response
// {35.48, -1415.60, 29.27, -114.57} 2nd good response better theta coverage and lasted much longer
void setup()
{
  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_0, OUTPUT);
  pinMode(ENC_1, OUTPUT);
 
  //Initialize the UART serial connection for debugging
  Serial.begin(BAUDRATE);
 
  //Get the CS line high which is the default inactive state
  digitalWrite(ENC_0, HIGH);
  digitalWrite(ENC_1, HIGH);
 
  //set the clockrate. Uno clock rate is 16Mhz, divider of 32 gives 500 kHz.
  //500 kHz is a good speed for our test environment
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  
  //start SPI bus
  SPI.begin();
 
  setZeroSPI(ENC_0);
  setZeroSPI(ENC_1);
}
 
void loop()
{
  //create a 16 bit variable to hold the encoders position
  uint16_t pendulumArm_current = getPositionSPI(ENC_0, RES14);
  if(pendulumArm_current != 0xFFFF) {
    uint16_t motorArm_current = getPositionSPI(ENC_1, RES14);
    if(motorArm_current != 0xFFFF) {
      uint16_t pendulumArm_prev = pendulumArm_current;
      uint16_t motorArm_prev = motorArm_current;
    
      //if you want to set the zero position before beggining uncomment the following function call
      int t_current;
      int t_prev = 0;
      float alpha = 3.1416 - pendulumArm_current*2*3.1416/16384.0;
      float theta = 3.1416 - motorArm_current*2*3.1416/16384.0;
      float alpha_prev = alpha;
      float theta_prev = theta;
 
      while(!(abs(alpha) < 0.02) || !(abs(theta) < 0.02)) {
        pendulumArm_current = getPositionSPI(ENC_0, RES14);
        if(pendulumArm_current != 0xFFFF) {
          uint16_t motorArm_current = getPositionSPI(ENC_1, RES14);
          if(motorArm_current != 0xFFFF) {
            alpha = 3.1416 - pendulumArm_current*2*3.1416/16384.0;
            theta = 3.1416 - motorArm_current*2*3.1416/16384.0;
            Serial.print("alpha: ");
            Serial.print(alpha);
            Serial.print(", theta: ");
            Serial.print(theta);
            Serial.println(". Set Pendulum");
          }
          else {
            Serial.print("pin ");
            Serial.print(ENC_1);
            Serial.println(":ENC_1 error.");
          }
        }
        else {
          Serial.print("pin ");
          Serial.print(ENC_0);
          Serial.println(":ENC_0 error.");
          uint16_t motorArm_current = getPositionSPI(ENC_1, RES14);
          if(motorArm_current == 0xFFFF) {
            Serial.print("pin ");
            Serial.print(ENC_1);
            Serial.println(":ENC_1 error.");
          }
        }
      }
 
      while(1) {
        t_current = millis();
        int dt = t_current - t_prev;
        t_prev = t_current;
        uint16_t pendulumArm_current = getPositionSPI(ENC_0, RES14);
        if(pendulumArm_current != 0xFFFF) {
          uint16_t motorArm_current = getPositionSPI(ENC_1, RES14);
          if(motorArm_current != 0xFFFF) {
            float alpha = 3.1416 - pendulumArm_current*2*3.1416/16384.0;
            float theta = 3.1416 - motorArm_current*2*3.1416/16384.0;
            float alpha_dot = 1000.0*(alpha - alpha_prev)/dt;
            float theta_dot = 1000.0*(theta - theta_prev)/dt;
            Serial.print("alpha: ");
            Serial.print(alpha);
            Serial.print(", theta: ");
            Serial.print(theta);
            Serial.print(", alpha_prev: ");
            Serial.print(alpha_prev);
            Serial.print(", theta_prev: ");
            Serial.print(theta_prev);
            Serial.print(", alpha_dot: ");
            Serial.print(alpha_dot);
            Serial.print(", theta_dot: ");
            Serial.print(theta_dot);
            control_input = -(k[0]*theta + k[1]*alpha + k[2]*theta_dot + k[3]*alpha_dot)*0.8;
            if(control_input > 255.0) {control_input = 255.0;}
            if(control_input < -255.0) {control_input = -255.0;}
            float motor_plus_value = (128.0 + (control_input)/2);
            float motor_minus_value = (128.0 - (control_input)/2);
            Serial.print(", motor_plus_value: ");
            Serial.print(motor_plus_value);
            Serial.print(", motor_minus_value: ");
            Serial.println(motor_minus_value);
            analogWrite(motor_plus_pin, motor_plus_value);
            analogWrite(motor_minus_pin, motor_minus_value);
            alpha_prev = alpha;
            theta_prev = theta;
          }
          else {
            Serial.print("pin ");
            Serial.print(ENC_1);
            Serial.println(":ENC_1 error.");
          }
        }
        else {
          Serial.print("pin ");
          Serial.print(ENC_0);
          Serial.println(":ENC_0 error.");
          uint16_t motorArm_current = getPositionSPI(ENC_1, RES14);
          if(motorArm_current == 0xFFFF) {
            Serial.print("pin ");
            Serial.print(ENC_1);
            Serial.println(":ENC_1 error.");
          }
        }
      }
    }
    else {
      Serial.print("pin ");
      Serial.print(ENC_1);
      Serial.println(":ENC_1 error.");
    }
  }
  else {
    Serial.print("pin ");
    Serial.print(ENC_0);
    Serial.println(":ENC_0 error.");
    uint16_t motorArm_current = getPositionSPI(ENC_1, RES14);
    if(motorArm_current == 0xFFFF) {
      Serial.print("pin ");
      Serial.print(ENC_1);
      Serial.println(":ENC_1 error.");
    }
  }
}
 
/*
   This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
   for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
   For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
   is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
   This function takes the pin number of the desired device as an input
   This funciton expects res12 or res14 to properly format position responses.
   Error values are returned as 0xFFFF
*/
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum
 
  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;
 
  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);
 
  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);
 
  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for (int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));
 
  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
      && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
  {
    //we got back a good position, so just mask away the checkbits
    currentPosition &= 0x3FFF;
  }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }
 
  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;
 
  return currentPosition;
}
 
/*
   This function does the SPI transfer. sendByte is the byte to transmit.
   Use releaseLine to let the spiWriteRead function know if it should release
   the chip select line after transfer.
   This function takes the pin number of the desired device as an input
   The received data is returned.
*/
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;
 
  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder , LOW);
 
  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);
 
  //send the command
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
 
  return data;
}
 
/*
   This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere
   This function takes the pin number of the desired device as an input
*/
void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}
 
/*
   The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
   second byte is the command.
   This function takes the pin number of the desired device as an input
*/
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);
 
  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);
 
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}
 
/*
   The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
   second byte is the command.
   This function takes the pin number of the desired device as an input
*/
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);
 
  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);
 
  spiWriteRead(AMT22_RESET, encoder, true);
 
  delay(250); //250 second delay to allow the encoder to start back up
}
