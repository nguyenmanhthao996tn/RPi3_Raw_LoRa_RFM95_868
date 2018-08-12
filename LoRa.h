#ifndef __LORA_H__
#define __LORA_H__

#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <string.h>

/* 
  MISO  -> 21 (SPI_MISO)
  MOSI  -> 19 (SPI_MOSI)
  SCK   -> 23 (SPI_CLK)

  NSS   -> 13 (GPIO2)
  DIO0  -> 7  (GPIO7)
  DIO1  -> 15 (GPIO3)
  RESET -> 11 (GPIO0)

  BTN1 -> 31 (GPIO22)
  BTN2 -> 33 (GPIO23)

  LED1 -> 35 (GPIO24)
  LED2 -> 37 (GPIO25)
 */

#define SPI_CHANNEL 0

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

class LoRaClass
{
public:
  LoRaClass();

  int begin(long frequency);
  void end();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void setPins(int ss, int reset, int dio0);
  int parsePacket(int size = 0);
  int available();
  int read();
  int packetRssi();
  void idle();
  void sleep();

private:
  int _spiFd;
  int _ss;
  int _reset;
  int _dio0;
  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  void setLdoFlag();

  void explicitHeaderMode();
  void implicitHeaderMode();

  int getSpreadingFactor();
  long getSignalBandwidth();
};

extern LoRaClass LoRa;

#endif /* __LORA_H__ */
