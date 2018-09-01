#ifndef _DS18B20SERIAL_H_
#define _DS18B20SERIAL_H_

#include <inttypes.h>
#include "Arduino.h"       // for delayMicroseconds, digitalPinToBitMask, etc

#include <libmaple/dma.h>
#include <libmaple/usart.h>
#include <libmaple/gpio.h>
#include <HardwareSerial.h>
#include <HardwareTimer.h>


#define BAUD 115200
#define OW_0    0x00
#define OW_1    0xff
#define OW_R    0xff
#define DS18x20_STATE_FINISH 13
#define DS18x20_STATE_START 1


class DS18x20Serial {

  public:
    DS18x20Serial(uint8_t NumUSART);
    void ds18x20_state_machine(void);
    void DS18x20SerialInit(void);
    bool ds18x20_present = false;
    bool ds18x20_state_stop = false;
    uint8_t ds18x20_state = 1;
    uint16_t tt = 0;

  private:
    bool OW_ConvertDataTT(void);
    void OW_SendCommandReadData(void);
    void OW_SendCommand(void);
    void OW_Reset(void);
    void setup_dma_xfer(void);
    void setup_tube_config_rx(void);
    void setup_tube_config_tx(void);
    void setup_usart(void);
 
  protected:
};
#endif

