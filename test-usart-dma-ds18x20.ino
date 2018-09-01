#include "ds18x20.h"
DS18x20Serial ds(3); //num USART for DS18x20

float celsius;

void setup(void) {
  Serial.begin(2000000);
  ds.DS18x20SerialInit();
}

void loop(void) {
  ds.ds18x20_state_machine();
  if (ds.ds18x20_state_stop) {
    if (ds.ds18x20_present) {
      celsius = (float)ds.tt / 16.0;
      Serial.print("Temperature = ");
      Serial.println(celsius);
    } else {
      Serial.println("Temperature = XXXX");
    }
    ds.ds18x20_state = DS18x20_STATE_START;
  }

}

