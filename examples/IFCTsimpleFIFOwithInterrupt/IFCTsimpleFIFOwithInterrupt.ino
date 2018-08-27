#include <IFCT.h>

void setup() {
  pinMode(2, OUTPUT); // for the transceiver enable pin
  delay(1000);
  Can0.setBaudRate(1000000);
  Can0.enableFIFO(1);
  Can0.enableFIFOInterrupt(1);
  Can0.onReceive(canSniff);
  Can0.intervalTimer(); // enable queue system and run callback in background.
}

void loop() {
  // put your main code here, to run repeatedly:
}

void canSniff(const CAN_message_t &msg) { // global callback
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" REMOTE: "); Serial.print(msg.rtr);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}
