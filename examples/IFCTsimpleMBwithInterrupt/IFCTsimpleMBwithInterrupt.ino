#include <IFCT.h>

void setup() {
  pinMode(2, OUTPUT); // for the transceiver enable pin
  Can0.setBaudRate(1000000);
  Can0.onReceive(canSniff);
  Can0.enableMBInterrupt(MB0); // enable specific mailbox interrupts
  Can0.enableMBInterrupt(MB1);
  Can0.enableMBInterrupt(MB2);
  Can0.enableMBInterrupt(MB3);
  Can0.enableMBInterrupt(MB4);
  Can0.enableMBInterrupt(MB5);
  Can0.enableMBInterrupt(MB6);
  Can0.enableMBInterrupt(MB7);
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
