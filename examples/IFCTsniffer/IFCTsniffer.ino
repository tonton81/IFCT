#include <IFCT.h>

#define cbsize 16

Circular_Buffer<uint32_t, cbsize> ids;
Circular_Buffer<uint32_t, cbsize, 10> storage;

void setup() {
  pinMode(2, OUTPUT); // for the transceiver enable pin
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
  uint32_t frame[10] = { msg.id };

  if ( !storage.find(frame, 10, 0, 0, 0) ) {
    if ( storage.size() == storage.capacity() ) {
      Serial.print("Buffer full, couldn't add CAN ID to the list!");
      return;
    }
    frame[0] = msg.id;
    for ( uint8_t i = 0; i < 8; i++ ) frame[i + 1] = msg.buf[i];
    frame[9] = 1;
    storage.push_back(frame, 10);
    ids.push_back(msg.id);
    ids.sort_ascending();
  }
  else {
    frame[9]++;
    for ( uint8_t i = 0; i < 8; i++ ) frame[i + 1] = msg.buf[i];
    storage.replace(frame, 10, 0, 0, 0);
  }
  Serial.print("\n\n\n\tCAN ID\tDATA[0]\tDATA[1]\tDATA[2]\tDATA[3]\tDATA[4]\tDATA[5]\tDATA[6]\tDATA[7]\tCOUNT\n\t   ");
  for ( uint32_t k = 0; k < storage.size(); k++ ) {
    frame[0] = ids.peek(k);
    storage.find(frame, 10, 0, 0, 0);
    for ( uint8_t i = 0; i < 10; i++ ) {
      Serial.print(frame[i]); Serial.print("\t ");
    } Serial.print("\n\t   ");
  } Serial.println();
}
