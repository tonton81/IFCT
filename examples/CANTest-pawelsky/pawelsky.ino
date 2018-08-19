#include <IFCT.h>

FlexCAN CANbus0(500000, 0);

static CAN_message_t msg;
static uint8_t hex[17] = "0123456789abcdef";

// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr) {
  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
    Serial.print(" ");
  }
  //  Serial.write('\r');
  // Serial.write('\n');
}


// -------------------------------------------------------------
void setup(void)
{
  CANbus0.begin();
  pinMode(2, OUTPUT); digitalWrite(2, LOW);
}


// -------------------------------------------------------------
void loop(void) {
  if (CANbus0.available()) {
    CANbus0.read(msg);
    Serial.print("CAN bus 0: "); hexDump(8, msg.buf);
    Serial.print(" ID: 0x"); Serial.print(msg.id, HEX);
    Serial.write('\r');
    Serial.write('\n');

    CANbus0.write(msg);
  }
}
