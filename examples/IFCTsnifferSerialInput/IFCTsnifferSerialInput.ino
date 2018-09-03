#include <IFCT.h>

#define cbsize 16

Circular_Buffer<uint32_t, cbsize> ids;
Circular_Buffer<uint32_t, cbsize, 10> storage;
Circular_Buffer<uint32_t, cbsize, 10> last_entries;

uint8_t option = 0;
uint8_t format = HEX;
uint32_t last_id = 0;

void setup() {
  pinMode(2, OUTPUT); // for the transceiver enable pin
  pinMode(13, OUTPUT);
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
      if ( !option ) Serial.print("Buffer full, couldn't add CAN ID to the list!");
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

  if ( !option ) main_console();
  if ( option == 1 && frame[0] == last_id ) {
    last_entries.push_front(frame, 10);
    last_output();
  }
}


void last_output() {
  uint32_t frame[10] = { last_id };
  Serial.print("\n\n\n   Lastest results for CAN ID: 0x");
  Serial.print(last_id, HEX);
  Serial.print("\t  ID hit count: ");

  ( storage.find(frame, 10, 0, 0, 0) ) ? Serial.print(frame[9]) : Serial.print("0");

  if ( format != BIN) Serial.print("\n\tDATA[0]\tDATA[1]\tDATA[2]\tDATA[3]\tDATA[4]\tDATA[5]\tDATA[6]\tDATA[7]\t\n\t ");
  else Serial.print("\n\t   DATA[0]\t   DATA[1]\t   DATA[2]\t   DATA[3]\t   DATA[4]\t   DATA[5]\t   DATA[6]\t   DATA[7]\t\n\t ");

  for ( uint32_t k = 0; k < last_entries.size(); k++ ) {
    last_entries.peek_front(frame, 10, k);
    for ( uint8_t i = 1; i < 9; i++ ) {
      if ( format == BIN ) {
        Serial.print("0b");
        for ( uint8_t j = 0;  j < abs((32 - __builtin_clz(frame[i])) - 8); j++ ) Serial.print("0");
        if ( frame[i] ) Serial.print(frame[i], format);
        Serial.print("\t ");
      }
      else {
        if ( format == HEX && i < 9 && i > 0 ) ( frame[i] < 0x10 ) ? Serial.print("0x0") : Serial.print("0x");
        Serial.print(frame[i], (i == 9) ? DEC : format); Serial.print("\t ");
      }


    } Serial.print("\n\t ");
  } Serial.println();
}

void main_console() {
  uint32_t frame[10];

  if ( format != BIN) Serial.print("\n\n\n\tCAN ID\tDATA[0]\tDATA[1]\tDATA[2]\tDATA[3]\tDATA[4]\tDATA[5]\tDATA[6]\tDATA[7]\tCOUNT\n    ");
  else Serial.print("\n\tCAN ID\t   DATA[0]\t   DATA[1]\t   DATA[2]\t   DATA[3]\t   DATA[4]\t   DATA[5]\t   DATA[6]\t   DATA[7]\tCOUNT\n    ");

  for ( uint32_t k = 0; k < storage.size(); k++ ) {
    frame[0] = ids.peek(k);
    storage.find(frame, 10, 0, 0, 0);
    for ( uint8_t i = 0; i < 10; i++ ) {
      if ( !i ) {
        char padded[12];
        sprintf(padded, "%08lX", frame[i]);
        Serial.print("0x");
        Serial.print(padded); Serial.print("\t ");
        continue;
      }
      if ( format == BIN ) {
        if ( i < 9 ) {
          Serial.print("0b");
          for ( uint8_t j = 0;  j < abs((32 - __builtin_clz(frame[i])) - 8); j++ ) Serial.print("0");
        }
        if ( frame[i] ) Serial.print(frame[i], (i == 9) ? DEC : format);
        Serial.print("\t ");
      }
      else {
        if ( format == HEX && i < 9 && i > 0 ) ( frame[i] < 0x10 ) ? Serial.print("0x0") : Serial.print("0x");
        Serial.print(frame[i], (i == 9) ? DEC : format); Serial.print("\t ");
      }
    } Serial.print("\n    ");
  } Serial.println();
}







void serialEvent() {

  char serial_buffer[50];
  uint8_t terminate = Serial.readBytesUntil('\n', serial_buffer, sizeof(serial_buffer) - 1);
  if ( serial_buffer[terminate] != '\0' ) serial_buffer[terminate] = '\0';
  char delimiters[] = "!:, ";
  char* _pos = strtok(serial_buffer, delimiters);
  switch ( _pos[0] ) {
    case 'O': {
        _pos = strtok(NULL, delimiters);

        uint32_t frame[10] = { ( !atoi(_pos) ) ? strtoul(_pos, NULL, 16) : atoi(_pos) };
        last_id = frame[0];

        if ( storage.find(frame, 10, 0, 0, 0) ) {
          last_entries.flush();
          last_entries.push_front(frame, 10);
          option = 1;
          last_output();
        }
        else option = 0;
        break;
      }
    case 'D': { // DEC output
        format = DEC;
        break;
      }
    case 'B': { // BIN output
        format = BIN;
        break;
      }
    case 'H': { // HEX output
        format = HEX;
        break;
      }
    case 'M': { // main output
        option = 0;
        break;
      }
    case 'R': { // reset
        option = 0;
        __disable_irq();
        storage.flush();
        ids.flush();
        __enable_irq();
        break;
      }

  }
  digitalWrite(13, !digitalRead(13));
}


