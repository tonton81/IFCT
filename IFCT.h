/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Designed and tested for PJRC Teensy (3.6 currently testing..).

  Forum link : https://forum.pjrc.com/threads/52906-IFCT-Improved-Flexcan-Teensy-Library

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#if !defined(_IFCT_H_) && ( defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) )
#define _IFCT_H_
#include <kinetis_flexcan.h>
#include <kinetis.h>
#include "Arduino.h"
#include "circular_buffer.h"

#define ext flags.extended
#define rtr flags.remote
#define FLEXCANb_MCR(b)  (*(vuint32_t*)(b))
#define FLEXCANb_CTRL1(b)(*(vuint32_t*)(b+4))
#define FLEXCANb_TIMER(b)(*(vuint32_t*)(b+8))
#define FLEXCANb_CTRL2(b)(*(vuint32_t*)(b+0x34))
#define FLEXCANb_RXMGMASK(b)              (*(vuint32_t*)(b+0x10))
#define FLEXCANb_IFLAG1(b)                (*(vuint32_t*)(b+0x30))
#define FLEXCANb_IMASK1(b)                (*(vuint32_t*)(b+0x28))
#define FLEXCANb_RXFGMASK(b)              (*(vuint32_t*)(b+0x48))
#define FLEXCANb_MBn_CS(b, n)             (*(vuint32_t*)(b+0x80+n*0x10))
#define FLEXCANb_MBn_ID(b, n)             (*(vuint32_t*)(b+0x84+n*0x10))
#define FLEXCANb_MBn_WORD0(b, n)          (*(vuint32_t*)(b+0x88+n*0x10))
#define FLEXCANb_MBn_WORD1(b, n)          (*(vuint32_t*)(b+0x8C+n*0x10))
#define FLEXCANb_IDFLT_TAB(b, n)          (*(vuint32_t*)(b+0xE0+(n*4)))
#define FLEXCANb_MB_MASK(b, n)            (*(vuint32_t*)(b+0x880+(n*4)))
#define FLEXCANb_ESR1(b) (*(vuint32_t*)(b+0x20))
#define FLEXCANb_MAXMB_SIZE(b)            (((FLEXCANb_MCR(b) & FLEXCAN_MCR_MAXMB_MASK) & 0x7F)+1)

#define FLEXCAN_BUFFER_SIZE 16

typedef struct CAN_message_t {
  uint32_t id;          // can identifier
  uint16_t timestamp;   // FlexCAN time when message arrived
  struct {
    uint8_t extended: 1; // identifier is extended (29-bit)
    uint8_t remote: 1;  // remote transmission request packet type
    uint8_t overrun: 1; // message overrun
    uint8_t reserved: 5;
  } flags;
  uint8_t len = 0;      // length of data
  uint8_t buf[8];       // data
  uint8_t mb = 0;       // used to identify mailbox reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
} CAN_message_t;


typedef struct CAN_filter_t {
    uint32_t id;
    struct {
        uint8_t extended:1;  // identifier is extended (29-bit)
        uint8_t remote:1;    // remote transmission request packet type
        uint8_t reserved:6;
    } flags;
} CAN_filter_t;


typedef enum IFCTMBNUM {
  MB0 = 0,
  MB1 = 1,
  MB2 = 2,
  MB3 = 3,
  MB4 = 4,
  MB5 = 5,
  MB6 = 6,
  MB7 = 7,
  MB8 = 8,
  MB9 = 9,
  MB10 = 10,
  MB11 = 11,
  MB12 = 12,
  MB13 = 13,
  MB14 = 14,
  MB15 = 15,
} IFCTMBNUM;
typedef enum IFCTMBTXRX {
  TX,
  RX,
  DISABLE
} IFCTMBTXRX;
typedef enum IFCTMBIDE {
  NONE = 0,
  IDE = 1,
} IFCTMBIDE;
typedef enum IFCTALTPIN {
  ALT = 0,
  DEF = 1,
} IFCTALTPIN;
typedef enum IFCTMBFLTEN {
  ACCEPT_ALL = 0,
  REJECT_ALL = 1
} IFCTMBFLTEN;


typedef void (*_MB_ptr)(const CAN_message_t &msg); /* mailbox / global callbacks */


class IFCT {

  public:
    IFCT(uint32_t baud = 1000000, uint32_t base = FLEXCAN0_BASE);
    void enableFIFOInterrupt(bool status = 1);
    void disableFIFOInterrupt();
    void enableFIFO(bool status = 1);
    void disableFIFO();
    void setBaudRate(uint32_t baud = 1000000);
    void setMB(const IFCTMBNUM &mb_num, const IFCTMBTXRX &mb_rx_tx, const IFCTMBIDE &ide = NONE);
    void enableMBInterrupt(const IFCTMBNUM &mb_num, bool status = 1);
    void disableMBInterrupt(const IFCTMBNUM &mb_num);
    void onReceive(const IFCTMBNUM &mb_num, _MB_ptr handler); /* individual mailbox callback function */
    void onReceive(_MB_ptr handler); /* global callback function */
    void mailboxStatus(); /* shows status of each mailbox, RX, TX, FIFO, etc... */
    static _MB_ptr _MB0handler; 
    static _MB_ptr _MB1handler; 
    static _MB_ptr _MB2handler; 
    static _MB_ptr _MB3handler; 
    static _MB_ptr _MB4handler; 
    static _MB_ptr _MB5handler; 
    static _MB_ptr _MB6handler; 
    static _MB_ptr _MB7handler; 
    static _MB_ptr _MB8handler; 
    static _MB_ptr _MB9handler; 
    static _MB_ptr _MB10handler; 
    static _MB_ptr _MB11handler; 
    static _MB_ptr _MB12handler; 
    static _MB_ptr _MB13handler; 
    static _MB_ptr _MB14handler; 
    static _MB_ptr _MB15handler; 
    static _MB_ptr _MBAllhandler; 
    bool pollFIFO(CAN_message_t &msg, bool poll = 1);
    int write(const CAN_message_t &msg); /* use any available mailbox for transmitting */
    int write(IFCTMBNUM mb_num, const CAN_message_t &msg); /* use a single mailbox for transmitting */
    int read(CAN_message_t &msg);
    void IFCT_message_ISR(void);
    void setTX(IFCTALTPIN which = DEF);
    void setRX(IFCTALTPIN which = DEF);
    void setMRP(bool mrp = 1); /* mailbox(1)/fifo(0) priority */
    void setRRS(bool rrs = 1); /* store remote frames */
    void setMaxMB(uint8_t last = 16);
    void setMBFilter(IFCTMBFLTEN input); /* enable/disable traffic for all MBs (for individual masking) */
    void setMBFilter(IFCTMBNUM mb_num, IFCTMBFLTEN input); /* set specific MB to accept/deny traffic */
    void setMBFilter(IFCTMBNUM mb_num, uint32_t id1); /* input 1 ID to be filtered */
    void setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2); /* input 2 ID's to be filtered */
    void setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3); /* input 3 ID's to be filtered */
    void setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4); /* input 4 ID's to be filtered */
    void setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t id5); /* input 5 ID's to be filtered */
    void setMBFilterRange(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2); /* filter a range of ids */
    void events();
    static Circular_Buffer<uint8_t, FLEXCAN_BUFFER_SIZE, sizeof(CAN_message_t)> flexcan_buffer; /* create an array buffer of struct size, 16 levels deep. */

  private:
    static bool can_events;
    void struct2queue(const CAN_message_t &msg);
    void queue2struct(CAN_message_t &msg);
    uint32_t _baseAddress = FLEXCAN0_BASE;
    uint32_t NVIC_IRQ = 0UL;
    void softReset();
    void FLEXCAN_EnterFreezeMode();
    void FLEXCAN_ExitFreezeMode();
    void setMBFilterProcessing(IFCTMBNUM mb_num, uint32_t filter_id, uint32_t calculated_mask);

};
extern IFCT Can0;
extern IFCT Can1;
#endif
