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

#include <IFCT.h>
#include <kinetis_flexcan.h>
#include <kinetis.h>
#include "Arduino.h"

Circular_Buffer<uint8_t, FLEXCAN_BUFFER_SIZE, sizeof(CAN_message_t)> IFCT::flexcan_buffer;
bool IFCT::can_events = 0;

_MB_ptr IFCT::_MB0handler = nullptr;
_MB_ptr IFCT::_MB1handler = nullptr;
_MB_ptr IFCT::_MB2handler = nullptr;
_MB_ptr IFCT::_MB3handler = nullptr;
_MB_ptr IFCT::_MB4handler = nullptr;
_MB_ptr IFCT::_MB5handler = nullptr;
_MB_ptr IFCT::_MB6handler = nullptr;
_MB_ptr IFCT::_MB7handler = nullptr;
_MB_ptr IFCT::_MB8handler = nullptr;
_MB_ptr IFCT::_MB9handler = nullptr;
_MB_ptr IFCT::_MB10handler = nullptr;
_MB_ptr IFCT::_MB11handler = nullptr;
_MB_ptr IFCT::_MB12handler = nullptr;
_MB_ptr IFCT::_MB13handler = nullptr;
_MB_ptr IFCT::_MB14handler = nullptr;
_MB_ptr IFCT::_MB15handler = nullptr;
_MB_ptr IFCT::_MBAllhandler = nullptr;


IFCT Can0 = IFCT(1000000,FLEXCAN0_BASE);
#if defined(__MK66FX1M0__)
IFCT Can1 = IFCT(1000000,FLEXCAN1_BASE);
#endif

IFCT::IFCT(uint32_t baud, uint32_t base) {

  Serial.begin(115200); // usb serial
  NVIC_SET_PRIORITY(IRQ_USBOTG, 0);

  _baseAddress = base;
#if !defined(__MK20DX256__) && !defined(__MK64FX512__) && !defined(__MK66FX1M0__)
  return; /* mcu not supported */
#endif

#if defined(__MK20DX256__)
  if( base == FLEXCAN0_BASE ) NVIC_IRQ = IRQ_CAN_MESSAGE;
#endif

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if( base == FLEXCAN0_BASE ) NVIC_IRQ = IRQ_CAN0_MESSAGE;
#endif

#if defined(__MK66FX1M0__)
  else if( base == FLEXCAN1_BASE ) NVIC_IRQ = IRQ_CAN1_MESSAGE;
#endif

  OSC0_CR |= OSC_ERCLKEN;
  if ( base == FLEXCAN0_BASE ) SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
#if defined(__MK66FX1M0__)
  else if( base == FLEXCAN1_BASE ) SIM_SCGC3 |= SIM_SCGC3_FLEXCAN1;
#endif
  FLEXCANb_CTRL1(_baseAddress) &= ~FLEXCAN_CTRL_CLK_SRC;
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_FRZ; /* enable module */
  FLEXCANb_MCR(_baseAddress) &= ~FLEXCAN_MCR_MDIS; /* enable module */

  while (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_LPM_ACK);
  softReset(); /* reset bus */
  while (!(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK));
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_SRX_DIS; /* Disable self-reception */

  disableFIFO();
  disableFIFOInterrupt();
  setBaudRate(baud);
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_IRMQ; // individual mailbox masking
  FLEXCANb_CTRL2(_baseAddress) |= FLEXCAN_CTRL2_RRS | // store remote frames
                    FLEXCAN_CTRL2_MRP; // mailbox > FIFO priority.
  FLEXCANb_MCR(_baseAddress) &= ~FLEXCAN_MCR_HALT; /* start the CAN */

  while (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK);
  while (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_NOT_RDY); /* wait until ready */

  setRX(); setTX();

  NVIC_SET_PRIORITY(NVIC_IRQ, 1); /* set interrupt priority */
  NVIC_ENABLE_IRQ(NVIC_IRQ); /* enable message interrupt */

}
 
void IFCT::softReset() {
  /*
    Soft Reset
    The following registers are reset: MCR (except the MDIS bit), TIMER , ECR, ESR1, ESR2,
    IMASK1, IMASK2, IFLAG1, IFLAG2 and CRCR. Configuration registers that control the interface to the
    CAN bus are not affected by soft reset.
    The following registers are unaffected: CTRL1, CTRL2, all RXIMR
    registers, RXMGMASK, RX14MASK, RX15MASK, RXFGMASK, RXFIR, all Message Buffers .
  */
  FLEXCANb_MCR(_baseAddress) ^= FLEXCAN_MCR_SOFT_RST;
  while (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_SOFT_RST);
}


void IFCT::enableFIFO(bool status) {
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  FLEXCANb_MCR(_baseAddress) &= ~FLEXCAN_MCR_FEN; // Disable FIFO if already enabled for cleanup.
  FLEXCANb_IMASK1(_baseAddress) = 0UL; // disable all FIFO/MB Interrupts
  for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) { // clear all mailboxes
    FLEXCANb_MBn_ID(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_WORD0(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_WORD1(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_CS(_baseAddress, i) = 0x00000000;
  }
  for ( uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) FLEXCANb_MB_MASK(_baseAddress, i) = 0UL; // CLEAR MAILBOX MASKS (RXIMR)
  /*
    RXMGMASK is provided for legacy application support.
    •  When the MCR[IRMQ] bit is negated, RXMGMASK is always in effect.
    •  When the MCR[IRMQ] bit is asserted, RXMGMASK has no effect.
    RXMGMASK is used to mask the filter fields of all Rx MBs, excluding MBs 14-15,
    which have individual mask registers
    RX14MASK/RX15MASK is provided for legacy application support. When the MCR[IRMQ] bit is
    asserted, RX14MASK/RX15MASK has no effect
  */
  FLEXCANb_RXMGMASK(_baseAddress) = 0;
  FLEXCANb_RXFGMASK(_baseAddress) = 0;
  /*
      Enable RX FIFO
      Before enabling the RFEN, the CPU must service the IFLAG bits asserted in the Rx
      FIFO region; Otherwise, these IFLAG bits will mistakenly show
      the related MBs now belonging to FIFO as having contents to be serviced.
  */
  FLEXCANb_IFLAG1(_baseAddress) = FLEXCANb_IFLAG1(_baseAddress); // (all bits reset when written back)
  if ( status ) {
    FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_FEN;
//QUICKFIND-KEYWORD
    /*
      Each group of eight filters occupies a memory space equivalent to two Message Buffers which
      means that the more filters are implemented the less Mailboxes will be available.
    */
    FLEXCAN_set_rffn(FLEXCANb_CTRL2(_baseAddress), 0); // setup 8 Filters for FIFO, 0-5 = FIFO, 6-7 FILTER, 8-16 MBs, max value 0x3 which leaves MB14/15 free to use.
    // Setup TX mailboxes from 8 -> 15, FIFO uses the first 8 (6MB for FIFO, 2MB for 8 filters for FIFO).
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    for (uint8_t i = remaining_mailboxes; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) FLEXCANb_MBn_CS(_baseAddress,i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  } 
  else { // FIFO disabled default setup of mailboxes, 0-7 RX, 8-15 TX
    for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) { // clear all mailboxes
      if ( i < 8 ) {
        FLEXCANb_MBn_ID(_baseAddress, i) = 0x00000000;
        FLEXCANb_MBn_WORD0(_baseAddress, i) = 0x00000000;
        FLEXCANb_MBn_WORD1(_baseAddress, i) = 0x00000000;
        if ( i < 4 ) FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
        else FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_IDE;
      }
      else {
        FLEXCANb_MBn_ID(_baseAddress, i) = 0x00000000;
        FLEXCANb_MBn_WORD0(_baseAddress, i) = 0x00000000;
        FLEXCANb_MBn_WORD1(_baseAddress, i) = 0x00000000;
        FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
      }
    }
  }
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}


void IFCT::disableFIFO() {
  enableFIFO(0);
}


void IFCT::enableFIFOInterrupt(bool status) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) ) {
    Serial.println("FIFO must be enabled first"); return;
  }
  if ( FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I ) {
    Serial.println("FIFO interrupts already enabled"); return;
  }
  ( !status ) ? FLEXCANb_IMASK1(_baseAddress) &= ~FLEXCAN_IFLAG1_BUF5I /* disable FIFO interrupt */ : FLEXCANb_IMASK1(_baseAddress) |= FLEXCAN_IFLAG1_BUF5I; /* enable FIFO interrupt */
}


void IFCT::disableFIFOInterrupt() {
  enableFIFOInterrupt(0);
}


void IFCT::setBaudRate(uint32_t baud) {
  uint32_t divisor = 0, bestDivisor = 0, result = 16000000 / baud / (divisor + 1);
  int error = baud - (16000000 / (result * (divisor + 1))), bestError = error;
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  while (result > 5) {
    divisor++;
    result = 16000000 / baud / (divisor + 1);
    if (result <= 25) {
      error = baud - (16000000 / (result * (divisor + 1)));
      if (error < 0) error *= -1;
      if (error < bestError) {
        bestError = error;
        bestDivisor = divisor;
      }
      if ((error == bestError) && (result > 11) && (result < 19)) {
        bestError = error;
        bestDivisor = divisor;
      }
    }
  }

  divisor = bestDivisor;
  result = 16000000 / baud / (divisor + 1);

  if ((result < 5) || (result > 25) || (bestError > 300)) return;
  result -= 5; // the bitTimingTable is offset by 5 since there was no reason to store bit timings for invalid numbers
  uint8_t bitTimingTable[21][3] = {
    {0, 0, 1}, //5
    {1, 0, 1}, //6
    {1, 1, 1}, //7
    {2, 1, 1}, //8
    {2, 2, 1}, //9
    {2, 3, 1}, //10
    {2, 3, 2}, //11
    {2, 4, 2}, //12
    {2, 5, 2}, //13
    {2, 5, 3}, //14
    {2, 6, 3}, //15
    {2, 7, 3}, //16
    {2, 7, 4}, //17
    {3, 7, 4}, //18
    {3, 7, 5}, //19
    {4, 7, 5}, //20
    {4, 7, 6}, //21
    {5, 7, 6}, //22
    {6, 7, 6}, //23
    {6, 7, 7}, //24
    {7, 7, 7}, //25
  }, propSeg = bitTimingTable[result][0], pSeg1 = bitTimingTable[result][1], pSeg2 = bitTimingTable[result][2];
  FLEXCANb_CTRL1(_baseAddress) = (FLEXCAN_CTRL_PROPSEG(propSeg) | FLEXCAN_CTRL_RJW(1) | FLEXCAN_CTRL_PSEG1(pSeg1) |
                    FLEXCAN_CTRL_PSEG2(pSeg2) | FLEXCAN_CTRL_PRESDIV(divisor));
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}


void IFCT::setMB(const IFCTMBNUM &mb_num, const IFCTMBTXRX &mb_rx_tx, const IFCTMBIDE &ide) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) {
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB(16) - FIFO(6) */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    if ( mb_num < ( FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes ) ) {
      Serial.println("Mailbox not available"); return;
    }
  }
  if ( mb_rx_tx == RX ) {
    FLEXCANb_IMASK1(_baseAddress) &= ~(1UL << mb_num); /* immediately disable mailbox interrupt */
    FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mb_num)); // Reading Control Status atomically locks mailbox (if it is RX mode).
    FLEXCANb_MBn_ID(_baseAddress, mb_num) = 0x00000000;
    FLEXCANb_MBn_WORD0(_baseAddress, mb_num) = 0x00000000;
    FLEXCANb_MBn_WORD1(_baseAddress, mb_num) = 0x00000000;
    if ( ide != EXT ) FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
    else FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  if ( mb_rx_tx == TX ) {
    FLEXCANb_IMASK1(_baseAddress) &= ~(1UL << mb_num); /* immediately disable mailbox interrupt */
    FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mb_num)); // Reading Control Status atomically locks mailbox (if it is RX mode).
    FLEXCANb_MBn_ID(_baseAddress, mb_num) = 0x00000000;
    FLEXCANb_MBn_WORD0(_baseAddress, mb_num) = 0x00000000;
    FLEXCANb_MBn_WORD1(_baseAddress, mb_num) = 0x00000000;
    FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
  FLEXCANb_TIMER(_baseAddress); // reading timer unlocks individual mailbox
}
void IFCT::enableMBInterrupt(const IFCTMBNUM &mb_num, bool status) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) {
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB(16) - FIFO(6) */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    if ( mb_num < ( FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes ) ) {
      Serial.println("Mailbox not available"); return;
    }
  }
  if ( status ) {
    if ( !(FLEXCANb_IMASK1(_baseAddress) & ( 1UL << mb_num )) ) FLEXCANb_IMASK1(_baseAddress) |= (1UL << mb_num); /* enable mailbox interrupt */
    else {
      Serial.print("Mailbox interrupt already enabled for MB"); Serial.println(mb_num);
    }
  }
  else FLEXCANb_IMASK1(_baseAddress) &= ~(1UL << mb_num); /* disable mailbox interrupt */
}
void IFCT::disableMBInterrupt(const IFCTMBNUM &mb_num) {
  enableMBInterrupt(mb_num, 0);
}


void IFCT::setMaxMB(uint8_t last) {

#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) 
  last = constrain(last,1,16);
  last--;
#endif

  FLEXCAN_EnterFreezeMode();
  bool fifo_was_cleared = 0;

  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) {
    disableFIFO(); /* let fifo clear current mailboxes */
    fifo_was_cleared = 1;
  }
  else disableFIFO();
  
  FLEXCANb_IFLAG1(_baseAddress) = FLEXCANb_IFLAG1(_baseAddress); // (all bits reset when written back) (needed for MAXMB changes)
  FLEXCANb_MCR(_baseAddress) &= ~FLEXCAN_MCR_MAXMB_MASK; // disable mailboxes
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_MAXMB(last); // set mailbox max

  if ( fifo_was_cleared ) enableFIFO();

  FLEXCAN_ExitFreezeMode();
}


void IFCT::FLEXCAN_EnterFreezeMode() {
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_HALT;
  while (!(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK));
}

void IFCT::FLEXCAN_ExitFreezeMode() {
  FLEXCANb_MCR(_baseAddress) &= ~FLEXCAN_MCR_HALT;
  while (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK);
}

void IFCT::setMRP(bool mrp) { /* mailbox priority (1) or FIFO priority (0) */
  FLEXCAN_EnterFreezeMode();
  if ( mrp ) FLEXCANb_CTRL2(_baseAddress) |= FLEXCAN_CTRL2_MRP;
  else FLEXCANb_CTRL2(_baseAddress) &= ~FLEXCAN_CTRL2_MRP;
  FLEXCAN_ExitFreezeMode();
}

void IFCT::setRRS(bool rrs) { /* store remote frames */
  FLEXCAN_EnterFreezeMode();
  if ( rrs ) FLEXCANb_CTRL2(_baseAddress) |= FLEXCAN_CTRL2_RRS;
  else FLEXCANb_CTRL2(_baseAddress) &= ~FLEXCAN_CTRL2_RRS;
  FLEXCAN_ExitFreezeMode();
}




void IFCT::onReceive(const IFCTMBNUM &mb_num, _MB_ptr handler) {
  switch ( mb_num ) {
    case MB0: {
        _MB0handler = handler; break;
      }
    case MB1: {
        _MB1handler = handler; break;
      }
    case MB2: {
        _MB2handler = handler; break;
      }
    case MB3: {
        _MB3handler = handler; break;
      }
    case MB4: {
        _MB4handler = handler; break;
      }
    case MB5: {
        _MB5handler = handler; break;
      }
    case MB6: {
        _MB6handler = handler; break;
      }
    case MB7: {
        _MB7handler = handler; break;
      }
    case MB8: {
        _MB8handler = handler; break;
      }
    case MB9: {
        _MB9handler = handler; break;
      }
    case MB10: {
        _MB10handler = handler; break;
      }
    case MB11: {
        _MB11handler = handler; break;
      }
    case MB12: {
        _MB12handler = handler; break;
      }
    case MB13: {
        _MB13handler = handler; break;
      }
    case MB14: {
        _MB14handler = handler; break;
      }
    case MB15: {
        _MB15handler = handler; break;
      }
  } // end switch
}
void IFCT::onReceive(_MB_ptr handler) {
  _MB0handler = nullptr;
  _MB1handler = nullptr;
  _MB2handler = nullptr;
  _MB3handler = nullptr;
  _MB4handler = nullptr;
  _MB5handler = nullptr;
  _MB6handler = nullptr;
  _MB7handler = nullptr;
  _MB8handler = nullptr;
  _MB9handler = nullptr;
  _MB10handler = nullptr;
  _MB11handler = nullptr;
  _MB12handler = nullptr;
  _MB13handler = nullptr;
  _MB14handler = nullptr;
  _MB15handler = nullptr;
  _MBAllhandler = handler;
}
void sendMSGtoIndividualMBCallback(const IFCTMBNUM &mb_num, const CAN_message_t &msg) { /* this is global for ISR use */
  switch (mb_num) {
    case MB0: {
        if ( IFCT::_MB0handler != nullptr ) IFCT::_MB0handler(msg); break;
      }
    case MB1: {
        if ( IFCT::_MB1handler != nullptr ) IFCT::_MB1handler(msg); break;
      }
    case MB2: {
        if ( IFCT::_MB2handler != nullptr ) IFCT::_MB2handler(msg); break;
      }
    case MB3: {
        if ( IFCT::_MB3handler != nullptr ) IFCT::_MB3handler(msg); break;
      }
    case MB4: {
        if ( IFCT::_MB4handler != nullptr ) IFCT::_MB4handler(msg); break;
      }
    case MB5: {
        if ( IFCT::_MB5handler != nullptr ) IFCT::_MB5handler(msg); break;
      }
    case MB6: {
        if ( IFCT::_MB6handler != nullptr ) IFCT::_MB6handler(msg); break;
      }
    case MB7: {
        if ( IFCT::_MB7handler != nullptr ) IFCT::_MB7handler(msg); break;
      }
    case MB8: {
        if ( IFCT::_MB8handler != nullptr ) IFCT::_MB8handler(msg); break;
      }
    case MB9: {
        if ( IFCT::_MB9handler != nullptr ) IFCT::_MB9handler(msg); break;
      }
    case MB10: {
        if ( IFCT::_MB10handler != nullptr ) IFCT::_MB10handler(msg); break;
      }
    case MB11: {
        if ( IFCT::_MB11handler != nullptr ) IFCT::_MB11handler(msg); break;
      }
    case MB12: {
        if ( IFCT::_MB12handler != nullptr ) IFCT::_MB12handler(msg); break;
      }
    case MB13: {
        if ( IFCT::_MB13handler != nullptr ) IFCT::_MB13handler(msg); break;
      }
    case MB14: {
        if ( IFCT::_MB14handler != nullptr ) IFCT::_MB14handler(msg); break;
      }
    case MB15: {
        if ( IFCT::_MB15handler != nullptr ) IFCT::_MB15handler(msg); break;
      }
  }
}

bool IFCT::pollFIFO(CAN_message_t &msg, bool poll) {
  if ( !( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) ) return 0; /* FIFO is NOT enabled! */
  if ( !poll ) NVIC_ENABLE_IRQ(NVIC_IRQ); /* enable message interrupt */
  else NVIC_DISABLE_IRQ(NVIC_IRQ); /* disable message interrupt */
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, capture frames */
    if ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I ) {
      msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_baseAddress, 0));
      msg.flags.extended = (FLEXCANb_MBn_CS(_baseAddress, 0) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
      msg.flags.remote = (FLEXCANb_MBn_CS(_baseAddress, 0) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
      msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_baseAddress, 0));
      msg.id = (FLEXCANb_MBn_ID(_baseAddress, 0) & FLEXCAN_MB_ID_EXT_MASK);
      if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
      uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, 0);
      msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
      dataIn = FLEXCANb_MBn_WORD1(_baseAddress, 0);
      msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
      FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF5I; /* clear FIFO bit only! */
      if ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF6I ) FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF6I;
      if ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF7I ) FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF7I;
      return 1;
    }
  }
  return 0;
}


int IFCT::write(const CAN_message_t &msg) {
  int8_t retries = 3;
retry_tx_flexcan:
  uint8_t mailboxes = 0;
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining TX (if any) */
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
  }
  for (uint8_t i = mailboxes; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
      if (msg.flags.extended) FLEXCANb_MBn_ID(_baseAddress, i) = (msg.id & FLEXCAN_MB_ID_EXT_MASK);
      else FLEXCANb_MBn_ID(_baseAddress, i) = FLEXCAN_MB_ID_IDSTD(msg.id);
      FLEXCANb_MBn_WORD0(_baseAddress, i) = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      FLEXCANb_MBn_WORD1(_baseAddress, i) = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      uint32_t options = 0;
      if ( msg.flags.remote ) options |= FLEXCAN_MB_CS_RTR;
      if ( msg.flags.extended ) options |= FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR;
      FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE) | FLEXCAN_MB_CS_LENGTH(msg.len) | options;

      return 1; /* transmit entry accepted */
    } // CODE CHECK
  } // FOR LOOP
  if ( retries-- > 0 ) goto retry_tx_flexcan;
  return 0; /* transmit entry failed, no mailboxes available */
}

int IFCT::write(IFCTMBNUM mb_num, const CAN_message_t &msg) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return 0; /* mailbox not available to transmit */
  }
  if ( !((FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mb_num))) >> 3) ) return 0; /* not a transmit mailbox */
  uint32_t timeout = millis();
  while( FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mb_num)) != FLEXCAN_MB_CODE_TX_INACTIVE ) {
    if ( millis() - timeout > 100 ) return 0; /* we exit out on a timeout */
  }
  if (msg.flags.extended) FLEXCANb_MBn_ID(_baseAddress, mb_num) = (msg.id & FLEXCAN_MB_ID_EXT_MASK);
  else FLEXCANb_MBn_ID(_baseAddress, mb_num) = FLEXCAN_MB_ID_IDSTD(msg.id);
  FLEXCANb_MBn_WORD0(_baseAddress, mb_num) = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
  FLEXCANb_MBn_WORD1(_baseAddress, mb_num) = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

  uint32_t options = 0;
  if ( msg.flags.remote ) options |= FLEXCAN_MB_CS_RTR;
  if ( msg.flags.extended ) options |= FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR;
  FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE) | FLEXCAN_MB_CS_LENGTH(msg.len) | options;

  return 1; // transmit entry accepted //
}



int IFCT::read(CAN_message_t &msg) {
  static uint8_t mailbox_check = 0; /* incremental counter we use to sweep accross mailboxes evenly every call */
  uint8_t cycle_count = 0;
rescan_flexcan_rx_mailboxes:
  uint32_t status = FLEXCANb_IFLAG1(_baseAddress), timeout = millis();

  if ( !mailbox_check && FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining RX (if any) */
    /* run once only when position resets back to 0 if FIFO is enabled */ 
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailbox_check = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;

    /* if FIFO interrupt not enabled, we return at least 1 message while sequentially gathering mailboxes evenly */
    if (!(FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IMASK1_BUF5M) && (FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I) ) {
      msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_baseAddress, 0));
      msg.flags.extended = (FLEXCANb_MBn_CS(_baseAddress, 0) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
      msg.flags.remote = (FLEXCANb_MBn_CS(_baseAddress, 0) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
      msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_baseAddress, 0));
      msg.id = (FLEXCANb_MBn_ID(_baseAddress, 0) & FLEXCAN_MB_ID_EXT_MASK);
      if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
      uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, 0);
      msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
      dataIn = FLEXCANb_MBn_WORD1(_baseAddress, 0);
      msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
      FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF5I; /* clear FIFO bit only! */
      if ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF6I ) FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF6I;
      if ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF7I ) FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF7I;
      return 1;
    }
  }
  while (!(status & (1 << mailbox_check))) {
    if ( ++mailbox_check > 15 ) {
      if (FLEXCANb_IMASK1(_baseAddress) & (1 << mailbox_check)) continue; /* if this is an interrupt enabled mailbox, skip it */
      mailbox_check = 0U; // skip mailboxes that haven't triggered an interrupt
      if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) {  /* FIFO is enabled, get only remaining RX (if any) */
        uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
        if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
        mailbox_check = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
      }
    }
    if ( millis() - timeout > 100 ) return 0; /* no frames available */
  }

  uint32_t code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mailbox_check)); // Reading Control Status atomically locks mailbox.
  switch ( code ) {
    case FLEXCAN_MB_CODE_RX_FULL:           // rx full, Copy the frame to RX buffer
    case FLEXCAN_MB_CODE_RX_OVERRUN: {      // rx overrun. Incomming frame overwrote existing frame.
        msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_baseAddress, mailbox_check));
        msg.flags.extended = (FLEXCANb_MBn_CS(_baseAddress, mailbox_check) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
        msg.flags.remote = (FLEXCANb_MBn_CS(_baseAddress, mailbox_check) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
        msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_baseAddress, mailbox_check));
        msg.id = (FLEXCANb_MBn_ID(_baseAddress, mailbox_check) & FLEXCAN_MB_ID_EXT_MASK);
        if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
        uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, mailbox_check);
        msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
        dataIn = FLEXCANb_MBn_WORD1(_baseAddress, mailbox_check);
        msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
        msg.mb = mailbox_check; /* store the mailbox the message came from (for callback reference) */
        if (!msg.flags.extended) FLEXCANb_MBn_CS(_baseAddress, mailbox_check) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
        else FLEXCANb_MBn_CS(_baseAddress, mailbox_check) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
        FLEXCANb_TIMER(_baseAddress); // reading timer unlocks individual mailbox
        FLEXCANb_IFLAG1(_baseAddress) = (1 << mailbox_check); /* immediately flush interrupt of current mailbox */
        return 1; /* we got a frame, exit */
        break;
      }
    case FLEXCAN_MB_CODE_TX_INACTIVE: {       // TX inactive. Just chillin' waiting for a message to send.
        FLEXCANb_IFLAG1(_baseAddress) = (1 << mailbox_check); /* immediately flush interrupt of current mailbox */
        if ( ++mailbox_check > 15 ) mailbox_check = 0U;
        if ( ++cycle_count > 2 ) return 0; /* we skipped over enough TX buffers to check all RX mailboxes */
        goto rescan_flexcan_rx_mailboxes;
      }
    case FLEXCAN_MB_CODE_RX_BUSY:           // mailbox is busy, check it later.
    case FLEXCAN_MB_CODE_RX_INACTIVE:       // inactive Receive box. Must be a false alarm!?
    case FLEXCAN_MB_CODE_RX_EMPTY:          // rx empty already. Why did it interrupt then?
    case FLEXCAN_MB_CODE_TX_ABORT:          // TX being aborted.
    case FLEXCAN_MB_CODE_TX_RESPONSE:       // remote request response (deprecated)
    case FLEXCAN_MB_CODE_TX_ONCE:           // TX mailbox is full and will be sent as soon as possible
    case FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO: // remote request junk again. Go away.
      break;
    default:
      break;
  }
  if ( ++mailbox_check > 15 ) mailbox_check = 0U;
  return 0;
}



void can0_message_isr (void) {
  Can0.IFCT_message_ISR();
}
void can1_message_isr (void) {
  Can1.IFCT_message_ISR();
}


void IFCT::IFCT_message_ISR(void) {
  CAN_message_t msg; // setup a temporary storage buffer
  uint32_t status = FLEXCANb_IFLAG1(_baseAddress);
  if ( (FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IMASK1_BUF5M) && (status & FLEXCAN_IFLAG1_BUF5I) && (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) ) { /* FIFO is enabled, capture frames if triggered */
    while ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I ) {
      if ( FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IMASK1_BUF5M ) {
        msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_baseAddress, 0));
        msg.flags.extended = (FLEXCANb_MBn_CS(_baseAddress, 0) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
        msg.flags.remote = (FLEXCANb_MBn_CS(_baseAddress, 0) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
        msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_baseAddress, 0));
        msg.id = (FLEXCANb_MBn_ID(_baseAddress, 0) & FLEXCAN_MB_ID_EXT_MASK);
        if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
        uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, 0);
        msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
        dataIn = FLEXCANb_MBn_WORD1(_baseAddress, 0);
        msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
        FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF5I; /* clear FIFO bit only! */
        if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(msg);
      }
      FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF5I; /* clear FIFO bit only! */
    }
    if ( status & FLEXCAN_IFLAG1_BUF6I ) {
      FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF6I;
      status &= ~FLEXCAN_IFLAG1_BUF6I; /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
    }
    if ( status & FLEXCAN_IFLAG1_BUF7I ) {
      FLEXCANb_IFLAG1(_baseAddress) = FLEXCAN_IFLAG1_BUF7I;
      status &= ~FLEXCAN_IFLAG1_BUF7I; /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
    }
    status &= ~FLEXCAN_IFLAG1_BUF5I; /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
  }

  /* non FIFO mailbox handling */
  uint8_t mailboxes = 0;
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining RX (if any) */
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
  }
  /* non FIFO mailbox handling complete */
  /* mailbox handling routine */
  for (uint8_t i = mailboxes; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    if (!(status & (1 << i))) continue; // skip mailboxes that haven't triggered an interrupt
    uint32_t code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i)); // Reading Control Status atomically locks mailbox.
    switch ( code ) {
      case FLEXCAN_MB_CODE_RX_FULL:           // rx full, Copy the frame to RX buffer
      case FLEXCAN_MB_CODE_RX_OVERRUN: {      // rx overrun. Incomming frame overwrote existing frame.
          msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_baseAddress, i));
          msg.flags.extended = (FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
          msg.flags.remote = (FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
          msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_baseAddress, i));
          msg.id = (FLEXCANb_MBn_ID(_baseAddress, i) & FLEXCAN_MB_ID_EXT_MASK);
          if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
          uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, i);
          msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
          dataIn = FLEXCANb_MBn_WORD1(_baseAddress, i);
          msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
          msg.mb = i; /* store the mailbox the message came from (for callback reference) */
          if ( _baseAddress == FLEXCAN0_BASE ) msg.bus = 0;
          else if ( _baseAddress == FLEXCAN1_BASE ) msg.bus = 1;

          if ( !can_events ) {
            if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(msg);
            sendMSGtoIndividualMBCallback((IFCTMBNUM)i, msg); /* send frames direct to callback (unbuffered) */
          }
          else struct2queue(msg); /* store frame in queue ( buffered ) */

          if (!msg.flags.extended) FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
          else FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
          FLEXCANb_TIMER(_baseAddress); // reading timer unlocks individual mailbox
          FLEXCANb_IFLAG1(_baseAddress) = (1UL << i); /* immediately flush interrupt of current mailbox */
          status &= ~(1 << i); /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
          break;
        }
      case FLEXCAN_MB_CODE_TX_INACTIVE: {       // TX inactive. Just chillin' waiting for a message to send.
          FLEXCANb_IFLAG1(_baseAddress) = (1UL << i); /* immediately flush interrupt of current mailbox */
          status &= ~(1 << i); /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
          continue; /* skip mailboxes that are TX */
        }
      case FLEXCAN_MB_CODE_RX_BUSY:           // mailbox is busy, check it later.
      case FLEXCAN_MB_CODE_RX_INACTIVE:       // inactive Receive box. Must be a false alarm!?
      case FLEXCAN_MB_CODE_RX_EMPTY:          // rx empty already. Why did it interrupt then?
      case FLEXCAN_MB_CODE_TX_ABORT:          // TX being aborted.
      case FLEXCAN_MB_CODE_TX_RESPONSE:       // remote request response (deprecated)
      case FLEXCAN_MB_CODE_TX_ONCE:           // TX mailbox is full and will be sent as soon as possible
      case FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO: // remote request junk again. Go away.
        break;
      default:
        break;
    }
  }
  FLEXCANb_IFLAG1(_baseAddress) = status; /* essentially, if all bits were cleared above, it's basically writing 0 to IFLAG, to prevent new data interruptions. */
}



void IFCT::struct2queue(const CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  memmove(buf, &msg, sizeof(msg));
  flexcan_buffer.push_back(buf, sizeof(CAN_message_t));
}
void IFCT::queue2struct(CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  flexcan_buffer.pop_front(buf, sizeof(CAN_message_t));
  memmove(&msg, buf, sizeof(msg));
}

void IFCT::events() {
  CAN_message_t frame;
  if ( !can_events ) can_events = 1; /* handle callbacks from loop */

  if ( flexcan_buffer.size() ) { /* if a queue frame is available */
    queue2struct(frame);
    if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(frame);
    sendMSGtoIndividualMBCallback((IFCTMBNUM)frame.mb, frame); 
  }
}




void IFCT::mailboxStatus() {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) {
    Serial.print("FIFO Enabled --> "); ( FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I ) ? Serial.println("Interrupt Enabled") : Serial.println("Interrupt Disabled");
    Serial.print("\tFIFO Filters in use: ");
    Serial.println((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8); // 8 filters per 2 mailboxes
    Serial.print("\tRemaining Mailboxes: ");
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    Serial.println(remaining_mailboxes); // 8 filters per 2 mailboxes
    for ( uint8_t i = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) {
      switch ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i)) ) {
        case 0b0000: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_INACTIVE"); break;
          }
        case 0b0100: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.print(" code: RX_EMPTY");
            (FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE) ? Serial.println("\t(Extended Frame)") : Serial.println("\t(Standard Frame)");
            break;
          }
        case 0b0010: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_FULL"); break;
          }
        case 0b0110: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_OVERRUN"); break;
          }
        case 0b1010: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_RANSWER"); break;
          }
        case 0b0001: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_BUSY"); break;
          }
        case 0b1000: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: TX_INACTIVE"); break;
          }
        case 0b1001: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: TX_ABORT"); break;
          }
        case 0b1100: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.print(" code: TX_DATA (Transmitting)");
            uint32_t extid = (FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE);
            (extid) ? Serial.print("(Extended Frame)") : Serial.print("(Standard Frame)");
            uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, i);
            uint32_t id = (FLEXCANb_MBn_ID(_baseAddress, i) & FLEXCAN_MB_ID_EXT_MASK);
            if (!extid) id >>= FLEXCAN_MB_ID_STD_BIT_NO;
            Serial.print("(ID: 0x"); Serial.print(id, HEX); Serial.print(")");
            Serial.print("(Payload: "); Serial.print((uint8_t)(dataIn >> 24), HEX);
            Serial.print(" "); Serial.print((uint8_t)(dataIn >> 16), HEX);
            Serial.print(" "); Serial.print((uint8_t)(dataIn >> 8), HEX);
            Serial.print(" "); Serial.print((uint8_t)dataIn, HEX);
            dataIn = FLEXCANb_MBn_WORD1(_baseAddress, i);
            Serial.print(" "); Serial.print((uint8_t)(dataIn >> 24), HEX);
            Serial.print(" "); Serial.print((uint8_t)(dataIn >> 16), HEX);
            Serial.print(" "); Serial.print((uint8_t)(dataIn >> 8), HEX);
            Serial.print(" "); Serial.print((uint8_t)dataIn, HEX);
            Serial.println(")");
            break;
          }
        case 0b1110: {
            Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: TX_TANSWER"); break;
          }
      }
    } // for loop
    return;
  } // fifo detected ends here
  Serial.print("FIFO Disabled\n\tMailboxes:\n");
  for ( uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) {
    switch ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i)) ) {
      case 0b0000: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_INACTIVE"); break;
        }
      case 0b0100: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.print(" code: RX_EMPTY");
          (FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE) ? Serial.println("\t(Extended Frame)") : Serial.println("\t(Standard Frame)");
          break;
        }
      case 0b0010: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_FULL"); break;
        }
      case 0b0110: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_OVERRUN"); break;
        }
      case 0b1010: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_RANSWER"); break;
        }
      case 0b0001: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: RX_BUSY"); break;
        }
      case 0b1000: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: TX_INACTIVE"); break;
        }
      case 0b1001: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: TX_ABORT"); break;
        }
      case 0b1100: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.print(" code: TX_DATA (Transmitting)");
          uint32_t extid = (FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE);
          (extid) ? Serial.print("(Extended Frame)") : Serial.print("(Standard Frame)");
          uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, i);
          uint32_t id = (FLEXCANb_MBn_ID(_baseAddress, i) & FLEXCAN_MB_ID_EXT_MASK);
          if (!extid) id >>= FLEXCAN_MB_ID_STD_BIT_NO;
          Serial.print("(ID: 0x"); Serial.print(id, HEX); Serial.print(")");
          Serial.print("(Payload: "); Serial.print((uint8_t)(dataIn >> 24), HEX);
          Serial.print(" "); Serial.print((uint8_t)(dataIn >> 16), HEX);
          Serial.print(" "); Serial.print((uint8_t)(dataIn >> 8), HEX);
          Serial.print(" "); Serial.print((uint8_t)dataIn, HEX);
          dataIn = FLEXCANb_MBn_WORD1(_baseAddress, i);
          Serial.print(" "); Serial.print((uint8_t)(dataIn >> 24), HEX);
          Serial.print(" "); Serial.print((uint8_t)(dataIn >> 16), HEX);
          Serial.print(" "); Serial.print((uint8_t)(dataIn >> 8), HEX);
          Serial.print(" "); Serial.print((uint8_t)dataIn, HEX);
          Serial.println(")");
          break;
        }
      case 0b1110: {
          Serial.print("\t\tMB"); Serial.print(i); Serial.println(" code: TX_TANSWER"); break;
        }
    }
  } // for loop
}


void IFCT::setTX(IFCTALTPIN which) {

#if defined(__MK20DX256__)
  CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
#endif

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( _baseAddress == FLEXCAN0_BASE ) {
    if ( which == ALT ) {
      CORE_PIN3_CONFIG = 0; CORE_PIN29_CONFIG = PORT_PCR_MUX(2);
    }
    else if ( which == DEF ) {
      CORE_PIN29_CONFIG = 0; CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
    }
  } /* Alternative CAN1 pins are not broken out on Teensy 3.6 */
#endif

#if defined(__MK66FX1M0__)
  if ( _baseAddress == FLEXCAN1_BASE ) {
      CORE_PIN33_CONFIG = PORT_PCR_MUX(2);
  }
#endif

}




void IFCT::setRX(IFCTALTPIN which) {

#if defined(__MK20DX256__)
  CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
#endif

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( _baseAddress == FLEXCAN0_BASE ) {
    if ( which == ALT ) {
      CORE_PIN4_CONFIG = 0; CORE_PIN30_CONFIG = PORT_PCR_MUX(2);
    }
    else if ( which == DEF ) {
      CORE_PIN30_CONFIG = 0; CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
    }
  } /* Alternative CAN1 pins are not broken out on Teensy 3.6 */
#endif

#if defined(__MK66FX1M0__)
  if ( _baseAddress == FLEXCAN1_BASE ) {
      CORE_PIN34_CONFIG = PORT_PCR_MUX(2);
  }
#endif

}



void IFCT::setMBFilter(IFCTMBFLTEN input) {
  uint8_t mailboxes = 0;
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining TX (if any) */
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
  }
  FLEXCAN_EnterFreezeMode();
  for (uint8_t i = mailboxes; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    if ( input == ACCEPT_ALL ) FLEXCANb_MB_MASK(_baseAddress, i) = 0x00000000; // (RXIMR)
    if ( input == REJECT_ALL ) FLEXCANb_MB_MASK(_baseAddress, i) = 0x3FFFFFFF; // (RXIMR)
    FLEXCANb_MBn_ID(_baseAddress, i) = 0x00000000;
  }
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setMBFilter(IFCTMBNUM mb_num, IFCTMBFLTEN input) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return; /* mailbox not available */
  }

  FLEXCAN_EnterFreezeMode();
  if ( input == ACCEPT_ALL ) FLEXCANb_MB_MASK(_baseAddress, mb_num) = 0x00000000; // (RXIMR)
  if ( input == REJECT_ALL ) FLEXCANb_MB_MASK(_baseAddress, mb_num) = 0x3FFFFFFF; // (RXIMR)
  FLEXCAN_ExitFreezeMode();
  FLEXCANb_MBn_ID(_baseAddress, mb_num) = 0x00000000;
}



void IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return; /* mailbox not available to be set */
  }
  uint32_t mask = 0;
  if (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) mask = ((((id1) ^ (id1)) ^ 0xFFFFFFFF) << 18 );
  else mask = ((((id1) ^ (id1)) ^ 0xFFFFFFFF) << 0 );
  setMBFilterProcessing(mb_num,id1,mask);
}

void IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return; /* mailbox not available to be set */
  }
  uint32_t mask = 0;
  if (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) mask = ((((id1 | id2) ^ (id1 & id2)) ^ 0xFFFFFFFF) << 18 );
  else mask = ((((id1 | id2) ^ (id1 & id2)) ^ 0xFFFFFFFF) << 0 );
  setMBFilterProcessing(mb_num,id1,mask);
}

void IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return; /* mailbox not available to be set */
  }
  uint32_t mask = 0;
  if (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) mask = ((((id1 | id2 | id3) ^ (id1 & id2 & id3)) ^ 0xFFFFFFFF) << 18 );
  else mask = ((((id1 | id2 | id3) ^ (id1 & id2 & id3)) ^ 0xFFFFFFFF) << 0 );
  setMBFilterProcessing(mb_num,id1,mask);
}

void IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return; /* mailbox not available to be set */
  }
  uint32_t mask = 0;
  if (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) mask = ((((id1 | id2 | id3 | id4) ^ (id1 & id2 & id3 & id4)) ^ 0xFFFFFFFF) << 18 );
  else mask = ((((id1 | id2 | id3 | id4) ^ (id1 & id2 & id3 & id4)) ^ 0xFFFFFFFF) << 0 );
  setMBFilterProcessing(mb_num,id1,mask);
}

void IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t id5) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return; /* mailbox not available to be set */
  }
  uint32_t mask = 0;
  if (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) mask = ((((id1 | id2 | id3 | id4 | id5) ^ (id1 & id2 & id3 & id4 & id5)) ^ 0xFFFFFFFF) << 18 );
  else mask = ((((id1 | id2 | id3 | id4 | id5) ^ (id1 & id2 & id3 & id4 & id5)) ^ 0xFFFFFFFF) << 0 );
  setMBFilterProcessing(mb_num,id1,mask);
}

void IFCT::setMBFilterRange(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) { /* FIFO is enabled, get only remaining MBs */
    uint8_t mailboxes = 0;
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes;
    if ( mb_num < mailboxes ) return; /* mailbox not available to be set */
  }

  if ( id1 > id2 || ((id2 > id1) && (id2-id1>1000)) || !id1 || !id2 ) return; /* don't play around... */

  uint32_t mask = 0, stage1 = id1, stage2 = id1;
  for ( uint16_t i = id1 + 1; i <= id2; i++ ) {
    stage1 |= i; stage2 &= i;
  }

  uint32_t _calc = ((stage1 ^ stage2) ^ 0xFFFFFFFF);
  (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) ? mask = _calc << 18 : mask = _calc << 0;
  setMBFilterProcessing(mb_num,id1,mask);
}

void IFCT::setMBFilterProcessing(IFCTMBNUM mb_num, uint32_t filter_id, uint32_t calculated_mask) {
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_MB_MASK(_baseAddress, mb_num) = calculated_mask;
  FLEXCAN_ExitFreezeMode();
  if (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) FLEXCANb_MBn_ID(_baseAddress, mb_num) = FLEXCAN_MB_ID_IDSTD(filter_id);
  else FLEXCANb_MBn_ID(_baseAddress, mb_num) = FLEXCAN_MB_ID_IDEXT(filter_id);
}



void IFCT::setFIFOFilter(const IFCTMBFLTEN &input) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  FLEXCAN_EnterFreezeMode();
  for (uint8_t i = 0; i < 8; i++) { /* block all ID's so filtering could be applied. */
    if ( input == REJECT_ALL ) { /* Tables A & B */
      FLEXCANb_IDFLT_TAB(_baseAddress, i) = 0xFFFFFFFF; /* reset id */
      FLEXCANb_MB_MASK(_baseAddress, i) = 0x3FFFFFFF; // (RXIMR) /* block all id's */

      if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 2 ) { /* If Table C is chosen for FIFO */
        FLEXCANb_IDFLT_TAB(_baseAddress, i) = 0x6E6E6E6E; /* reset id */
        FLEXCANb_MB_MASK(_baseAddress, i) = 0xFFFFFFFF; // (RXIMR) /* block all id's */
      }

    }
    else if ( input == ACCEPT_ALL ) {
      FLEXCANb_IDFLT_TAB(_baseAddress, i) = 0; /* reset id */
      FLEXCANb_MB_MASK(_baseAddress, i) = 0; // (RXIMR) /* allow all id's */
    }
  }
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, const IFCTMBIDE &ide, const IFCTMBIDE &remote) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 0 ) return; /* must be TableA to process */
  FLEXCAN_EnterFreezeMode();
  uint32_t mask = 0;
  ( ide != EXT ) ? (mask = ((((id1) ^ (id1)) ^ 0xFFFFFFFF) << 19 )) : (mask = ((((id1) ^ (id1)) ^ 0xFFFFFFFF) << 1 ));
  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
      ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide, const IFCTMBIDE &remote) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 0 ) return; /* must be TableA to process */
  FLEXCAN_EnterFreezeMode();
  uint32_t mask = 0;
  ( ide != EXT ) ? (mask = ((((id1 | id2) ^ (id1 & id2)) ^ 0xFFFFFFFF) << 19 )) : (mask = ((((id1 | id2) ^ (id1 & id2)) ^ 0xFFFFFFFF) << 1 ));
  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
      ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide, const IFCTMBIDE &remote) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 0 ) return; /* must be TableA to process */
  if ( id1 > id2 || ((id2 > id1) && (id2 - id1 > 1000)) || !id1 || !id2 ) return; /* don't play around... */
  FLEXCAN_EnterFreezeMode();
  uint32_t mask = 0, stage1 = id1, stage2 = id1;
  for ( uint16_t i = id1 + 1; i <= id2; i++ ) {
    stage1 |= i; stage2 &= i;
  }
  uint32_t _calc = ((stage1 ^ stage2) ^ 0xFFFFFFFF);
  ( ide != EXT ) ? mask = _calc << 19 : mask = _calc << 1;
  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
      ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  FLEXCAN_ExitFreezeMode();
}


void IFCT::setFIFOFilterTable(IFCTFIFOTABLE letter) {
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_IDAM(letter);
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, const IFCTMBIDE &ide1, const IFCTMBIDE &remote1, uint32_t id2, const IFCTMBIDE &ide2, const IFCTMBIDE &remote2) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 1 ) return; /* must be TableB to process */
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
      ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
      (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
      (ide2 == EXT ? (id2 >> (29 - 14)) : ((id2 & 0x7FF) << 3))  ; /* second ID is EXT or STD? */
  FLEXCANb_MB_MASK(_baseAddress, filter) = 0xFFFFFFFF; // (RXIMR)
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide1, const IFCTMBIDE &remote1, uint32_t id3, uint32_t id4, const IFCTMBIDE &ide2, const IFCTMBIDE &remote2) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 1 ) return; /* must be TableB to process */
  FLEXCAN_EnterFreezeMode();
  uint32_t mask = 0;

  if ( ide1 != EXT ) mask = (((((id1 | id2) ^ (id1 & id2)) ^ 0xFFFFFFFF) << 19 ) & 0x3FF80000) | 0xC0070000;
  else mask = (((((id1 | id2) ^ (id1 & id2)) ^ 0xFFFFFFFF) << 16 ) & 0x3FFF0000) | 0xC0000000;

  if ( ide2 != EXT ) mask |= (((((id3 | id4) ^ (id3 & id4)) ^ 0xFFFFFFFF) << 3 ) & 0x3FF8) | 0xC007;
  else mask |= (((((id3 | id4) ^ (id3 & id4)) ^ 0xFFFFFFFF) << 0 ) & 0x3FFF) | 0xC000;

  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
      ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
      (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
      (ide2 == EXT ? (id3 >> (29 - 14)) : ((id3 & 0x7FF) << 3))  ; /* second ID is EXT or STD? */
  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide1, const IFCTMBIDE &remote1, uint32_t id3, uint32_t id4, const IFCTMBIDE &ide2, const IFCTMBIDE &remote2) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 1 ) return; /* must be TableB to process */
  if ( id1 > id2 || ((id2 > id1) && (id2 - id1 > 1000)) || !id1 || !id2 ) return; /* don't play around... */
  if ( id3 > id4 || ((id4 > id3) && (id4 - id3 > 1000)) || !id3 || !id4 ) return; /* don't play around... */
  FLEXCAN_EnterFreezeMode();
  uint32_t mask = 0, stage1 = id1, stage2 = id1;
  if ( ide1 != EXT ) {
    for ( uint16_t i = id1 + 1; i <= id2; i++ ) {
      stage1 |= i; stage2 &= i;
    }
  }
  else {
    for ( uint16_t i = ( id1 >> (29 - 14)) + 1; i <= ( id2 >> (29 - 14)); i++ ) {
      stage1 |= i; stage2 &= i;
    }
  }

  uint32_t _IDset1 = ((stage1 ^ stage2) ^ 0xFFFFFFFF);

  stage1 = stage2 = id3;
  if ( ide2 != EXT ) {
    for ( uint16_t i = id3 + 1; i <= id4; i++ ) {
      stage1 |= i; stage2 &= i;
    }
  }
  else {
    for ( uint16_t i = ( id3 >> (29 - 14)) + 1; i <= ( id4 >> (29 - 14)); i++ ) {
      stage1 |= i; stage2 &= i;
    }
  }

  uint32_t _IDset2 = ((stage1 ^ stage2) ^ 0xFFFFFFFF);

  ( ide1 != EXT ) ? (mask = ((  _IDset1 << 19 ) & 0x3FF80000) | 0xC0070000) : (mask = (( _IDset1 << 16 ) & 0x3FFF0000) | 0xC0000000);

  ( ide2 != EXT ) ? (mask |= (( _IDset2 << 3 ) & 0x3FF8) | 0xC007) : (mask |= (( _IDset2 << 0 ) & 0x3FFF) | 0xC000);

  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
      ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
      (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
      (ide2 == EXT ? (id3 >> (29 - 14)) : ((id3 & 0x7FF) << 3))  ; /* second ID is EXT or STD? */
  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  FLEXCAN_ExitFreezeMode();
}


void IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4 ) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 2 ) return; /* must be TableC to process */
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) =
    (( id1 > 0x7FF ) ? ((id1 >> (29 - 8)) << 24) : ((id1 >> (11 - 8)) << 24)) |
    (( id2 > 0x7FF ) ? ((id2 >> (29 - 8)) << 16) : ((id2 >> (11 - 8)) << 16)) |
    (( id3 > 0x7FF ) ? ((id3 >> (29 - 8)) << 8) : ((id3 >> (11 - 8)) << 8)) |
    (( id4 > 0x7FF ) ? ((id4 >> (29 - 8)) << 0) : ((id4 >> (11 - 8)) << 0));
  FLEXCANb_MB_MASK(_baseAddress, filter) = 0xFFFFFFFF; // (RXIMR)
  FLEXCAN_ExitFreezeMode();
}
