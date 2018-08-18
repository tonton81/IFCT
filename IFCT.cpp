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
IntervalTimer CAN_timer[2];
#include "TeensyThreads.h"
Threads::Mutex CAN_THREAD[2];



Circular_Buffer<uint8_t, FLEXCAN_BUFFER_SIZE, sizeof(CAN_message_t)> IFCT::flexcan_buffer;
bool IFCT::can_events = 0;

_MB_ptr IFCT::_MBhandlers[16] = { nullptr };
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
  FLEXCANb_CTRL1(_baseAddress) |= FLEXCAN_CTRL_LOM; /* listen only mode enable */
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_FRZ; /* enable freeze bit */
  FLEXCANb_MCR(_baseAddress) &= ~FLEXCAN_MCR_MDIS; /* enable module */

  while (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_LPM_ACK);
  softReset(); /* reset bus */
  while (!(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK));
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_SRX_DIS; /* Disable self-reception */

  disableFIFO();
  disableFIFOInterrupt();

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

void IFCT::softResetRestore() {
  /*
    Soft Reset
    The following registers are reset: MCR (except the MDIS bit), TIMER , ECR, ESR1, ESR2,
    IMASK1, IMASK2, IFLAG1, IFLAG2 and CRCR. Configuration registers that control the interface to the
    CAN bus are not affected by soft reset.
    The following registers are unaffected: CTRL1, CTRL2, all RXIMR
    registers, RXMGMASK, RX14MASK, RX15MASK, RXFGMASK, RXFIR, all Message Buffers .
  */

  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }

  uint32_t mcr = FLEXCANb_MCR(_baseAddress);
  uint32_t imask1 = FLEXCANb_IMASK1(_baseAddress);

  FLEXCANb_MCR(_baseAddress) ^= FLEXCAN_MCR_SOFT_RST;
  while (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_SOFT_RST);

  FLEXCANb_MCR(_baseAddress) = mcr;
  FLEXCANb_IMASK1(_baseAddress) = imask1;
  FLEXCANb_IFLAG1(_baseAddress) = FLEXCANb_IFLAG1(_baseAddress);

  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
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
    /*
      Each group of eight filters occupies a memory space equivalent to two Message Buffers which
      means that the more filters are implemented the less Mailboxes will be available.
    */
    FLEXCAN_set_rffn(FLEXCANb_CTRL2(_baseAddress), 0); // setup 8 Filters for FIFO, 0-5 = FIFO, 6-7 FILTER, 8-15 MBs, max value 0x3 which leaves MB14/15 free to use.
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
        else FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR;
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
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) ) return; /* FIFO must be enabled first */

  if ( FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I ) return; /* FIFO interrupts already enabled */

  ( !status ) ? FLEXCANb_IMASK1(_baseAddress) &= ~FLEXCAN_IMASK1_BUF5M /* disable FIFO interrupt */ : FLEXCANb_IMASK1(_baseAddress) |= FLEXCAN_IMASK1_BUF5M; /* enable FIFO interrupt */
}


void IFCT::disableFIFOInterrupt() {
  enableFIFOInterrupt(0);
}


void IFCT::setBaudRate(uint32_t baud) {
  currentBitrate = baud;
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
  FLEXCANb_CTRL1(_baseAddress) &= ~FLEXCAN_CTRL_LOM; /* disable listen-only mode */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}


bool IFCT::setMB(const IFCTMBNUM &mb_num, const IFCTMBTXRX &mb_rx_tx, const IFCTMBIDE &ide) {

  if ( mb_num < mailboxOffset() ) return 0; /* mailbox not available to transmit */

  FLEXCANb_IMASK1(_baseAddress) &= ~(1UL << mb_num); /* immediately disable mailbox interrupt */
  FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mb_num)); // Reading Control Status atomically locks mailbox (if it is RX mode).
  FLEXCANb_MBn_ID(_baseAddress, mb_num) = 0x00000000;
  FLEXCANb_MBn_WORD0(_baseAddress, mb_num) = 0x00000000;
  FLEXCANb_MBn_WORD1(_baseAddress, mb_num) = 0x00000000;

  if ( mb_rx_tx == RX ) {
    if ( ide != EXT ) FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
    else FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }

  if ( mb_rx_tx == TX ) FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);

  FLEXCANb_TIMER(_baseAddress); /* reading timer unlocks individual mailbox */
  FLEXCANb_IFLAG1(_baseAddress) = ( 1 << mb_num ); /* clear mailbox reception flag */
  return 1;
}



void IFCT::enableMBInterrupt(const IFCTMBNUM &mb_num, bool status) {
  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) {
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB(16) - FIFO(6) */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    if ( mb_num < ( FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes ) ) return; /* Mailbox not available */
  }
  if ( status ) FLEXCANb_IMASK1(_baseAddress) |= (1UL << mb_num); /* enable mailbox interrupt */
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

  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) fifo_was_cleared = 1; /* let fifo clear current mailboxes */
  disableFIFO();
  
  FLEXCANb_IFLAG1(_baseAddress) = FLEXCANb_IFLAG1(_baseAddress); // (all bits reset when written back) (needed for MAXMB changes)
  FLEXCANb_MCR(_baseAddress) &= ~FLEXCAN_MCR_MAXMB_MASK; // disable mailboxes
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_MAXMB(last); // set mailbox max

  if ( fifo_was_cleared ) enableFIFO();

  FLEXCAN_ExitFreezeMode();
}


void IFCT::FLEXCAN_EnterFreezeMode() {
  /*
    in order to prevent lockups when entering freeze mode, datasheet states:

    NOTE:
      FRZACK will be asserted within 178 CAN bits from the freeze mode request by the CPU, and
      negated within 2 CAN bits after the freeze mode request removal (see Section "Protocol Timing").

    If there is an issue on the network or even running in single node mode, entering the FRZ state can hard lockup
    the processor, as the while loop will never exit. Rather than resetting the flexcan controller and setting up mcr
    and interrupt registers to deal with this, we simply disable the pins temporarily which gives it immediate access
    to freeze mode without any hard lockups.
  */

  uint8_t rx = 0, tx = 0;
  uint32_t pinBackupRx = 0, pinBackupTx = 0;

  if ( ( CORE_PIN3_CONFIG & 0x700 ) == 0x200 ) {
    tx = 3; pinBackupTx = CORE_PIN3_CONFIG; CORE_PIN3_CONFIG = 0;
  }
  else if ( ( CORE_PIN29_CONFIG & 0x700 ) == 0x200 ) {
    tx = 29; pinBackupTx = CORE_PIN29_CONFIG; CORE_PIN29_CONFIG = 0;
  }
  else if ( ( CORE_PIN33_CONFIG & 0x700 ) == 0x200 ) {
    tx = 33; pinBackupTx = CORE_PIN33_CONFIG; CORE_PIN33_CONFIG = 0;
  }

  if ( ( CORE_PIN4_CONFIG & 0x700 ) == 0x200 ) {
    rx = 4; pinBackupRx = CORE_PIN4_CONFIG; CORE_PIN4_CONFIG = 0;
  }
  else if ( ( CORE_PIN30_CONFIG & 0x700 ) == 0x200 ) {
    rx = 30; pinBackupRx = CORE_PIN30_CONFIG; CORE_PIN30_CONFIG = 0;
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else if ( ( CORE_PIN34_CONFIG & 0x700 ) == 0x200 ) {
    rx = 34; pinBackupRx = CORE_PIN34_CONFIG; CORE_PIN34_CONFIG = 0;
  }
#endif

  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_HALT;
  while (!(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FRZ_ACK));

  if ( tx == 3 ) CORE_PIN3_CONFIG = pinBackupTx;
  else if ( tx == 29 ) CORE_PIN29_CONFIG = pinBackupTx;
  else if ( tx == 33 ) CORE_PIN33_CONFIG = pinBackupTx;

  if ( rx == 4 ) CORE_PIN4_CONFIG = pinBackupRx;
  else if ( rx == 30 ) CORE_PIN30_CONFIG = pinBackupRx;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else if ( rx == 34 ) CORE_PIN34_CONFIG = pinBackupRx;
#endif
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
  IFCT::_MBhandlers[mb_num] = handler;
}


void IFCT::onReceive(_MB_ptr handler) {
  IFCT::_MBAllhandler = handler;
}


void sendMSGtoIndividualMBCallback(const IFCTMBNUM &mb_num, const CAN_message_t &msg) { /* this is global for ISR use */
  if ( IFCT::_MBhandlers[mb_num] != nullptr ) IFCT::_MBhandlers[mb_num](msg);
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

  uint8_t retries = 3;

  while ( retries ) {
    retries--;

    for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
      if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
        FLEXCANb_IFLAG1(_baseAddress) |= (1 << i); // 1st step clear flag in case it's set as per datasheet
        if (msg.flags.extended) FLEXCANb_MBn_ID(_baseAddress, i) = (msg.id & FLEXCAN_MB_ID_EXT_MASK);
        else FLEXCANb_MBn_ID(_baseAddress, i) = FLEXCAN_MB_ID_IDSTD(msg.id);
        FLEXCANb_MBn_WORD0(_baseAddress, i) = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
        FLEXCANb_MBn_WORD1(_baseAddress, i) = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

        FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE) |
                                            FLEXCAN_MB_CS_LENGTH(msg.len) |
                                            ((msg.flags.remote) ? FLEXCAN_MB_CS_RTR : 0UL) |
                                            ((msg.flags.extended) ? FLEXCAN_MB_CS_IDE : 0UL) |
                                            ((msg.flags.extended) ? FLEXCAN_MB_CS_SRR : 0UL);

        return 1; /* transmit entry accepted */
      } // CODE CHECK
    } // FOR LOOP
    delay(25);
  } // RETRY LOOP

  return 0; /* transmit entry failed, no mailboxes available */
}

int IFCT::write(IFCTMBNUM mb_num, const CAN_message_t &msg) {

  if ( mb_num < mailboxOffset() ) return 0; /* mailbox not available to transmit */

  if ( !((FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mb_num))) >> 3) ) return 0; /* not a transmit mailbox */
  uint32_t timeout = millis();
  while( FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mb_num)) != FLEXCAN_MB_CODE_TX_INACTIVE ) {
    if ( millis() - timeout > 100 ) return 0; /* we exit out on a timeout */
  }
  FLEXCANb_IFLAG1(_baseAddress) |= (1 << mb_num); // 1st step clear flag in case it's set as per datasheet
  if (msg.flags.extended) FLEXCANb_MBn_ID(_baseAddress, mb_num) = (msg.id & FLEXCAN_MB_ID_EXT_MASK);
  else FLEXCANb_MBn_ID(_baseAddress, mb_num) = FLEXCAN_MB_ID_IDSTD(msg.id);
  FLEXCANb_MBn_WORD0(_baseAddress, mb_num) = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
  FLEXCANb_MBn_WORD1(_baseAddress, mb_num) = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

  FLEXCANb_MBn_CS(_baseAddress, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE) |
                                      FLEXCAN_MB_CS_LENGTH(msg.len) |
                                      ((msg.flags.remote) ? FLEXCAN_MB_CS_RTR : 0UL) |
                                      ((msg.flags.extended) ? FLEXCAN_MB_CS_IDE : 0UL) |
                                      ((msg.flags.extended) ? FLEXCAN_MB_CS_SRR : 0UL);

  return 1; // transmit entry accepted //
}




int IFCT::readMB(CAN_message_t &msg) {
  uint8_t cycle_count = 0;

rescan_rx_mbs:

  if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) {  /* FIFO is enabled, get only remaining RX (if any) */
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    if ( mailbox_reader_increment < ( FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes ) )
      mailbox_reader_increment = FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes -1;
  }

  if ( ++mailbox_reader_increment >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) {
    mailbox_reader_increment = 0;
    if ( ++cycle_count > FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* if cycles are greater than number of mailboxes */
    if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) goto rescan_rx_mbs; /* FIFO enabled? offset mailbox.. */
  }

  if (FLEXCANb_IMASK1(_baseAddress) & (1 << mailbox_reader_increment)) { /* don't read interrupt enabled mailboxes */
    if ( ++cycle_count > FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* if cycles are greater than number of mailboxes */
    goto rescan_rx_mbs;
  }

  uint32_t code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment)); // Reading Control Status atomically locks mailbox.
  switch ( code ) {
    case FLEXCAN_MB_CODE_RX_FULL:           // rx full, Copy the frame to RX buffer
    case FLEXCAN_MB_CODE_RX_OVERRUN: {      // rx overrun. Incomming frame overwrote existing frame.
        FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment) |= FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_INACTIVE); /* deactivate mailbox */
        msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment));
        msg.flags.extended = (FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
        msg.flags.remote = (FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
        msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment));
        msg.id = (FLEXCANb_MBn_ID(_baseAddress, mailbox_reader_increment) & FLEXCAN_MB_ID_EXT_MASK);
        if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
        uint32_t dataIn = FLEXCANb_MBn_WORD0(_baseAddress, mailbox_reader_increment);
        msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
        dataIn = FLEXCANb_MBn_WORD1(_baseAddress, mailbox_reader_increment);
        msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
        msg.mb = mailbox_reader_increment; /* store the mailbox the message came from (for callback reference) */
        if (!msg.flags.extended) FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
        else FLEXCANb_MBn_CS(_baseAddress, mailbox_reader_increment) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
        FLEXCANb_TIMER(_baseAddress); // reading timer unlocks individual mailbox
        FLEXCANb_IFLAG1(_baseAddress) = (1 << mailbox_reader_increment); /* immediately flush interrupt of current mailbox */
        if ( filter_enhancement[mailbox_reader_increment][0] ) { /* filter enhancement (if enabled) */
          if ( !filter_enhancement[mailbox_reader_increment][1] ) { /* multi-ID based filter enhancement */
            for ( uint8_t i = 0; i < 5; i++ ) {
              if ( msg.id == filter_enhancement_config[mailbox_reader_increment][i] ) return 1;
            }
          }
          else { /* range based ID filtering enhancement */
            if ( (msg.id >= filter_enhancement_config[mailbox_reader_increment][0]) &&
                 (msg.id <= filter_enhancement_config[mailbox_reader_increment][1]) ) return 1;
          }
          goto rescan_rx_mbs; /* enhanced filtering match not found, check for another frame */
        }
        return 1; /* we got a frame, exit */
      }
    case FLEXCAN_MB_CODE_TX_INACTIVE: {       // TX inactive. Just chillin' waiting for a message to send.
        goto rescan_rx_mbs;
      }
    case FLEXCAN_MB_CODE_RX_BUSY:           // mailbox is busy, check it later.
    case FLEXCAN_MB_CODE_RX_INACTIVE:       // inactive Receive box. Must be a false alarm!?
    case FLEXCAN_MB_CODE_RX_EMPTY:          // rx empty already. Why did it interrupt then?
    case FLEXCAN_MB_CODE_TX_ABORT:          // TX being aborted.
    case FLEXCAN_MB_CODE_TX_RESPONSE:       // remote request response (deprecated)
    case FLEXCAN_MB_CODE_TX_ONCE:           // TX mailbox is full and will be sent as soon as possible
    case FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO: // remote request junk again. Go away.
    default:
      break;
  }
  return 0; /* no messages available */
}

int IFCT::readFIFO(CAN_message_t &msg) {
  if ( FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IMASK1_BUF5M ) return 0; /* FIFO interrupt enabled, manual read blocked */
  if ( ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) && ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I ) ) {
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

    bool enhance_filtering_success = 0;
    uint8_t mailboxes = mailboxOffset();

    for ( uint8_t i = 0; i < mailboxes; i++ ) {
      if ( filter_enhancement[i][0] && filter_set[i] ) { /* if enhancement is active and set */
        if ( !filter_enhancement[i][1] ) { /* if it's multi ID */
          for ( uint8_t j = 0; j < 5; j++ ) {
            if ( msg.id == filter_enhancement_config[i][j] ) {
              enhance_filtering_success = 1;
              break;
            }
          }
        }
        else { /* if it's range based */
          if ( (msg.id >= filter_enhancement_config[i][0]) &&
               (msg.id <= filter_enhancement_config[i][1]) ) enhance_filtering_success = 1;
        }
      }
    }
    if ( ( !enhance_filtering_success && filter_enhancement[0][0] ) ) return 0; /* did not pass enhancement filter */

    return 1;
  }
  return 0;
}




int IFCT::read(CAN_message_t &msg) {

  if ( flexcan_library_emulation ) {
    if ( flexcan_library.size() ) {
      flexcan_library_queue2struct(msg);
      return 1;
    }
    return 0;
  }

  bool _random = random(0, 2);
  if ( ( !_random ) && ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) &&
       !( FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IMASK1_BUF5M ) &&
       ( FLEXCANb_IFLAG1(_baseAddress) & FLEXCAN_IFLAG1_BUF5I ) ) return readFIFO(msg);
  return readMB(msg);

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

        bool enhance_filtering_success = 0;

        for ( uint8_t i = 0; i < mailboxOffset(); i++ ) {
          if ( filter_enhancement[i][0] && filter_set[i] ) { /* if enhancement is active and set */
            if ( !filter_enhancement[i][1] ) { /* if it's multi ID */
              for ( uint8_t j = 0; j < 5; j++ ) {
                if ( msg.id == filter_enhancement_config[i][j] ) {
                  enhance_filtering_success = 1;
                  break;
                }
              }
            }
            else { /* if it's range based */

              if ( (msg.id >= filter_enhancement_config[i][0]) &&
                   (msg.id <= filter_enhancement_config[i][1]) ) enhance_filtering_success = 1;

              if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) { 
                if ( (msg.id >= filter_enhancement_config[i][2]) &&
                     (msg.id <= filter_enhancement_config[i][3]) ) enhance_filtering_success = 1;
              }

            }
          }
        }

        /* if enhancement is on and a match is found, OR, enhancement is disabled, run handler or queue as normal */

        if ( ( enhance_filtering_success && filter_enhancement[0][0] ) || !filter_enhancement[0][0] ) { /* if enhanced AND success, OR not enhanced */
          if ( !can_events ) {
            if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(msg);
            sendMSGtoIndividualMBCallback((IFCTMBNUM)0, msg); /* send frames direct to callback (unbuffered) */
          }
          else {
            if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(msg);
            struct2queue(msg); /* store frame in queue ( buffered ) */
            if ( flexcan_library_emulation ) flexcan_object_oriented_callbacks(msg);
          }
        }

        /* callback, queue, or neither, we check other filters for cross-matches */
        if ( msg_distribution && can_events ) packet_distribution(msg);

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
  uint8_t mailboxes = mailboxOffset();

  /* non FIFO mailbox handling complete */
  /* mailbox handling routine */
  for (uint8_t i = mailboxes; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    if (!(FLEXCANb_IMASK1(_baseAddress) & ( 1UL << i ))) continue; // skip mailboxes that don't have interrupts enabled
    if (!(status & (1UL << i))) continue; // skip mailboxes that haven't triggered an interrupt
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

          bool enhance_filtering_success = 0;
          if ( filter_enhancement[i][0] ) { /* filter enhancement (if enabled) */
            if ( !filter_enhancement[i][1] ) { /* multi-ID based filter enhancement */
              for ( uint8_t i = 0; i < 5; i++ ) {
              if ( msg.id == filter_enhancement_config[msg.mb][i] ) {
                enhance_filtering_success = 1;
                break;
                }
              }
            }
            else { /* range based ID filtering enhancement */
              if ( (msg.id >= filter_enhancement_config[i][0]) &&
                   (msg.id <= filter_enhancement_config[i][1]) ) enhance_filtering_success = 1;
            }
          }

          /* if enhancement is on and a match is found, OR, enhancement is disabled, run handler or queue as normal */

          if ( ( enhance_filtering_success && filter_enhancement[i][0] ) || !filter_enhancement[i][0] ) { /* if enhanced AND success, OR not enhanced */
            if ( !can_events ) {
              if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(msg);
              sendMSGtoIndividualMBCallback((IFCTMBNUM)i, msg); /* send frames direct to callback (unbuffered) */
              if ( flexcan_library_emulation ) flexcan_object_oriented_callbacks(msg);
            }
            else {
              if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(msg);
              struct2queue(msg); /* store frame in queue ( buffered ) */
              if ( flexcan_library_emulation ) flexcan_object_oriented_callbacks(msg);
            }
          }

          if (!msg.flags.extended) FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
          else FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
          FLEXCANb_TIMER(_baseAddress); // reading timer unlocks individual mailbox
          FLEXCANb_IFLAG1(_baseAddress) = (1UL << i); /* immediately flush interrupt of current mailbox */
          status &= ~(1UL << i); /* remove bit from initial flag lookup so it's not set at end when another frame is captured */

          /* callback, queue, or neither, we check other filters for cross-matches */
          if ( msg_distribution && can_events ) packet_distribution(msg);

          break;
        }
      case FLEXCAN_MB_CODE_TX_INACTIVE: {       // TX inactive. Just chillin' waiting for a message to send.
          FLEXCANb_IFLAG1(_baseAddress) = (1UL << i); /* immediately flush interrupt of current mailbox */
          status &= ~(1UL << i); /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
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
  FLEXCANb_ESR1(_baseAddress) |= FLEXCAN_ESR_ERR_INT; /* we clear the ERROR bit if we received a functional callback */
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

  FLEXCAN_EnterFreezeMode();
  for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) {
    filter_enhancement[i][0] = 0;
    filter_enhancement[i][1] = 0;
  }
  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    filter_set[i] = 0; /* reset enhancement filters bit set */
    if ( input == ACCEPT_ALL ) FLEXCANb_MB_MASK(_baseAddress, i) = 0x00000000; // (RXIMR)
    if ( input == REJECT_ALL ) FLEXCANb_MB_MASK(_baseAddress, i) = 0xFFFFFFFF; // (RXIMR)
    FLEXCANb_MBn_ID(_baseAddress, i) = 0x00000000;
  }
  FLEXCAN_ExitFreezeMode();
}



void IFCT::setMBFilter(IFCTMBNUM mb_num, IFCTMBFLTEN input) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return; /* mailbox not available */

  filter_enhancement[mb_num][0] = 0;
  filter_enhancement[mb_num][1] = 0;
  filter_set[mb_num] = 1; /* set filters flag */
  FLEXCAN_EnterFreezeMode();
  if ( input == ACCEPT_ALL ) FLEXCANb_MB_MASK(_baseAddress, mb_num) = 0x00000000; // (RXIMR)
  if ( input == REJECT_ALL ) FLEXCANb_MB_MASK(_baseAddress, mb_num) = 0xFFFFFFFF; // (RXIMR)
  FLEXCAN_ExitFreezeMode();
  FLEXCANb_MBn_ID(_baseAddress, mb_num) = 0x00000000;
}



bool IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* mailbox not available */
  bool extbit = FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE;

  if ( (id1 > 0x7FF) != extbit ) return 0;

  filter_enhancement[mb_num][0] = 0;
  filter_enhancement[mb_num][1] = 0;
  filter_enhancement_config[mb_num][0] = id1;
  filter_enhancement_config[mb_num][1] = id1;
  filter_enhancement_config[mb_num][2] = id1;
  filter_enhancement_config[mb_num][3] = id1;
  filter_enhancement_config[mb_num][4] = id1;
  filter_set[mb_num] = 1; /* set filters flag */

  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1) ^ (id1)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1) ^ (id1)) ^ 0x1FFFFFFF);

  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

bool IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* mailbox not available */
  bool extbit = FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE;

  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit ) return 0;

  filter_enhancement[mb_num][0] = 0;
  filter_enhancement[mb_num][1] = 0;
  filter_enhancement_config[mb_num][0] = id1;
  filter_enhancement_config[mb_num][1] = id2;
  filter_enhancement_config[mb_num][2] = id2;
  filter_enhancement_config[mb_num][3] = id2;
  filter_enhancement_config[mb_num][4] = id2;
  filter_set[mb_num] = 1; /* set filters flag */

  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2) ^ (id1 & id2)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2) ^ (id1 & id2)) ^ 0x1FFFFFFF);

  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

bool IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* mailbox not available */
  bool extbit = FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE;

  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit || (id3 > 0x7FF) != extbit ) return 0;

  filter_enhancement[mb_num][0] = 0;
  filter_enhancement[mb_num][1] = 0;
  filter_enhancement_config[mb_num][0] = id1;
  filter_enhancement_config[mb_num][1] = id2;
  filter_enhancement_config[mb_num][2] = id3;
  filter_enhancement_config[mb_num][3] = id3;
  filter_enhancement_config[mb_num][4] = id3;
  filter_set[mb_num] = 1; /* set filters flag */

  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2 | id3) ^ (id1 & id2 & id3)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2 | id3) ^ (id1 & id2 & id3)) ^ 0x1FFFFFFF);

  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}


void IFCT::enhanceFilter(IFCTMBNUM mb_num) {

  if ( mb_num < mailboxOffset() ) return; /* use "FIFO" instead of "MB(x)" for FIFO region */
  if ( mb_num == FIFO ) {
    for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 1; /* enhance FIFO filters based on consumed mailboxes (RFFN) */
    return; /* finished FIFO filter enhancements */
  }
  filter_enhancement[mb_num][0] = 1;
}

bool IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* mailbox not available */
  bool extbit = FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE;

  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit || (id3 > 0x7FF) != extbit || (id4 > 0x7FF) != extbit ) return 0;

  filter_enhancement[mb_num][0] = 0;
  filter_enhancement[mb_num][1] = 0;
  filter_enhancement_config[mb_num][0] = id1;
  filter_enhancement_config[mb_num][1] = id2;
  filter_enhancement_config[mb_num][2] = id3;
  filter_enhancement_config[mb_num][3] = id4;
  filter_enhancement_config[mb_num][4] = id4;
  filter_set[mb_num] = 1; /* set filters flag */

  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2 | id3 | id4) ^ (id1 & id2 & id3 & id4)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2 | id3 | id4) ^ (id1 & id2 & id3 & id4)) ^ 0x1FFFFFFF);

  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

bool IFCT::setMBFilter(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t id5) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* mailbox not available */
  bool extbit = FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE;

  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit || (id3 > 0x7FF) != extbit || (id4 > 0x7FF) != extbit || (id5 > 0x7FF) != extbit ) return 0;

  filter_enhancement[mb_num][0] = 0;
  filter_enhancement[mb_num][1] = 0;
  filter_enhancement_config[mb_num][0] = id1;
  filter_enhancement_config[mb_num][1] = id2;
  filter_enhancement_config[mb_num][2] = id3;
  filter_enhancement_config[mb_num][3] = id4;
  filter_enhancement_config[mb_num][4] = id5;
  filter_set[mb_num] = 1; /* set filters flag */

  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2 | id3 | id4 | id5) ^ (id1 & id2 & id3 & id4 & id5)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2 | id3 | id4 | id5) ^ (id1 & id2 & id3 & id4 & id5)) ^ 0x1FFFFFFF);

  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

bool IFCT::setMBFilterRange(IFCTMBNUM mb_num, uint32_t id1, uint32_t id2) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return 0; /* mailbox not available */
  bool extbit = FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE;

  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit ) return 0;
  if ( id1 > id2 || ((id2 > id1) && (id2-id1>1000)) || !id1 || !id2 ) return 0; /* don't play around... */

  filter_enhancement[mb_num][0] = 0;
  filter_enhancement[mb_num][1] = 1;
  filter_enhancement_config[mb_num][0] = id1;
  filter_enhancement_config[mb_num][1] = id2;
  filter_enhancement_config[mb_num][2] = id2;
  filter_enhancement_config[mb_num][3] = id2;
  filter_enhancement_config[mb_num][4] = id2;
  filter_set[mb_num] = 1; /* set filters flag */


  uint32_t stage1 = id1, stage2 = id1;
  for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
    stage1 |= i; stage2 &= i;
  }

  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD( (stage1 ^ stage2) ^ 0x1FFFFFFF ) : FLEXCAN_MB_ID_IDEXT( (stage1 ^ stage2) ^ 0x1FFFFFFF );

  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

void IFCT::setMBFilterProcessing(IFCTMBNUM mb_num, uint32_t filter_id, uint32_t calculated_mask) {
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_MB_MASK(_baseAddress, mb_num) = calculated_mask;
  masks[mb_num] = calculated_mask; /* we store updated masks to array, since we can't access it out of FRZ mode */
  FLEXCAN_ExitFreezeMode();
  if (!(FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE)) FLEXCANb_MBn_ID(_baseAddress, mb_num) = FLEXCAN_MB_ID_IDSTD(filter_id);
  else FLEXCANb_MBn_ID(_baseAddress, mb_num) = FLEXCAN_MB_ID_IDEXT(filter_id);
}

uint8_t IFCT::mailboxOffset() {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN ) ) return 0; /* return offset 0 since FIFO is disabled */

  uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_baseAddress) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
  if ( FLEXCANb_MAXMB_SIZE(_baseAddress) < (6 + ((((FLEXCANb_CTRL2(_baseAddress) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
  return (FLEXCANb_MAXMB_SIZE(_baseAddress) - remaining_mailboxes); /* otherwise return offset MB position after FIFO area */

}


void IFCT::setFIFOFilter(const IFCTMBFLTEN &input) {
  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */

  uint8_t mailboxes = mailboxOffset();

  FLEXCAN_EnterFreezeMode();
  for (uint8_t i = 0; i < mailboxes; i++) { /* block all ID's so filtering could be applied. */
    filter_set[i] = 0; /* reset enhancement filters bit set all off */
    if ( input == REJECT_ALL ) {

      if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) { /* If Table A is chosen for FIFO */
        FLEXCANb_IDFLT_TAB(_baseAddress, i) = 0xFFFFFFFF; /* reset id */
        FLEXCANb_MB_MASK(_baseAddress, i) = 0xFFFFFFFF; // (RXIMR) /* block all id's */
        masks[i] = 0xFFFFFFFF;
      }

      else if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) { /* If Table B is chosen for FIFO */
        FLEXCANb_IDFLT_TAB(_baseAddress, i) = 0xFFFFFFFF; /* reset id */
        FLEXCANb_MB_MASK(_baseAddress, i) = 0x3FFF3FFF; // (RXIMR) /* block all id's */
        masks[i] = 0x3FFF3FFF;
      }

      else if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 2 ) { /* If Table C is chosen for FIFO */
        FLEXCANb_IDFLT_TAB(_baseAddress, i) = 0x6E6E6E6E; /* reset id */
        FLEXCANb_MB_MASK(_baseAddress, i) = 0xFFFFFFFF; // (RXIMR) /* block all id's */
        masks[i] = 0xFFFFFFFF;
      }

    }
    else if ( input == ACCEPT_ALL ) {
      FLEXCANb_IDFLT_TAB(_baseAddress, i) = 0; /* reset id */
      FLEXCANb_MB_MASK(_baseAddress, i) = 0; // (RXIMR) /* allow all id's */
      masks[i] = 0;
    }
  }
  FLEXCAN_ExitFreezeMode();
}



// FIFO TABLE_A SINGLE ID FILTER STD/EXT
bool IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, const IFCTMBIDE &ide, const IFCTMBIDE &remote) {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 0 ) return 0; /* must be TableA to process */
  if ( filter >= mailboxOffset() ) return 0; /* not in FIFO region */

  filter_set[filter] = 1; /* show that it's set */
  for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 0; /* disable FIFO enhancement */
  filter_enhancement[filter][1] = 0; /* set it as multiid based */
  filter_enhancement_config[filter][0] = id1;
  filter_enhancement_config[filter][1] = id1;
  filter_enhancement_config[filter][2] = id1;
  filter_enhancement_config[filter][3] = id1;
  filter_enhancement_config[filter][4] = id1;

  FLEXCAN_EnterFreezeMode();

  uint32_t mask = ( ide != EXT ) ? ((((id1) ^ (id1)) ^ 0x7FF) << 19 ) | 0xC0000001 : ((((id1) ^ (id1)) ^ 0x3FFFFFFF) << 1 ) | 0xC0000001;

  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  masks[filter] = mask;
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
      ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  FLEXCAN_ExitFreezeMode();
  return 1;
}


// FIFO TABLE_A 2x FILTER STD or EXT
bool IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide, const IFCTMBIDE &remote) {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 0 ) return 0; /* must be TableA to process */
  if ( filter >= mailboxOffset() ) return 0; /* not in FIFO region */

  filter_set[filter] = 1; /* show that it's set */
  for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 0; /* disable FIFO enhancement */
  filter_enhancement[filter][1] = 0; /* set it as multiid based */
  filter_enhancement_config[filter][0] = id1;
  filter_enhancement_config[filter][1] = id2;
  filter_enhancement_config[filter][2] = id2;
  filter_enhancement_config[filter][3] = id2;
  filter_enhancement_config[filter][4] = id2;

  FLEXCAN_EnterFreezeMode();

  uint32_t mask = ( ide != EXT ) ? ((((id1 | id2) ^ (id1 & id2)) ^ 0x7FF) << 19 ) | 0xC0000001 : ((((id1 | id2) ^ (id1 & id2)) ^ 0x3FFFFFFF) << 1 ) | 0xC0000001;

  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  masks[filter] = mask;
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
      ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  FLEXCAN_ExitFreezeMode();
  return 1;
}


// FIFO TABLE_A FILTER RANGE min -> max, STD or EXT
bool IFCT::setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide, const IFCTMBIDE &remote) {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 0 ) return 0; /* must be TableA to process */
  if ( id1 > id2 || ((id2 > id1) && (id2 - id1 > 1000)) || !id1 || !id2 ) return 0; /* don't play around... */
  if ( filter >= mailboxOffset() ) return 0; /* not in FIFO region */

  filter_set[filter] = 1; /* show that it's set */
  for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 0; /* disable FIFO enhancement */
  filter_enhancement[filter][1] = 1; /* set it as range based */
  filter_enhancement_config[filter][0] = id1;
  filter_enhancement_config[filter][1] = id2;
  filter_enhancement_config[filter][2] = id2;
  filter_enhancement_config[filter][3] = id2;
  filter_enhancement_config[filter][4] = id2;

  FLEXCAN_EnterFreezeMode();

  uint32_t stage1 = id1, stage2 = id1;
  for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
    stage1 |= i; stage2 &= i;
  }

  uint32_t mask = ( ide != EXT ) ? (((stage1 ^ stage2) ^ 0x7FF) << 19) | 0xC0000001 : (((stage1 ^ stage2) ^ 0x3FFFFFFF) << 1) | 0xC0000001;

  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  masks[filter] = mask;
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
      ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  FLEXCAN_ExitFreezeMode();
  return 1;
}


void IFCT::setFIFOFilterTable(IFCTFIFOTABLE letter) {
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_MCR(_baseAddress) |= FLEXCAN_MCR_IDAM(letter);
  FLEXCAN_ExitFreezeMode();
}


// FIFO TABLE_B DUAL STD/EXT/MIXED ID FILTER
bool IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, const IFCTMBIDE &ide1, const IFCTMBIDE &remote1, uint32_t id2, const IFCTMBIDE &ide2, const IFCTMBIDE &remote2) {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 1 ) return 0; /* must be TableB to process */
  if ( filter >= mailboxOffset() ) return 0; /* not in FIFO region */

  filter_set[filter] = 1; /* show that it's set */
  for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 0; /* disable FIFO enhancement */
  filter_enhancement[filter][1] = 0; /* set it as multi id based */
  filter_enhancement_config[filter][0] = id1;
  filter_enhancement_config[filter][1] = id2;
  filter_enhancement_config[filter][2] = id2;
  filter_enhancement_config[filter][3] = id2;
  filter_enhancement_config[filter][4] = id2;

  FLEXCAN_EnterFreezeMode();
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
      ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
      (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
      (ide2 == EXT ? ((id2 >> (29 - 14)) <<  0) : ((id2 & 0x7FF) <<  3)) ; /* second ID is EXT or STD? */

  uint32_t mask = ( ide1 != EXT ) ? ((((id1) ^ (id1)) ^ 0x7FF) << 19 ) | 0xC0070000 : ((((id1) ^ (id1)) ^ 0x3FFF) << 16 | 0xC0000000 );
  ( ide2 != EXT ) ? mask |= ((((id2) ^ (id2)) ^ 0x7FF) << 3 ) | 0xC007 : mask |= ((((id2) ^ (id2)) ^ 0x3FFF) << 0 | 0xC000);

  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  masks[filter] = mask;
  FLEXCAN_ExitFreezeMode();
  return 1;
}


// FIFO TABLE_B QUAD STD/14MSB EXT/MIXED ID FILTER
bool IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide1, const IFCTMBIDE &remote1, uint32_t id3, uint32_t id4, const IFCTMBIDE &ide2, const IFCTMBIDE &remote2) {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 1 ) return 0; /* must be TableB to process */
  if ( filter >= mailboxOffset() ) return 0; /* not in FIFO region */

  filter_set[filter] = 1; /* show that it's set */
  for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 0; /* disable FIFO enhancement */
  filter_enhancement[filter][1] = 0; /* set it as multi id based */
  filter_enhancement_config[filter][0] = id1;
  filter_enhancement_config[filter][1] = id2;
  filter_enhancement_config[filter][2] = id3;
  filter_enhancement_config[filter][3] = id4;
  filter_enhancement_config[filter][4] = id4;

  FLEXCAN_EnterFreezeMode();

  uint32_t mask = ( ide1 != EXT ) ? ((((id1 | id2) ^ (id1 & id2)) ^ 0x7FF) << 19 ) | 0xC0070000 : ((((id1 | id2) ^ (id1 & id2)) ^ 0x3FFF) << 16 ) | 0xC0000000;
  mask |= ( ide2 != EXT ) ? ((((id3 | id4) ^ (id3 & id4)) ^ 0x7FF) << 3 ) | 0xC007 : ((((id3 | id4) ^ (id3 & id4)) ^ 0x3FFF) << 0 ) | 0xC000;

  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
      ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */

      (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
      (ide2 == EXT ? ((id3 >> (29 - 14)) << 0 ) : ((id3 & 0x7FF) << 3 ))  ; /* second ID is EXT or STD? */

  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  masks[filter] = mask;
  FLEXCAN_ExitFreezeMode();
  return 1;
}


// FIFO TABLE_B DUAL RANGE STD/EXT/MIXED ID FILTER 
bool IFCT::setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const IFCTMBIDE &ide1, const IFCTMBIDE &remote1, uint32_t id3, uint32_t id4, const IFCTMBIDE &ide2, const IFCTMBIDE &remote2) {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 1 ) return 0; /* must be TableB to process */
  if ( filter >= mailboxOffset() ) return 0; /* not in FIFO region */
  if ( id1 > id2 || ((id2 > id1) && (id2 - id1 > 1000)) || !id1 || !id2 ) return 0; /* don't play around... */
  if ( id3 > id4 || ((id4 > id3) && (id4 - id3 > 1000)) || !id3 || !id4 ) return 0; /* don't play around... */

  filter_set[filter] = 1; /* show that it's set */
  for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 0; /* disable FIFO enhancement */
  filter_enhancement[filter][1] = 1; /* set it as range based */
  filter_enhancement_config[filter][0] = id1;
  filter_enhancement_config[filter][1] = id2;
  filter_enhancement_config[filter][2] = id3;
  filter_enhancement_config[filter][3] = id4;
  filter_enhancement_config[filter][4] = id4;

  FLEXCAN_EnterFreezeMode();

  uint32_t stage1 = id1, stage2 = id1;

  for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
    stage1 |= i; stage2 &= i;
  }

  uint32_t mask = ( ide1 != EXT ) ? (((stage1 ^ stage2) ^ 0x7FF) << 19) | 0xC0070000 : (((stage1 ^ stage2) ^ 0x3FFF) << 16) | 0xC0000000;

  stage1 = stage2 = id3;

  for ( uint32_t i = id3 + 1; i <= id4; i++ ) {
    stage1 |= i; stage2 &= i;
  }

  ( ide2 != EXT ) ? mask |= (((stage1 ^ stage2) ^ 0x7FF) << 3) | 0xC007 : mask |= (((stage1 ^ stage2) ^ 0x3FFF) << 0) | 0xC000;

  FLEXCANb_IDFLT_TAB(_baseAddress, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
      ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
      (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
      (ide2 == EXT ? ((id3 >> (29 - 14)) << 0 ) : ((id3 & 0x7FF) << 3 ))  ; /* second ID is EXT or STD? */
  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  masks[filter] = mask;
  FLEXCAN_ExitFreezeMode();
  return 1;
}

// FIFO TABLE_C QUAD ID EXT/STD, 8MSBs each.
bool IFCT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4 ) {

  if ( !(FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) != 2 ) return 0; /* must be TableC to process */
  if ( filter >= mailboxOffset() ) return 0; /* not in FIFO region */

  filter_set[filter] = 1; /* show that it's set */
  for ( uint8_t i = 0; i < mailboxOffset(); i++ ) filter_enhancement[i][0] = 0; /* disable FIFO enhancement */
  filter_enhancement[filter][1] = 1; /* set it as range based */
  filter_enhancement_config[filter][0] = id1;
  filter_enhancement_config[filter][1] = id2;
  filter_enhancement_config[filter][2] = id3;
  filter_enhancement_config[filter][3] = id4;
  filter_enhancement_config[filter][4] = id4;

  FLEXCAN_EnterFreezeMode();
  FLEXCANb_IDFLT_TAB(_baseAddress, filter) =
    (( id1 > 0x7FF ) ? ((id1 >> (29 - 8)) << 24) : ((id1 >> (11 - 8)) << 24)) |
    (( id2 > 0x7FF ) ? ((id2 >> (29 - 8)) << 16) : ((id2 >> (11 - 8)) << 16)) |
    (( id3 > 0x7FF ) ? ((id3 >> (29 - 8)) << 8) :  ((id3 >> (11 - 8)) <<  8)) |
    (( id4 > 0x7FF ) ? ((id4 >> (29 - 8)) << 0) :  ((id4 >> (11 - 8)) <<  0));

  uint32_t mask = ( id1 > 0x7FF ) ? (id1>>(29-8)) << 24 : (id1>>(11-8)) << 24;
  mask |= ( id2 > 0x7FF ) ? (id2>>(29-8)) << 16 : (id2>>(11-8)) << 16;
  mask |= ( id3 > 0x7FF ) ? (id3>>(29-8)) << 8  : (id3>>(11-8)) <<  8;
  mask |= ( id4 > 0x7FF ) ? (id4>>(29-8)) << 0  : (id4>>(11-8)) <<  0;

  FLEXCANb_MB_MASK(_baseAddress, filter) = mask; // (RXIMR)
  masks[filter] = mask;
  FLEXCAN_ExitFreezeMode();
  return 1;
}



bool IFCT::connected() {
  uint32_t esr1 = FLEXCANb_ESR1(_baseAddress);
  if ( esr1&0x40000 && !(esr1&0xF000) && !(esr1&0x30) ) return 1; /* synch, no crc/ack/bit errors */
  return 0;
}



uint16_t IFCT::events() {

  { Threads::Scope scope(CAN_THREAD[((this == &Can0) ? 0 : 1)]);

    if ( !can_events ) can_events = 1; /* handle callbacks from loop */

    if ( flexcan_buffer.size() ) { /* if a queue frame is available */
      CAN_message_t frame;
      queue2struct(frame);
      if ( IFCT::_MBAllhandler != nullptr ) IFCT::_MBAllhandler(frame);
      sendMSGtoIndividualMBCallback((IFCTMBNUM)frame.mb, frame); 
      return flexcan_buffer.size();
    }

  } // SCOPE LOCK END 
  return 0;
}



void IFCT::packet_distribution(CAN_message_t &frame) {
  uint8_t mailbox_offset = mailboxOffset();

  for ( uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {

    /* if fifo enabled, skip the area it occupies except the MB0 slot when checking interrupt enable status */
    /* continue scanning from 0, then depending on RFFN, lets say 0, scan continues at slot 8 onwards */

    if ( !(FLEXCANb_IMASK1(_baseAddress) & (1 << i)) ) {
      if ( (FLEXCANb_IMASK1(_baseAddress) & FLEXCAN_IMASK1_BUF5M) && (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) && ( i < mailbox_offset ) ); /* do not skip fifo filters */
      else continue; /* skip non-interrupt mailboxes */
    }

    /* here we prevent 2 or more FIFO filters from duplicating a message frame */
    if ( (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) && !frame.mb && i < mailbox_offset ) continue;

    if ( filter_set[i] ) { /* if MB/FIFO filter is set */

      if ( filter_enhancement[i][0] ) {
        if ( !filter_enhancement[i][1] ) { /* multi-ID based filter enhancement */
          for ( uint8_t j = 0; j < 5; j++ ) {
            if ( i != frame.mb && frame.id == filter_enhancement_config[i][j] ) {
              uint8_t mb = frame.mb;
              frame.mb = i; /* set ID to distributed mailbox */
              if ( (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) && i < mailbox_offset ) { /* make FIFO entries always 0 */
                frame.mb = 0;
                i = mailbox_offset - 1;
              }
              struct2queue(frame);
              frame.mb = mb; /* restore id for next cycle */
              break;
            }
          }
        } // MULTI-ID BASED FILTERING


        else { /* if it's range based */

          bool enhance_filtering_success = 0;

          if ( (frame.id >= filter_enhancement_config[i][0]) &&
               (frame.id <= filter_enhancement_config[i][1]) ) enhance_filtering_success = 1;
          if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) { 
            if ( (frame.id >= filter_enhancement_config[i][2]) &&
                 (frame.id <= filter_enhancement_config[i][3]) ) enhance_filtering_success = 1;
          }

          if ( i != frame.mb && enhance_filtering_success ) {
            uint8_t mb = frame.mb;
            frame.mb = i; /* set ID to distributed mailbox */
            if ( (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) && i < mailbox_offset ) { /* make FIFO entries always 0 */
              frame.mb = 0;
              i = mailbox_offset - 1;
            }
            struct2queue(frame);
            frame.mb = mb; /* restore id for next cycle */
          }
        } // RANGE BASED

      } // ENHANCED FILTER


      else { /* if no enhancement */

        if ( i >= mailboxOffset() && i != frame.mb ) { /* mailbox area, id from different mailbox */

          /* if the ID doesn't match the EXT flag, skip filter check */
          if ( (( frame.id >  0x7FF ) && !( FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE )) ||
               (( frame.id <= 0x7FF ) &&  ( FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE )) ) continue;

          uint32_t mask = ( !(FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE) ) ? (masks[i] >> 18) & 0x7FF : (masks[i] >> 0) & 0x1FFFFFFF;

          if ( ((frame.id)&mask) == ((filter_enhancement_config[i][0])&mask) ) {
            uint8_t mb = frame.mb;
            frame.mb = i; /* set ID to distributed mailbox */
            struct2queue(frame);
            frame.mb = mb; /* restore id for next cycle */
          }
        } // MB AREA

        else if ( FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN && i < mailboxOffset() && i != frame.mb ) {

          if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) { /* Table A */

            uint32_t mask = (( !(FLEXCANb_IDFLT_TAB(_baseAddress, i) & (1UL << 30)) ) ? (masks[i] >> 19) & 0x7FF : (masks[i] >> 16) & 0x3FFF);

            if ( ((frame.id)&mask) == ((filter_enhancement_config[i][0])&mask) ) {
              uint8_t mb = frame.mb;
              frame.mb = 0; /* set ID to FIFO buffer */
              i = mailbox_offset - 1;
              struct2queue(frame);
              frame.mb = mb; /* restore id for next cycle */
            }
          } // TABLE_A

          else if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) { /* Table B */

            bool filter_match1 = 0, filter_match2 = 0;

            if ( FLEXCANb_IDFLT_TAB(_baseAddress, i) & (1UL << 30) ) { /* if extended bit */
              if ( ((frame.id >> (29 - 14))&(masks[i]&0x3FFF)) == ((filter_enhancement_config[i][0] >> (29 - 14))&(masks[i]&0x3FFF)) ) filter_match1 = 1;
            }
            else {
              if ( (frame.id&((masks[i] >> 19) & 0x7FF)) == (filter_enhancement_config[i][0]&((masks[i] >> 19) & 0x7FF)) ) filter_match1 = 1;
            }

            if ( FLEXCANb_IDFLT_TAB(_baseAddress, i) & (1UL << 14) ) { /* if extended bit */
              if ( ((frame.id >> (29 - 14))&(masks[i]&0x3FFF)) == ((filter_enhancement_config[i][2] >> (29 - 14))&(masks[i]&0x3FFF)) ) filter_match2 = 1;
            } 
            else {
              if ( (frame.id&((masks[i] >> 19) & 0x7FF)) == (filter_enhancement_config[i][0]&((masks[i] >> 19) & 0x7FF)) ) filter_match2 = 1;
            }

            if ( filter_match1 || filter_match2 ) {
              uint8_t mb = frame.mb;
              frame.mb = 0; /* set ID to FIFO buffer */
              i = mailbox_offset - 1;
              struct2queue(frame);
              frame.mb = mb; /* restore id for next cycle */
            }
          } // TABLE_B

          else if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 2 ) { /* Table C */

              bool filter_match = 0;

              uint8_t bits = (( frame.id > 0x7FF ) ? 29 : 11);

              for ( uint8_t j = 0; j < 4; j++ ) {

                /* for partial masks, we double check ext frames dont mix with std frames */
                /* from local copy of original ID and frame.id itself */
                if ( (frame.ext && filter_enhancement_config[i][j] <= 0x7FF) ||
                     (!frame.ext && filter_enhancement_config[i][j] > 0x7FF) ) continue;

                /* look at the 4 partial masks for a match */
                if ( ((frame.id >> (bits - 8))&(masks[i]&0xFF)) == ((filter_enhancement_config[i][j] >> (bits - 8))&(masks[i]&0xFF)) ) {
                  filter_match = 1;
                  break;
                }
              }

              if ( filter_match ) {
                uint8_t mb = frame.mb;
                frame.mb = 0; /* set ID to FIFO buffer */
                i = mailbox_offset - 1;
                struct2queue(frame);
                frame.mb = mb; /* restore id for next cycle */
              }
          } // TABLE_C

        } // FIFO AREA
      } // NOT ENHANCED

    } // FILTER_SET
  } // FOR LOOP
}



void IFCT::currentMasks() {

  FLEXCAN_EnterFreezeMode(); /* let's get current list, must be in FRZ mode */
  for ( uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) masks[i] = FLEXCANb_MB_MASK(_baseAddress, i);
  FLEXCAN_ExitFreezeMode();

  Serial.println();
  if ( (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) ) {
    Serial.print("FIFO enabled. Filters 0 to ");
    Serial.print(mailboxOffset() - 1);
    Serial.println(" used for FIFO.");
  }

  Serial.print("\nMasks:\n");
  char mask_padded[10];
  for ( uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) {

    if ( (i >= mailboxOffset()) && (FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i)) & 0x08) ) { // TX or RX mailbox?
      Serial.print("[ ");
      Serial.print(i);
      Serial.print(" ]:\tTransmit Mailbox\n\n");
      continue;
    }

    sprintf(mask_padded, "%08lX", masks[i]);
    Serial.print("[ ");
    Serial.print(i);
    Serial.print(" ]:\tFlexcan Mask: 0x");
    Serial.print(mask_padded);

    if ( (FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_FEN) && i < mailboxOffset() ) { /* detailed filter for FIFO only */

      Serial.print(" ( FIFO Filter )\n\t\t^-- ");

      if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) { /* Table A */
        if ( !filter_set[i] ) {
          Serial.print(" [Table A]\n\t\t\t * Filter was not set\n\n");
          continue;
        }

        Serial.print(" [Table A]\n\t\t\t * User Mask: ");
        if ( !(masks[i] & 0x1FFFE) ) {
          Serial.print("(Standard Mask) 0x");
          Serial.print((masks[i] >> 19)&0x7FF,HEX);
        }
        else {
          Serial.print("(Extended Mask) 0x");
          Serial.print((masks[i] >> 1)&0x1FFFFFFF,HEX);
        }
        Serial.println();
      }


      else if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) { /* Table B */
        if ( !filter_set[i] ) {
          Serial.print(" [Table B]\n\t\t\t * Filter was not set\n\n");
          continue;
        }

        Serial.print(" [Table B]\n\t\t\t * User Mask 1: ");
        if ( (FLEXCANb_IDFLT_TAB(_baseAddress, i) & (1UL<<30)) ) {
          Serial.print("(Extended Mask) 0x");
          Serial.print((masks[i] >> 16)&0x3FFF,HEX);
        }
        else {
          Serial.print("(Standard Mask) 0x");
          Serial.print((masks[i] >> 19)&0x7FF,HEX);
        }

        Serial.print("\n\t\t\t * User Mask 2: ");

        if ( (FLEXCANb_IDFLT_TAB(_baseAddress, i) & (1UL<<14)) ) {
          Serial.print("(Extended Mask) 0x");
          Serial.print((masks[i] >> 0)&0x3FFF,HEX);
        }
        else {
          Serial.print("(Standard Mask) 0x");
          Serial.print((masks[i] >> 3)&0x7FF,HEX);
        }

        Serial.println();

      }
      else if ( ((FLEXCANb_MCR(_baseAddress) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 2 ) { /* Table C */
        if ( !filter_set[i] ) {
          Serial.print(" [Table C] * Filter was not set\n\n");
          continue;
        }
        Serial.print(" [Table C]\n\t\t\t * User Mask 1: 0x");
        Serial.print((masks[i] >> 0)&0xFF000000,HEX);
        Serial.print("\n\t\t\t * User Mask 2: 0x");
        Serial.print((masks[i] >> 0)&0x00FF0000,HEX);
        Serial.print("\n\t\t\t * User Mask 3: 0x");
        Serial.print((masks[i] >> 0)&0x0000FF00,HEX);
        Serial.print("\n\t\t\t * User Mask 4: 0x");
        Serial.println((masks[i] >> 0)&0x000000FF,HEX);
      }
    }

    if ( i >= mailboxOffset() ) { /* detailed filter for mailboxes only */
      Serial.print(" ( Mailbox Filter )\n\t\t^-- ");

      if ( !filter_set[i] ) {
        Serial.print("* Filter was not set\n\n");
        continue;
      }

      if ( (FLEXCANb_MBn_CS(_baseAddress, i) & FLEXCAN_MB_CS_IDE) ? 0 : 1 ) {
        Serial.print("(Standard Mask) * User Mask: 0x");
        Serial.print((masks[i] >> 18)&0x7FF,HEX);
      }
      else {
        Serial.print("(Extended Mask) * User Mask: 0x");
        Serial.print((masks[i] >> 0)&0x1FFFFFFF,HEX);
      }
      Serial.println();
    }
    Serial.println();
  }
  Serial.println();
}

bool IFCT::autoBaud() {

  const uint32_t speeds[] = { 10000, 20000, 33333, 50000, 83333, 125000, 250000, 500000, 800000, 1000000 };
  uint8_t count = sizeof(speeds) / sizeof(speeds[0]);
  bool found = 0;
  uint32_t esr1 = (FLEXCANb_ESR1(_baseAddress) = 0x6); // clear and set the variable

  while ( count ) {
    count--;
    esr1 = (FLEXCANb_ESR1(_baseAddress) = 0x6);
    FLEXCAN_EnterFreezeMode();
    FLEXCANb_ECR(_baseAddress) = 0;
    softResetRestore();
    setBaudRate(speeds[count]);
    FLEXCANb_CTRL1(_baseAddress) |= FLEXCAN_CTRL_LOM; /* listen only mode enable */
    FLEXCAN_ExitFreezeMode();

    uint32_t timeout = millis();
    while ( millis() - timeout < 79 ) {
      esr1 = FLEXCANb_ESR1(_baseAddress);
      esr1 = FLEXCANb_ESR1(_baseAddress);
      if ( millis() - timeout > 29 ) {
        if ( !(esr1&0x2) && esr1&0x40000 && (esr1&0x8) && !(esr1&0xF000) && !(esr1&0x400) ) {
          found = 1;
          break;
        }
      }
    }
    if ( found ) break;
  }

  if ( found ) {
    FLEXCAN_EnterFreezeMode();
    FLEXCANb_CTRL1(_baseAddress) &= ~FLEXCAN_CTRL_LOM; /* unset listen-only mode */
    FLEXCAN_ExitFreezeMode();
    return 1;
  }

  currentBitrate = 0;
  return 0;
}




void IFCT::acceptedIDs(const IFCTMBNUM &mb_num, bool list) {

  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_baseAddress) ) return; /* mailbox not available */

  Serial.print("\nMB");
  Serial.print(mb_num);
  Serial.print(" accepted IDs: \t");

  uint32_t shifted_mask = (FLEXCANb_MBn_CS(_baseAddress, mb_num) & FLEXCAN_MB_CS_IDE) ? masks[mb_num] : masks[mb_num] >> 18;
  uint32_t canid = filter_enhancement_config[mb_num][0];
  uint32_t min = canid >> __builtin_clz(shifted_mask) << __builtin_clz(shifted_mask);
  uint32_t max = min + shifted_mask;

  for ( uint32_t i = min, count = 0, list_count = 0; i <= max; i++ ) {

    if ( ( canid & shifted_mask ) == ( i & shifted_mask ) ) {

      if ( !list && list_count++ > 50 ) {
        Serial.print("...");
        break;
      }


      Serial.print("0x"); Serial.print(i, HEX);
      if ( count++ >= 5 ) {
        Serial.print("\n\t\t\t");
        count = 0;
        continue;
      }
      Serial.print("\t");
    }
  }
  Serial.println("\n");

  if ( !filter_enhancement[mb_num][0] ) Serial.println("Enhancement Disabled");
  else Serial.println("Enhancement Enabled");

  if ( filter_enhancement[mb_num][1] ) {
    Serial.print("\t\t* Enhanced ID-range mode filtering:  0x"); /* ID range based */
    Serial.print(filter_enhancement_config[mb_num][0],HEX);
    Serial.print(" <--> 0x");
    Serial.print(filter_enhancement_config[mb_num][1],HEX);
  }
  else {
    Serial.print("\t\t* Enhanced Multi-ID mode filtering:  "); /* multi-id based */

    std::sort(&filter_enhancement_config[mb_num][0], &filter_enhancement_config[mb_num][5]);

    for ( uint8_t i = 0; i < 5; ) {
      Serial.print("0x");
      Serial.print(filter_enhancement_config[mb_num][i],HEX);
      Serial.print(" ");
      while ( (i < 4) && (filter_enhancement_config[mb_num][i] == filter_enhancement_config[mb_num][i+1]) ) {
        i++;
        continue;
      }
      i++;
    }
  } Serial.println("\n");
}

















// ################################################################################################################
// ########################################  /* INTERVALTIMER */ ##################################################
// ################################################################################################################

void intervalTimerCan0() {
  Can0.events();
}

#if defined(__MK66FX1M0__)
void intervalTimerCan1() {
  Can1.events();
}
#endif

void IFCT::intervalTimer(bool enable, uint32_t time, uint8_t priority) {
  if ( this == &Can0 ) {
    if ( enable ) {
      CAN_timer[0].begin(intervalTimerCan0, time);
      CAN_timer[0].priority(priority);
      teensyThread(0);
    }
    else CAN_timer[0].end();
  }
#if defined(__MK66FX1M0__)
  if ( this == &Can1 ) {
    if ( enable ) {
      CAN_timer[1].begin(intervalTimerCan1, time);
      CAN_timer[1].priority(priority);
      teensyThread(0);
    }
    else CAN_timer[1].end();
  }
#endif
}



// ################################################################################################################
// ########################################  /* TEENSYTHREADS */ ##################################################
// ################################################################################################################

uint8_t can0_thread_block = 0;
uint8_t can1_thread_block = 0;

void can0_thread() {
  while(1) {
    if ( !can0_thread_block ) Can0.events();
  }
}

#if defined(__MK66FX1M0__)
void can1_thread() {
  while(1) {
    if ( !can1_thread_block ) Can1.events();
  }
}
#endif

void IFCT::teensyThread(bool enable) {
  if ( this == &Can0 ) {
    if ( enable ) {
      intervalTimer(0);
      static bool once = 1;
      if ( once ) threads.addThread(can0_thread);
      once = 0;
      can0_thread_block = 0;
    }
    else can0_thread_block = 1;
  }
#if defined(__MK66FX1M0__)
  if ( this == &Can1 ) {
    if ( enable ) {
      static bool once = 1;
      if ( once ) threads.addThread(can1_thread);
      once = 0;
      can1_thread_block = 0;
    }
    else can1_thread_block = 1;
  }
#endif
}






























// ################################################################################################################
// ####################  /* FLEXCAN_LIBRARY COLLIN80 COMPATIBILITY MODE */ ########################################
// ################################################################################################################

CAN_filter_t IFCT::defaultMask;
Circular_Buffer<uint8_t, FLEXCAN_BUFFER_SIZE, sizeof(CAN_message_t)> IFCT::flexcan_library;

void IFCT::begin(uint32_t baud, const CAN_filter_t &mask, uint8_t txAlt, uint8_t rxAlt) {
  if ( flexcan_library_choice == tonton81 ) flexcan_library_choice = collin80;

  setRX((IFCTALTPIN)!rxAlt); setTX((IFCTALTPIN)!rxAlt);

  setBaudRate(baud);

  disableFIFO();

  for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) { // clear all mailboxes
    FLEXCANb_MBn_ID(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_WORD0(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_WORD1(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_CS(_baseAddress, i) = 0x00000000;
  }

  for (uint8_t i = FLEXCANb_MAXMB_SIZE(_baseAddress) -2; i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++ ) { // set tx mailboxes
    FLEXCANb_MBn_ID(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_WORD0(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_WORD1(_baseAddress, i) = 0x00000000;
    FLEXCANb_MBn_CS(_baseAddress, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }

  setNumTxBoxes(2);

  for (uint8_t c = 0; c < getNumRxBoxes(); c++) {
    setMask(0, c);
    setFilter(mask, c);
  }

  flexcan_library_emulation = 1;

  for (uint32_t i = 0; i < SIZE_LISTENERS; i++) listener[i] = nullptr;

  FLEXCANb_IMASK1(_baseAddress) = 0xFFFF;
}


uint8_t IFCT::getNumRxBoxes() {
  uint8_t count = 0;
  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    if ( !(FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i) >> 3)) ) count++; // if RX
  }
  return count;
} 


void IFCT::setMask(uint32_t mask, uint8_t mbox) {
  if ( mbox >= mailboxOffset() ) {
    FLEXCAN_EnterFreezeMode();
    FLEXCANb_MB_MASK(_baseAddress, mbox) = mask;
    masks[mbox] = mask;
    FLEXCAN_ExitFreezeMode();
  }
}


void IFCT::setFilter(const CAN_filter_t &filter, uint8_t mbox) {
  if ( mbox >= mailboxOffset() ) {
    filter_enhancement_config[mbox][0] = filter.id;
    filter_set[mbox] = 1;
    if (filter.flags.extended) {
      FLEXCANb_MBn_ID(_baseAddress, mbox) = (filter.id & FLEXCAN_MB_ID_EXT_MASK);
      FLEXCANb_MBn_CS(_baseAddress, mbox) |= FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR;
    }

    else {
      FLEXCANb_MBn_ID(_baseAddress, mbox) = FLEXCAN_MB_ID_IDSTD(filter.id);
      FLEXCANb_MBn_CS(_baseAddress, mbox) |= FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
    }
  }
}


uint8_t IFCT::setNumTxBoxes(uint8_t txboxes) {
  uint8_t c;
  uint32_t oldIde;

  if (txboxes > getNumMailBoxes() - 1) txboxes = getNumMailBoxes() - 1;
  if (txboxes < 1) txboxes = 1;

  for (c = 0; c < getNumRxBoxes(); c++) {
    oldIde = FLEXCANb_MBn_CS(_baseAddress, c) & FLEXCAN_MB_CS_IDE;
    FLEXCANb_MBn_CS(_baseAddress, c) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | oldIde;
  }

  for (c = getFirstTxBox(); c < getNumMailBoxes(); c++) {
    FLEXCANb_MBn_CS(_baseAddress, c) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }

  return getTxBoxCount();
}


uint8_t IFCT::getFirstTxBox() {
  uint8_t count = 0;
  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i) >> 3)) ) count++; // if TX
  }

  return (getNumMailBoxes() - count);
}

uint8_t IFCT::getTxBoxCount() {
  uint8_t count = 0;
  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_baseAddress); i++) {
    if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, i) >> 3)) ) count++; // if TX
  }
  return count;
}


void IFCT::flexcan_library_struct2queue(const CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  memmove(buf, &msg, sizeof(msg));
  flexcan_library.push_back(buf, sizeof(CAN_message_t));
}


void IFCT::flexcan_library_queue2struct(CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  flexcan_library.pop_front(buf, sizeof(CAN_message_t));
  memmove(&msg, buf, sizeof(msg));
}


int IFCT::write(const CAN_message_t &msg, uint8_t mbox) {
  if ( mbox < mailboxOffset() || !(FLEXCAN_get_code(FLEXCANb_MBn_CS(_baseAddress, mbox) >> 3)) ) return 0;
  return write((IFCTMBNUM)mbox,msg);
}


bool IFCT::getFilter(CAN_filter_t &filter, uint8_t mbox) {
  if ( mbox >= mailboxOffset() ) {

    filter.flags.extended = FLEXCANb_MBn_CS(_baseAddress, mbox) & FLEXCAN_MB_CS_IDE;

    filter.id = (FLEXCANb_MBn_ID(_baseAddress, mbox) & FLEXCAN_MB_ID_EXT_MASK);
    if (!filter.flags.extended) filter.id >>= FLEXCAN_MB_ID_STD_BIT_NO;

    filter.flags.remote = FLEXCANb_MBn_CS(_baseAddress, mbox) & FLEXCAN_MB_CS_RTR;
    return 1;
  }
  return 0;
}


void IFCT::setListenOnly(bool mode){
  FLEXCAN_EnterFreezeMode();
  (mode) ? FLEXCANb_CTRL1(_baseAddress) |= FLEXCAN_CTRL_LOM : FLEXCANb_CTRL1(_baseAddress) &= ~FLEXCAN_CTRL_LOM;
  FLEXCAN_ExitFreezeMode();
}





CANListener::CANListener() {
  callbacksActive = 0;
}

bool CANListener::frameHandler(CAN_message_t &/*frame*/, int /*mailbox*/, uint8_t /*controller*/) {
  return 0;
}

void CANListener::txHandler(int /*mailbox*/, uint8_t /*controller*/) {
}

void CANListener::attachMBHandler(uint8_t mailBox) {
  if ( mailBox < 16 ) callbacksActive |= (1UL << mailBox);
}

void CANListener::detachMBHandler(uint8_t mailBox) {
  if ( mailBox < 16 ) callbacksActive &= ~(1UL << mailBox);
}

void CANListener::attachGeneralHandler(void) {
  callbacksActive |= (1UL << 31);
}

void CANListener::detachGeneralHandler(void) {
  callbacksActive &= ~(1UL << 31);
}


bool IFCT::attachObj(CANListener *listener) {
  for (uint32_t i = 0; i < SIZE_LISTENERS; i++) {
    if (this->listener[i] == nullptr) {
      this->listener[i] = listener;
      listener->callbacksActive = 0;
      return true;
    }
  }
  return false;
}

bool IFCT::detachObj(CANListener *listener) {
  for (uint32_t i = 0; i < SIZE_LISTENERS; i++) {
    if (this->listener[i] == listener) {
      this->listener[i] = nullptr;
      return true;
    }
  }
  return false;
}

void IFCT::flexcan_object_oriented_callbacks(CAN_message_t &msg) {
  bool handledFrame = 0;
  uint8_t controller = 0;
  CANListener *thisListener;

  for (uint32_t listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++) {
    thisListener = listener[listenerPos];
    if (thisListener != nullptr) {
      if (thisListener->callbacksActive & (1UL << msg.mb)) handledFrame = thisListener->frameHandler(msg, msg.mb, controller);
      else if (thisListener->callbacksActive & (1UL << 31)) handledFrame = thisListener->frameHandler(msg, -1, controller);
    }
  }
  if (handledFrame == false) flexcan_library_struct2queue(msg);
}

void IFCT::simulate(FLSIMULATE user) {
  flexcan_library_choice = user;
}










// ################################################################################################################
// ####################  /* FLEXCAN_LIBRARY PAWELSKY/TEACHOP COMPATIBILITY MODE */ ################################
// ################################################################################################################

FlexCAN::FlexCAN(uint32_t baud, uint8_t id, uint8_t txAlt, uint8_t rxAlt) {

  if ( !id ) controller = &Can0;
#ifdef __MK66FX1M0__
  else if (id > 0) controller = &Can1;
#endif

  baudRate = baud;
  library = pawelsky;

  rxPin = rxAlt;
  txPin = txAlt;
}

void FlexCAN::begin(const CAN_filter_t &mask) {

  controller->setRX((IFCTALTPIN)!rxPin); controller->setTX((IFCTALTPIN)!rxPin);

  if ( library == pawelsky || library == teachop ) controller->flexcan_library_choice = pawelsky;

  controller->setBaudRate(baudRate);
  controller->enableFIFO();

  controller->FLEXCAN_EnterFreezeMode();
  if (mask.ext) FLEXCANb_RXFGMASK(controller->_baseAddress) = ((mask.rtr?1:0) << 31) | ((mask.ext?1:0) << 30) | ((mask.id & FLEXCAN_MB_ID_EXT_MASK) << 1);
  else FLEXCANb_RXFGMASK(controller->_baseAddress) = ((mask.rtr?1:0) << 31) | ((mask.ext?1:0) << 30) | (FLEXCAN_MB_ID_IDSTD(mask.id) << 1);
  controller->FLEXCAN_ExitFreezeMode();
}

void FlexCAN::setFilter(const CAN_filter_t &filter, uint8_t n) {
  if ( n < 8 ) {
    controller->FLEXCAN_EnterFreezeMode();
    if (filter.ext) FLEXCANb_IDFLT_TAB(controller->_baseAddress, n) = ((filter.rtr?1:0) << 31) | ((filter.ext?1:0) << 30) | ((filter.id & FLEXCAN_MB_ID_EXT_MASK) << 1);
    else FLEXCANb_IDFLT_TAB(controller->_baseAddress, n) = ((filter.rtr?1:0) << 31) | ((filter.ext?1:0) << 30) | (FLEXCAN_MB_ID_IDSTD(filter.id) << 1);
    controller->FLEXCAN_ExitFreezeMode();
  }
}


int FlexCAN::available(void) {
  return (FLEXCANb_IFLAG1(controller->_baseAddress) & FLEXCAN_IMASK1_BUF5M)? 1:0;
}


int FlexCAN::read(CAN_message_t &msg) {
  return controller->readFIFO(msg);
}

int FlexCAN::write(const CAN_message_t &msg) {
  return controller->write(msg);
}
