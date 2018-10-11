#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include "gps.h"

// T-Beam specific hardware
#define AUX_BUTTON 39
#ifdef TTGO_TBEAM_OLDV
 #define BUILTIN_LED 21 // TBEAM OLD
#else
 #define BUILTIN_LED 14 // TBEAM T22 V07
#endif

char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[9];
gps gps;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

const __FlashStringHelper * sfmod_text(const _dr_eu868_t sfmod) {
  switch(sfmod) {
    case DR_SF12:
  return F("SF12");
    case DR_SF11:
  return F("SF11");
    case DR_SF10:
  return F("SF10");
    case DR_SF9:
  return F("SF9");
    case DR_SF8:
  return F("SF8");
    case DR_SF7:
  return F("SF7");
    case DR_SF7B:
  return F("SF7B");
    case DR_FSK:
  return F("FSK");
    default:
  return F("UNK");
  }
}

_dr_eu868_t get_new_mod() {
      // Alterna modulacao
      int16_t rmod = random(1 << DR_SF7);
      if (LMIC.seqnoUp % 10 == 0) // de 10 em 10 as modulações nao incluem SF7 e SF8
        rmod |= 0x0003;
      int8_t imod;
      _dr_eu868_t mod;
      static _dr_eu868_t lmod;
      for (imod = DR_SF7; rmod & 0x01; --imod)
        rmod >>= 1;
      mod = (_dr_eu868_t)(imod % DR_NONE);
      if (mod < DR_SF9 && lmod < DR_SF9)
        mod = DR_SF9;
      lmod = mod;
      //    _dr_eu868_t mod = (_dr_eu868_t)(LMIC.seqnoUp % DR_NONE);
      return mod;
}

static osjob_t sendjob;
static osjob_t ledjob;
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;

#ifdef TTGO_TBEAM_OLDV
// Pin mapping NGOMES
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {26, 33, 32},
};
#else
// Pin mapping PAMR
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
//  .rst = LMIC_UNUSED_PIN, // was "14,"
  .rst = 23, // TBEAM T22 V07
  .dio = {26, 33, 32},
};
#endif

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      _dr_eu868_t newmod;
      newmod = get_new_mod();
      LMIC_setDrTxpow(newmod, 14);
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks( (DR_SF7B - newmod) * TX_INTERVAL ), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_ledoff(osjob_t* j) {  
      digitalWrite(BUILTIN_LED, LOW);
}

void do_send(osjob_t* j) {  

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    static uint8_t nofixcnt = 0; 
    if (gps.checkGpsFix()) {
      nofixcnt = 0;
      // Prepare upstream data transmission at the next possible time.
      gps.buildPacket(txBuffer);
//      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      LMIC_setTxData2(2, txBuffer, sizeof(txBuffer), 0); // PAMR Compatibilidade com APP TTN
      digitalWrite(BUILTIN_LED, HIGH);
      Serial.print(F("Packet "));
      Serial.print(LMIC.seqnoUp);
      Serial.print(F(" queued at "));
      Serial.println(sfmod_text((_dr_eu868_t)LMIC.datarate));
    }
    else
    {
// Send a dummy message at DR_SF7 to port 16 each 10 try without fix
      if(++nofixcnt % 10 == 0) {
        txBuffer[0] = 0xFF;
        LMIC_setDrTxpow(DR_SF7, 14);
        LMIC_setTxData2(16, txBuffer, 1, 0);
        Serial.print(F(" Dummy packet "));
        Serial.print(LMIC.seqnoUp);
        Serial.print(F(" queued at "));
        Serial.println(sfmod_text((_dr_eu868_t)LMIC.datarate));
      }
      digitalWrite(BUILTIN_LED, HIGH);
      // Led will go off in 100ms
      os_setTimedCallback(&ledjob, os_getTime() + ms2osticks(100), do_ledoff);
      //try again in 3 seconds
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("TTN Mapper"));
  
  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();
  gps.init();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14); 

  do_send(&sendjob);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  pinMode(AUX_BUTTON, INPUT);
}

void loop() {
    os_runloop_once();
}
