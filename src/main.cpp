#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>

#define DEBUG

#ifdef DEBUG
#define dprint(x)     Serial.print(x)
#define dprintln(x)   Serial.println(x)
#define dwrite(x)     Serial.write(x)
#else
#define dprint(x)
#define dprintln(x)
#define dwrite(x)
#endif


/*****************************************************************************
 * LoRaWAN Settings
 ****************************************************************************/
//Cayene LPP Variable
CayenneLPP lpp(51); //51 would be save, 71 is okay as we send only every 5 minutes. 

bool flag_TXCOMPLETE = false;
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xfd, 0xfd, 0x47, 0x76, 0x82, 0x08, 0x8f, 0x67 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xb6, 0x60, 0x22, 0xc9, 0x01, 0x52, 0xc4, 0x0e, 0x47, 0xf1, 0x58, 0xe5, 0x9b, 0xcf, 0xf0, 0x46 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

//TODO: Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 38,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 32,
    .dio = {34, 36, 37},
};


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        dprintln(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    }
    dprintln(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    dprint(os_getTime());
    dprint(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            dprintln(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            dprintln(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            dprintln(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            dprintln(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            dprintln(F("EV_JOINING"));
            break;
        case EV_JOINED:
            dprintln(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            flag_TXCOMPLETE = true;
            break;
        case EV_RFU1:
            dprintln(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            dprintln(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            dprintln(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            dprintln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              dprintln(F("Received ack"));
            if (LMIC.dataLen) {
              dprintln(F("Received "));
              dprintln(LMIC.dataLen);
              dprintln(F(" bytes of payload"));
            }
            flag_TXCOMPLETE = true;
            break;
        case EV_LOST_TSYNC:
            dprintln(F("EV_LOST_TSYNC"));
            flag_TXCOMPLETE= true;
            break;
        case EV_RESET:
            dprintln(F("EV_RESET"));
            flag_TXCOMPLETE = true;
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            dprintln(F("EV_RXCOMPLETE"));
            flag_TXCOMPLETE = true;
            break;
        case EV_LINK_DEAD:
            dprintln(F("EV_LINK_DEAD"));
            flag_TXCOMPLETE = true;
            break;
        case EV_LINK_ALIVE:
            dprintln(F("EV_LINK_ALIVE"));
            flag_TXCOMPLETE = true;
            break;
         default:
            dprintln(F("Unknown event"));
            flag_TXCOMPLETE = true;
            break;
    }
}


void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  dprintln("Starting communication");

  /****************************************************************************
   * Initialize RFM95 LoRa Chip
   ****************************************************************************/
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  lpp.reset();

  // Start LoRa job (sending automatically starts OTAA too)
  flag_TXCOMPLETE = false;
  do_send(&sendjob);


}

void loop() {
  // put your main code here, to run repeatedly:


  while(!flag_TXCOMPLETE) { //this flag is set to false each time before a sendjob is scheduled. it is set back to true after a successful txcomplete.
    os_runloop_once();
  }
}