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
//TODO: Pin mapping
const lmic_pinmap lmic_pins = {
    .nss =  15,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 33,
    .dio = {32, 27, LMIC_UNUSED_PIN},
};

//Cayene LPP Variable
CayenneLPP lpp(51); //51 would be save, 71 is okay as we send only every 5 minutes. 

bool flag_TXCOMPLETE = false;
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above. //New:
static const u1_t PROGMEM DEVEUI[8]={ 0x85, 0xa0, 0x6d, 0x84, 0xd4, 0x04, 0x51, 0xc1 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key. //New: 0xd0, 0xc9, 0x55, 0xae, 0x71, 0x2d, 0xb0, 0x64, 0x5a, 0x6a, 0xac, 0x3b, 0xf3, 0xde, 0x0e, 0x8e
static const u1_t PROGMEM APPKEY[16] = { 0xd0, 0xc9, 0x55, 0xae, 0x71, 0x2d, 0xb0, 0x64, 0x5a, 0x6a, 0xac, 0x3b, 0xf3, 0xde, 0x0e, 0x8e };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

/******************************************************************************
 * Timing & Array Declarations
 *****************************************************************************/
unsigned long int data_fetch_time = 0;
unsigned long int data_transmit_time = 0;
unsigned int counter = 1;

#define DATA_FETCH_DELAY 60000 //Fetch data Every minute
#define DATA_TRANSMIT_WINDOW 300000 //Transmit every five minutes



/******************************************************************************
 * Lorawan Functions
 *****************************************************************************/
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        dprintln(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        //Reset Counter
        counter = 1;
        lpp.reset();
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

    data_transmit_time = millis() + DATA_TRANSMIT_WINDOW;

}

void loop() {
    if (data_fetch_time == 0 || data_fetch_time <= millis() || data_fetch_time + DATA_FETCH_DELAY > millis()) {
        //Get data
        lpp.addAnalogOutput(0, DATA_FETCH_DELAY); //0 is the delay between every measurement
        lpp.addGenericSensor(counter, 1);

        //If value exceeds fixed limit transfer directly and do not wait till the array is full
        if (false) {
            do_send(&sendjob);
            data_transmit_time = millis() + DATA_TRANSMIT_WINDOW;
        }

        //Increase counter
        counter++;

        //Set time where
        data_fetch_time = millis() + DATA_FETCH_DELAY;
    }
    //Send 
    if (data_transmit_time == 0 || data_transmit_time <= millis() || data_transmit_time + DATA_FETCH_DELAY > millis()) {

        do_send(&sendjob);
        data_transmit_time = millis() + DATA_TRANSMIT_WINDOW;
    }


    // put your main code here, to run repeatedly:
    // Idea: Transmit every 5 Minutes. Transmit an array: 1. Pos = Value of Min 1, 2. Pos = Value of Min 2, 3. Pos = Value of Min 3.... last Pos = current Value
    // This leaves room for an earlier transmit if a measurement value is out of range (e.g. too high)
    //lpp.addAnalogInput(1,)

    while(!flag_TXCOMPLETE) { //this flag is set to false each time before a sendjob is scheduled. it is set back to true after a successful txcomplete.
        os_runloop_once();
    }
}