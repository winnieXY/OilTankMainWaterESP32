#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
//#include <util/atomic.h>
#include <Esp32Atomic.h>
#include <WiFi.h>
#include <driver/adc.h>

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
const lmic_pinmap lmic_pins = {
    .nss =  15,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 33,
    .dio = {32, 27, LMIC_UNUSED_PIN},
    // .rxtx_rx_active = 0,
    // .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    // .spi_freq = 8000000,
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
unsigned int array_counter = 1;

#define DATA_SUMMATION_PERIOD 60000 //Fetch data Every minute
#define DATA_ARRAY_SIZE 5 //Usually transmit after 5 measurements 
#define DATA_PERIOD_EXCEED_ALARM 20 //Transfer data immediately if the data count per period exceeds this value

//Debouncing the interrupt. 
#define DEBOUNCE_TIME 250 //defaulting to 250ms debounce time
unsigned long debouncetime = 0; 

//Watercounter which is used within the interrupt; Number corresponds to liter = 1 Interrupt => 1 SI Unit (e.g. 1 L, 1 W, ...)
volatile unsigned int datacounter = 0;

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
        lpp.reset();
    }
    dprintln(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
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
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              dprint("netid: ");
              dprintln(netid);
              dprint("devaddr: ");
              dprintln(devaddr);
              dprint("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  dprint("-");
                printHex2(artKey[i]);
              }
              dprintln("");
              dprint("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              dprint("-");
                      printHex2(nwkKey[i]);
              }
              dprintln();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     dprintln(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            dprintln(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            dprintln(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            dprintln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              dprintln(F("Received ack"));
            if (LMIC.dataLen) {
              dprint(F("Received "));
              dprint(LMIC.dataLen);
              dprintln(F(" bytes of payload"));
            }
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            dprintln(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            dprintln(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            dprintln(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            dprintln(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            dprintln(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    dprintln(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            dprintln(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            dprintln(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            dprintln(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            dprint(F("Unknown event: "));
            dprintln((unsigned) ev);
            break;
    }
}

/******************************************************************************
 * Interrupt handling function - simply count a value using a debouncer
 *****************************************************************************/
void datacount() {
    unsigned long now = millis();
    if (debouncetime == 0 || now - debouncetime >= DEBOUNCE_TIME ) {
        datacounter++;
        debouncetime = millis();
    }
}

void setup() {
    // put your setup code here, to run once:
    #ifdef DEBUG
        Serial.begin(115200, SERIAL_8N1, 3, 1);
    #endif
    dprintln("Starting communication");

    //Power savings: see https://www.mischianti.org/2021/03/06/esp32-practical-power-saving-manage-wifi-and-cpu-1/
    //and https://github.com/espressif/arduino-esp32/issues/1077
    //Stop Bluetooth for power saving
    btStop();
    //Disable wifi to save energy
    adc_power_release();
    WiFi.mode(WIFI_OFF);

    //Decrease CPU Frequency, we don't need so much computing power
    setCpuFrequencyMhz(80);

    //TODO: Remove later - for debugging:
    randomSeed(analogRead(2));

    //Attach to interrupt 
    attachInterrupt(digitalPinToInterrupt(26), datacount, RISING);

    SPI.begin(14,12,13,15);

    /****************************************************************************
     * Initialize RFM95 LoRa Chip
     ****************************************************************************/
    // LMIC init
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    os_init(); //_ex(pPinMap);
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setAdrMode(1);
    LMIC_setLinkCheckMode(1);

    lpp.reset();
    
    // Start LoRa job (sending automatically starts OTAA too)
    flag_TXCOMPLETE = false;
    do_send(&sendjob);
    dprintln("Finished initialisation!");
}

void loop() {
    // dprintln("Test");
    // Idea: Transmit every 5 Minutes. Transmit an array: 1. Pos = Value of Min 1, 2. Pos = Value of Min 2, 3. Pos = Value of Min 3.... last Pos = current Value
    // This leaves room for an earlier transmit if a measurement value is out of range (e.g. too high)
    unsigned long now = millis();
    if (data_fetch_time == 0 || now - data_fetch_time >= DATA_SUMMATION_PERIOD) {
        dprintln("Get Data and transport it!");

        //Get data
        int datatmp = 0;
        ATOMIC() {
            datatmp = datacounter;
            datacounter = 0;
        }
        datatmp = random(-8,22);
        if (datatmp <0) { datatmp = 0;} 

        lpp.addAnalogOutput(0, DATA_SUMMATION_PERIOD/1000); //0 is the delay between every measurement in seconds
        lpp.addAnalogOutput(array_counter, datatmp);

        //If value exceeds fixed limit transfer directly and do not wait till the array is full
        if (array_counter >= DATA_ARRAY_SIZE || datatmp > DATA_PERIOD_EXCEED_ALARM )  {
            dprintln("Send data to gateway");
            do_send(&sendjob);

            //Reset Counter
            array_counter = 1;
        }
        else {
            //Increase counter if not sending data to gateway
            array_counter++;
        }
   
        //Set time where the next data storage should occur
        data_fetch_time = millis();
    }

    os_runloop_once();
}