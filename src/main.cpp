#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <Esp32Atomic.h>
#include <esp_wifi.h>
#include <driver/adc.h>
#include <EEPROM.h>
#include <TimeLib.h>


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

uint32_t userUTCTime; // Seconds since the UTC epoch

#define TIMEUPDATE_SPAN 86400000 //Once per day
unsigned long timeupdate = 0;


/******************************************************************************
 * Timing & Array Declarations
 *****************************************************************************/
unsigned long int data_fetch_time = 0;
unsigned int array_counter = 1;
unsigned int data_count_sum = 0;

//This defines the default values which are used initially if the EEPROM is empty. If the EEPROM holds an actual
//value the value from the EEPROM is used instead.

#define DATA_SUMMATION_PERIOD 60000                 //Default value in ms - Fetch data Every minute
                                                    //EEPROM Address "0" -> Data is stored in s and needs to be multiplied by 1000 to get the PERIOD
unsigned int data_summation_period = 0;

#define DATA_ARRAY_SIZE 5                           //Default value - Usually transmit after 5 measurement - only transmit faster if on a good datarate
                                                    //EEPROM Address "1" -> Data is stored as is
unsigned int data_array_size = 0;

#define DATA_PERIOD_EXCEED_ALARM 20                 //Default value - ransfer data immediately if the data count per period exceeds this value
#define DATA_PERIOD_EXCEED_ALARM_MULTIPLICATOR 1    //EEPROM Address "2" & "3"
                                                    //Value in Address "2" is multiplied with Value in Address "3"
                                                    //e.g. "20" x "1" => 20
                                                    //or   "20" x "10" => 200
unsigned int data_period_exceed_alarm = 0;
unsigned int data_period_exceed_alarm_multiplicator = 0;

//Debouncing the interrupt. 
#define DEBOUNCE_TIME 250 //defaulting to 250ms debounce time
unsigned long debouncetime = 0; 

//Watercounter which is used within the interrupt; Number corresponds to liter = 1 Interrupt => 1 SI Unit (e.g. 1 L, 1 W, ...)
volatile unsigned int datacounter = 0;

/******************************************************************************
 * Lorawan Functions
 *****************************************************************************/
void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

    // A struct that will be populated by LMIC_getNetworkTimeReference.
    // It contains the following fields:
    //  - tLocal: the value returned by os_GetTime() when the time
    //            request was sent to the gateway, and
    //  - tNetwork: the seconds between the GPS epoch and the time
    //              the gateway received the time request
    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1) {
        dprintln(F("USER CALLBACK: Not a success"));
        return;
    }

    // Populate "lmic_time_reference"
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        dprintln(F("USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed"));
        return;
    }

    // Update userUTCTime, considering the difference between the GPS and UTC
    // epoch, and the leap seconds
    *pUserUTCTime = lmicTimeReference.tNetwork + 315964800;

    // Add the delay between the instant the time was transmitted and
    // the current time

    // Current time, in ticks
    ostime_t ticksNow = os_getTime();
    // Time when the request was sent, in ticks
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;

    // Update the system time with the time read from the network
    setTime(*pUserUTCTime);

    dprint(F("The current UTC time is: "));
    dprint(hour());
    dprint(minute());
    dprint(second());
    dprint(' ');
    dprint(day());
    dprint('/');
    dprint(month());
    dprint('/');
    dprint(year());
    dprintln();
}




void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        dprintln(F("OP_TXRXPEND, not sending"));
    } else {
        unsigned long now = millis();
        if (timeupdate == 0 || now - timeupdate >= TIMEUPDATE_SPAN ) {
            // Schedule a network time request at the next possible time
            LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);
            timeupdate = millis();
        }
        
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

short EEPROM_get(int pos, int defaultVal) {
    int tmp = EEPROM.read(pos);
    if (tmp == 255) return defaultVal;
    return tmp;
}

void EEPROM_put(int pos, int value) {
    EEPROM.write(pos, value);
}

// Only partially used - the DATA_ARRAY_SIZE is calculated based on the datarate used for transmission.
// the exceed alarm is just needed with a bad data rate - otherwise it is ignored as we transmit frequently
// enough.
void parseDownstream(u1_t frame[255], u1_t databeg, u1_t dataLen) {

    dprintln(dataLen);
    if (dataLen = 2) {
        dprintln(frame[databeg]);
        dprintln(frame[databeg+1]);
        unsigned int tmp = ( frame[databeg] << 8 ) + frame[databeg+1];
        dprintln(tmp);
        data_period_exceed_alarm_multiplicator = 1;
        while (tmp > 254) {
            data_period_exceed_alarm_multiplicator *= 10;
            tmp = tmp / data_period_exceed_alarm_multiplicator;
        }
        data_period_exceed_alarm = tmp * data_period_exceed_alarm_multiplicator;
        EEPROM_put(2, tmp);
        EEPROM_put(3, data_period_exceed_alarm_multiplicator);
    }
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
            break;
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

              parseDownstream(LMIC.frame, LMIC.dataBeg, LMIC.dataLen);
            }
            //Calculate next DATA_ARRAY_SIZE based on SF level for next transmission
            //SF7 & 8: => 1
            //SF9 & 10: => 2
            //SF11 & 12: => 5
            switch (LMIC.datarate) {
                case LORAWAN_DR5:
                case LORAWAN_DR4:
                    data_array_size = 1;
                    break;
                case LORAWAN_DR3:
                case LORAWAN_DR2:
                    data_array_size = 2;
                    break;
                default:
                    data_array_size = DATA_ARRAY_SIZE;
                    break;
            }
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
    // //Erase EEPROM once, after that delete that loop
    // for(int i = 0; i < 512; i++){
    //     EEPROM.write(i,255);
    // }
    // while (true) {}

    // put your setup code here, to run once:
    #ifdef DEBUG
        Serial.begin(115200, SERIAL_8N1, 3, 1);
    #endif
    dprintln("Starting communication");

    /**************************************************************************
     * Read in EEPROM with values from last run:
     *************************************************************************/

    data_period_exceed_alarm = EEPROM_get(2, DATA_PERIOD_EXCEED_ALARM);
    data_period_exceed_alarm_multiplicator = EEPROM_get(3, DATA_PERIOD_EXCEED_ALARM_MULTIPLICATOR);
    data_period_exceed_alarm *= data_period_exceed_alarm_multiplicator;


    //Power savings: see https://www.mischianti.org/2021/03/06/esp32-practical-power-saving-manage-wifi-and-cpu-1/
    //and https://github.com/espressif/arduino-esp32/issues/1077
    //Stop Bluetooth for power saving
    btStop();
    //Disable wifi to save energy
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    //Decrease CPU Frequency, we don't need so much computing power
    setCpuFrequencyMhz(80);

    pinMode(26, INPUT_PULLUP);
    
    //Attach to interrupt 
    attachInterrupt(digitalPinToInterrupt(26), datacount, RISING);

    /****************************************************************************
     * Initialize RFM95 LoRa Chip
     ****************************************************************************/
    SPI.begin(14,12,13,15);
    // LMIC init
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    os_init();
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setAdrMode(1);
    LMIC_setLinkCheckMode(1);
    LMIC_setBatteryLevel(MCMD_DEVS_EXT_POWER);

    lpp.reset();
    
    // Start LoRa job (sending automatically starts OTAA too)
    flag_TXCOMPLETE = false;
    do_send(&sendjob);
    dprintln("Finished initialisation!");
}

void loop() {
    unsigned long now = millis();
    if (data_fetch_time == 0 || now - data_fetch_time >= DATA_SUMMATION_PERIOD) {
        dprintln("Get Data and transport it!");

        //Get data
        int datatmp = 0;
        ATOMIC() {
            datatmp = datacounter;
            datacounter = 0;
        }
        data_count_sum += datatmp;

        lpp.addAnalogOutput(0, DATA_SUMMATION_PERIOD/1000); //0 is the delay between every measurement in seconds
        lpp.addAnalogOutput(array_counter, datatmp);

        //Transmit the value only if the sum is != 0 and the arraycounter is bigger than it should be (this is depended of the Lora DataRate)
        //OR transmit if the array_counter is bigger than it's max allowed size (defaults currently to 5)
        //OR transmit directly if the measured data is bigger than a user defined treshhold
        //
        //Especially the first check prevents transfers of data without any need as zero measurements will be delayed until the array size is filled up till maximum
        //So if there is no flow at all the data will be transferred only every 5 minutes. With DR5 (SF7) we are allowed to transmit ~18 messages per hour on average 
        //(every three minutes) to fullfill the TTN fair usage policy. This change will allow us to do so if there is not that much flow most of the day. 
        if ((array_counter >= data_array_size && data_count_sum != 0) || array_counter > DATA_ARRAY_SIZE || datatmp > DATA_PERIOD_EXCEED_ALARM )  {
            dprintln("Send data to gateway");
            do_send(&sendjob);

            //Reset Counter
            array_counter = 1;
            //Reset Data Count Summation
            data_count_sum = 0;
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