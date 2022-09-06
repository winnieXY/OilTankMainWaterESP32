#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
// #include <CayenneLPP.h>
#include <Esp32Atomic.h>
#include <esp_wifi.h>
#include <driver/adc.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <RunningMedian.h>
#include <HardwareSerial.h>
#include <CayenneLPPDecode.h>

#define DEBUG

#ifdef DEBUG
#define dprint(x) Serial.print(x)
#define dprintln(x) Serial.println(x)
#define dwrite(x) Serial.write(x)
#else
#define dprint(x)
#define dprintln(x)
#define dwrite(x)
#endif

//Typedefs
typedef union {
    unsigned int value;
    unsigned char byte[4];
} uint2byte;

/*****************************************************************************
 * LoRaWAN Settings
 ****************************************************************************/
const lmic_pinmap lmic_pins = {
    .nss = 15,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 33,
    .dio = {32, 27, LMIC_UNUSED_PIN},
    // .rxtx_rx_active = 0,
    // .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    // .spi_freq = 8000000,
};

// Cayene LPP Variable
CayenneLPP lpp(51); // 51 would be save, 71 is okay as we send only every 5 minutes.

bool flag_TXCOMPLETE = false;
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above. //New:
static const u1_t PROGMEM DEVEUI[8] = {0x4e, 0x90, 0xb0, 0xfa, 0xc2, 0x45, 0xe2, 0x63};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key. //New: 0x9c, 0x6a, 0x7e, 0xaa, 0xc8, 0x41, 0x1e, 0x0e, 0x3e, 0x2c, 0xbb, 0x5b, 0x6f, 0x84, 0xb0, 0x9c
static const u1_t PROGMEM APPKEY[16] = {0x9c, 0x6a, 0x7e, 0xaa, 0xc8, 0x41, 0x1e, 0x0e, 0x3e, 0x2c, 0xbb, 0x5b, 0x6f, 0x84, 0xb0, 0x9c};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

uint32_t userUTCTime; // Seconds since the UTC epoch

#define TIMEUPDATE_SPAN 86400000 // Once per day
unsigned long timeupdate = 0;

/******************************************************************************
 * Timing & Array Declarations
 *****************************************************************************/
unsigned long int data_fetch_time = 0;
unsigned int array_counter = 1;
unsigned int data_count_sum = 0;

// This defines the default values which are used initially if the EEPROM is empty. If the EEPROM holds an actual
// value the value from the EEPROM is used instead.

#define DATA_SUMMATION_PERIOD 60000 // Default value in ms - Fetch data Every minute
                                    // EEPROM Address "0" -> Data is stored in s and needs to be multiplied by 1000 to get the PERIOD
unsigned int data_summation_period = 0;

#define DATA_ARRAY_SIZE 5 // Default value - Usually transmit after 5 measurement - only transmit faster if on a good datarate
                          // EEPROM Address "1" -> Data is stored as is
unsigned int data_array_size = 0;

#define DATA_PERIOD_EXCEED_ALARM 20              // Default value - transfer data immediately if the data count per period exceeds this value

uint2byte data_period_exceed_alarm;


// Debouncing the interrupt.
#define DEBOUNCE_TIME 250 // defaulting to 250ms debounce time
unsigned long debouncetime = 0;

// Watercounter which is used within the interrupt; Number corresponds to liter = 1 Interrupt => 1 SI Unit (e.g. 1 L, 1 W, ...)
volatile unsigned int datacounter = 0;

/******************************************************************************
 * Begin US-100 Setup
 *****************************************************************************/
HardwareSerial US100Serial(2);
#define RXD2 16
#define TXD2 17

#define DISTANCE_RANGE_BEGIN 1340 // Maximale Distanz in mm (Sensor > Tankboden)
#define LITER_PER_MM 3.68

#define OIL_DELTA_TRANSMIT_TIME 3600 * 1000 // time in milliseconds between two transmits
#define OIL_DELTA_MEASURE_TIME 600 * 1000  * 6  // time in milliseconds between two measurements - every hour 
unsigned long oil_last_transmit_time = 0;
unsigned long oil_last_measure_time = 0;

// US-100 ultrasonic rangefinder:
unsigned int HByte = 0, LByte = 0;
int level = 0, lastLevel = 0, temp = 0, lastTemp = 0, junk, US100temp = 0, Average = 0, Distance = 0;
RunningMedian US100distance = RunningMedian(27);

/******************************************************************************
 * End US-100 Setup
 *****************************************************************************/

/******************************************************************************
 * Begin IR Reader Probe Setup
 *****************************************************************************/

#define WATER_DELTA_MEASURE_TIME 10 // Measure field every 10ms
unsigned long water_last_measure_time = 0;
const int analogInPin = 2;  // Analog input pin that the photo transistor is attached to
const int irOutPin = 22; // Digital output pin that the IR-LED is attached to
const int sdapin = 21;

int sensorValueOff = 0;  // value read from the photo transistor when ir LED is off
int sensorValueOn = 0;  // value read from the photo transistor when ir LED is on
int sensorValue = 0; // difference sensorValueOn - sensorValueOff

int watercount = 0;

// definitions for low pass filter
float filterAlpha = 0.1f;
float filterOut = 0;
boolean filterLoad = true;

// trigger state and level
uint2byte triggerLevelLow;
uint2byte triggerLevelHigh;
boolean autoOptimizeTrigger = true;

boolean triggerState = false;


#define AUTO_TRIGGER_COUNT 200
RunningMedian lowestValuesToTrigger = RunningMedian(AUTO_TRIGGER_COUNT);
RunningMedian highestValuesToTrigger = RunningMedian(AUTO_TRIGGER_COUNT);
double lastValLow = NAN;
double lastValHigh = NAN;

/******************************************************************************
 * End Magnetometer Setup
 *****************************************************************************/

#define EEPROM_BEGIN_DATA_AUTO_TRIGGER 0
#define EEPROM_BEGIN_TRIGGERLOW 4
#define EEPROM_BEGIN_TRIGGERHIGH 8
#define EEPROM_BEGIN_WATER_EXCEED_LIMIT 12



/******************************************************************************
 * Lorawan Functions
 *****************************************************************************/
void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess)
{
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *)pVoidUserUTCTime;

    // A struct that will be populated by LMIC_getNetworkTimeReference.
    // It contains the following fields:
    //  - tLocal: the value returned by os_GetTime() when the time
    //            request was sent to the gateway, and
    //  - tNetwork: the seconds between the GPS epoch and the time
    //              the gateway received the time request
    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1)
    {
        dprintln(F("USER CALLBACK: Not a success"));
        return;
    }

    // Populate "lmic_time_reference"
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1)
    {
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

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        dprintln(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        unsigned long now = millis();
        if (timeupdate == 0 || now - timeupdate >= TIMEUPDATE_SPAN)
        {
            // Schedule a network time request at the next possible time
            LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);
            timeupdate = millis();
        }
        // Set flag that a  transmit is ongoing
        flag_TXCOMPLETE = false;
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        lpp.reset();
    }
    dprintln(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
}

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

// Only partially used - the DATA_ARRAY_SIZE is calculated based on the datarate used for transmission.
// the exceed alarm is just needed with a bad data rate - otherwise it is ignored as we transmit frequently
// enough.
void parseDownstream(u1_t frame[255], u1_t databeg, u1_t dataLen)
{
    DynamicJsonDocument jsonBuffer(256);
    CayenneLPPDecode lppd;
    bool modify = false;

    JsonObject root = jsonBuffer.to<JsonObject>();

    lppd.write(frame+databeg, dataLen);

    if (lppd.isValid()) {
        //parse
        lppd.decode(root);
        dprintln(root);
        if (root.containsKey("digital_in_0")) { //Automatic trigger function
            autoOptimizeTrigger = root["digital_in_0"];
            EEPROM.write(EEPROM_BEGIN_DATA_AUTO_TRIGGER, autoOptimizeTrigger);
            modify = true;
            dprint("Got AutoOptimizeTrigger via Downstream: ["); dprint(autoOptimizeTrigger); dprintln("]");
        }
        if (root.containsKey("luminosity_1")) { //Low Trigger Value
            triggerLevelLow.value = root["luminosity_1"];
            EEPROM.write(EEPROM_BEGIN_TRIGGERLOW, triggerLevelLow.byte[0]);
            EEPROM.write(EEPROM_BEGIN_TRIGGERLOW + 1, triggerLevelLow.byte[1]);
            EEPROM.write(EEPROM_BEGIN_TRIGGERLOW + 2, triggerLevelLow.byte[2]);
            EEPROM.write(EEPROM_BEGIN_TRIGGERLOW + 3, triggerLevelLow.byte[3]);
            modify = true;
            dprintln("Got Low Level Trigger Value via Downstream: ["); dprint(triggerLevelLow.value); dprintln("]");
        }
        if (root.containsKey("luminosity_2")) {
            triggerLevelHigh.value = root["luminosity_2"];
            EEPROM.write(EEPROM_BEGIN_TRIGGERHIGH, triggerLevelHigh.byte[0]);
            EEPROM.write(EEPROM_BEGIN_TRIGGERHIGH + 1, triggerLevelHigh.byte[1]);
            EEPROM.write(EEPROM_BEGIN_TRIGGERHIGH + 2, triggerLevelHigh.byte[2]);
            EEPROM.write(EEPROM_BEGIN_TRIGGERHIGH + 3, triggerLevelHigh.byte[3]);
            modify = true;
            dprintln("Got High Level Trigger Value via Downstream:"); dprint(triggerLevelHigh.value); dprintln("]");
        }
        if (root.containsKey("luminosity_3")) {
            data_period_exceed_alarm.value = root["luminosity_3"];
            EEPROM.write(EEPROM_BEGIN_WATER_EXCEED_LIMIT, data_period_exceed_alarm.byte[0]);
            EEPROM.write(EEPROM_BEGIN_WATER_EXCEED_LIMIT + 1, data_period_exceed_alarm.byte[1]);
            EEPROM.write(EEPROM_BEGIN_WATER_EXCEED_LIMIT + 2, data_period_exceed_alarm.byte[2]);
            EEPROM.write(EEPROM_BEGIN_WATER_EXCEED_LIMIT + 3, data_period_exceed_alarm.byte[3]);
            modify = true;
            dprintln("Got Exceed Alarm Limit via Downstream:"); dprint(data_period_exceed_alarm.value); dprintln("]");
        }
        if (modify) {
            EEPROM.commit();
        }

    }
    else {
        dprintln("Recieved downlink is no valid cayenneLPP!");
    }
}

void onEvent(ev_t ev)
{
    dprint(os_getTime());
    dprint(": ");
    switch (ev)
    {
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
        flag_TXCOMPLETE = true;
        break;
    case EV_JOINED:
        dprintln(F("EV_JOINED"));
        flag_TXCOMPLETE = true;
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
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    dprint("-");
                printHex2(artKey[i]);
            }
            dprintln("");
            dprint("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    dprint("-");
                printHex2(nwkKey[i]);
            }
            dprintln();
        }
        break;
    case EV_JOIN_FAILED:
        dprintln(F("EV_JOIN_FAILED"));
        flag_TXCOMPLETE = true;
        break;
    case EV_REJOIN_FAILED:
        dprintln(F("EV_REJOIN_FAILED"));
        flag_TXCOMPLETE = true;
        break;
    case EV_TXCOMPLETE:
        dprintln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        flag_TXCOMPLETE = true;
        if (LMIC.txrxFlags & TXRX_ACK)
            dprintln(F("Received ack"));
        if (LMIC.dataLen)
        {
            dprint(F("Received "));
            dprint(LMIC.dataLen);
            dprintln(F(" bytes of payload"));

            parseDownstream(LMIC.frame, LMIC.dataBeg, LMIC.dataLen);
        }
        // Calculate next DATA_ARRAY_SIZE based on SF level for next transmission
        // SF7 & 8: => 1
        // SF9 & 10: => 2
        // SF11 & 12: => 5
        switch (LMIC.datarate)
        {
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
        flag_TXCOMPLETE = true;
        break;
    case EV_RESET:
        dprintln(F("EV_RESET"));
        flag_TXCOMPLETE = true;
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
        flag_TXCOMPLETE = true;
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        dprintln(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        flag_TXCOMPLETE = true;
        break;

    default:
        dprint(F("Unknown event: "));
        dprintln((unsigned)ev);
        break;
    }
}

/******************************************************************************
 * Interrupt handling function - simply count a value using a debouncer
 *****************************************************************************/
void datacount()
{
    unsigned long now = millis();
    if (debouncetime == 0 || now - debouncetime >= DEBOUNCE_TIME)
    {
        datacounter++;
        debouncetime = millis();
    }
}

void messung() // Sensor abfragen
{
    // Serielle Schnittstelle für US-100 öffnen
    US100Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(100);
    int DataToAvg = 15;
    for (int avgloop = 1; avgloop < (DataToAvg + 1); avgloop++)
    {
        while (US100Serial.available())
        {
            US100Serial.read();
        } // Clear the US100Serial buffer.
        dprintln("write 0x55");
        US100Serial.write(0x55); // Send a "distance measure" command to US-100
        delay(200);              // US100 response time depends on distance.
        dprint("available..");
        if (US100Serial.available() >= 2) // at least 2 bytes are in buffer
        {
            dprintln("yes");
            HByte = US100Serial.read(); // Read both bytes
            LByte = US100Serial.read();
            Distance = (HByte * 256 + LByte);
            delay(200);
        }
        // Serial.println(".");
        dprintln("Read " + String(Distance) + "mm");
        US100distance.add(Distance);
    }

    Average = US100distance.getAverage(27);

    level = (DISTANCE_RANGE_BEGIN - Average);

    // Read temperature from the US-100 ultrasonic rangefinder's temp sensor at the top of the tank. The tank air heats up in the sun.
    while (US100Serial.available())
    {
        US100Serial.read();
    }
    US100Serial.write(0x50); // send command to request temperature byte.
    delay(50);               // temp response takes about 2ms after command ends.
    if (US100Serial.available() >= 1)
    {
        US100temp = US100Serial.read();
        if ((US100temp > 1) && (US100temp < 130))
        {
            US100temp -= 45; // Correct by the 45 degree offset of the US100.
        }
    }
    US100Serial.end();
    temp = US100temp;
    dprintln("Temp is " + String(temp) + "°C");
}

/*
 * Low pass filter to eleminate spikes
 */
float lowpass(int value)
{
    if (filterLoad)
    {
        filterOut = value;
        filterLoad = false;
    }
    filterOut = filterAlpha * value + (1.f - filterAlpha) * filterOut;
    return filterOut;
}

// If the value is higher than triggerLevelHigh and the previous state was low the return value is "1"
// if the value is lower than triggerLevelLow and the previous state was high the return value is "0"
// otherwise the return value is "-1"
int detectTrigger(float val)
{
    int tmp = -1;
    boolean nextState = triggerState;
    if (val > triggerLevelHigh.value)
    {
        nextState = true;
    }
    else if (val < triggerLevelLow.value)
    {
        nextState = false;
    }
    if (nextState != triggerState)
    {
        triggerState = nextState;
        tmp = triggerState;
    }
    return tmp;
}

//Detect local maxima and local minima in a floating manner
void automodifyTriggers(float val)
{
    bool changed = false;
    if (val < lastValLow || lastValLow == NAN)
    {
        lastValLow = val;
    }
    if (val > lastValHigh || lastValHigh == NAN)
    {
        lastValHigh = val;
    }
    //Define the value as new local minima if the value if the current reading is 5% bigger than the last smallest reading 
    if (val > lastValLow*1.05  && lastValHigh != NAN && lastValLow != NAN)
    {
        lowestValuesToTrigger.add(lastValLow);
        lastValLow = NAN;
    }
    //Define the value as new local maxima if the value if the current reading is 5% smaller than the last biggest reading 
    if (val < lastValHigh*0.95 && lastValHigh != NAN && lastValLow != NAN)
    {
        highestValuesToTrigger.add(lastValHigh);
        lastValHigh = NAN;
    }
    if (lowestValuesToTrigger.getCount() == AUTO_TRIGGER_COUNT && highestValuesToTrigger.getCount() == AUTO_TRIGGER_COUNT)
    {
        float lowerAverage = lowestValuesToTrigger.getAverage();
        float higherAverage = highestValuesToTrigger.getAverage();

        float divlowHigh = higherAverage - lowerAverage;
        float triggerLevelHighNew = higherAverage - divlowHigh * 0.3;
        float triggerLevelLowNew = lowerAverage + divlowHigh * 0.3;

        char str[100];
        if (abs(triggerLevelLowNew - triggerLevelLow.value) / triggerLevelLowNew > 0.1)
        {
            triggerLevelLow.value = triggerLevelLowNew;
            lpp.addAnalogInput(DATA_ARRAY_SIZE + 4, triggerLevelLow.value);

            EEPROM.write(6,triggerLevelLow.byte[0]);
            EEPROM.write(7,triggerLevelLow.byte[1]);
            EEPROM.write(8,triggerLevelLow.byte[2]);
            EEPROM.write(9,triggerLevelLow.byte[3]);
            changed = true;

        }
        if (abs(triggerLevelHighNew - triggerLevelHigh.value) / triggerLevelHighNew > 0.1)
        {
            triggerLevelHigh.value = triggerLevelHighNew;
            lpp.addAnalogInput(DATA_ARRAY_SIZE + 5, triggerLevelHigh.value);
            
            EEPROM.write(10, triggerLevelHigh.byte[0]);
            EEPROM.write(11, triggerLevelHigh.byte[1]);
            EEPROM.write(12, triggerLevelHigh.byte[2]);
            EEPROM.write(13, triggerLevelHigh.byte[3]);
            changed = true;
        }
        if (changed) 
        {
            EEPROM.commit();
        }
    }
}

// Perform a measurement and detect a trigger. "-1" if no trigger is detected, otherwise 0 or 1.
int irprobe_measurement()
{
    digitalWrite(irOutPin, LOW);
    // wait 10 milliseconds
    delay(10);
    // read the analog in value:
    sensorValueOff = analogRead(analogInPin);           
    // turn IR LED on
    digitalWrite(irOutPin, HIGH);
    delay(10);
    // read the analog in value:
    sensorValueOn = analogRead(analogInPin);
    sensorValue = sensorValueOn - sensorValueOff;

    float value = lowpass(sensorValue);
    // dprint(sensorValue);
    // dprint(",");
    // dprint(value);
    // dprint(",");
    // dprint(triggerLevelHigh.value);
    // dprint(",");
    // dprintln(triggerLevelLow.value);

    int tmp = detectTrigger(value);
    if (autoOptimizeTrigger) //Can be set via lora
        automodifyTriggers(value);
    if (tmp == 1)
    {
        return tmp;
    }
    return 0;
}

void setup()
{
// //Erase EEPROM once, after that delete that loop
//EEPROM.begin(512);
// for(int i = 0; i < 512; i++){
//     EEPROM.write(i,255);
// }
//EEPROM.commit();
// while (true) {}

// put your setup code here, to run once:
#ifdef DEBUG
    Serial.begin(115200, SERIAL_8N1, 3, 1);
#endif
    dprintln("Starting communication");

    // Start up the EEPROM
    EEPROM.begin(20);

    /**************************************************************************
     * Read in EEPROM with values from last run:
     *************************************************************************/

    autoOptimizeTrigger = EEPROM.read(EEPROM_BEGIN_DATA_AUTO_TRIGGER);

    //Read in the trigger levels for the magnetometer
    triggerLevelLow.byte[0] = EEPROM.read(EEPROM_BEGIN_TRIGGERLOW);
    triggerLevelLow.byte[1] = EEPROM.read(EEPROM_BEGIN_TRIGGERLOW + 1);
    triggerLevelLow.byte[2] = EEPROM.read(EEPROM_BEGIN_TRIGGERLOW + 2);
    triggerLevelLow.byte[3] = EEPROM.read(EEPROM_BEGIN_TRIGGERLOW + 3);

    triggerLevelHigh.byte[0] = EEPROM.read(EEPROM_BEGIN_TRIGGERHIGH);
    triggerLevelHigh.byte[1] = EEPROM.read(EEPROM_BEGIN_TRIGGERHIGH + 1);
    triggerLevelHigh.byte[2] = EEPROM.read(EEPROM_BEGIN_TRIGGERHIGH + 2);
    triggerLevelHigh.byte[3] = EEPROM.read(EEPROM_BEGIN_TRIGGERHIGH + 3);

    data_period_exceed_alarm.byte[0] = EEPROM.read(EEPROM_BEGIN_WATER_EXCEED_LIMIT);
    data_period_exceed_alarm.byte[1] = EEPROM.read(EEPROM_BEGIN_WATER_EXCEED_LIMIT + 1);
    data_period_exceed_alarm.byte[2] = EEPROM.read(EEPROM_BEGIN_WATER_EXCEED_LIMIT + 2);
    data_period_exceed_alarm.byte[3] = EEPROM.read(EEPROM_BEGIN_WATER_EXCEED_LIMIT + 3);
    
    //Reset values to some sane default values if nothing is stored in eeprom
    if (triggerLevelLow.byte[0] == 0xFF 
    && triggerLevelLow.byte[1] == 0xFF 
    && triggerLevelLow.byte[2] == 0xFF 
    && triggerLevelLow.byte[3] == 0xFF) 
    {
        triggerLevelLow.value = 100; 
    }
    if (triggerLevelHigh.byte[0] == 0xFF 
        && triggerLevelHigh.byte[1] == 0xFF 
        && triggerLevelHigh.byte[2] == 0xFF 
        && triggerLevelHigh.byte[3] == 0xFF) 
    {
        triggerLevelHigh.value = 4900; 
    }
    
    if (data_period_exceed_alarm.byte[0] == 0xFF 
        && data_period_exceed_alarm.byte[1] == 0xFF 
        && data_period_exceed_alarm.byte[2] == 0xFF 
        && data_period_exceed_alarm.byte[3] == 0xFF) 
    {
        data_period_exceed_alarm.value = DATA_PERIOD_EXCEED_ALARM; 
    }


    // Power savings: see https://www.mischianti.org/2021/03/06/esp32-practical-power-saving-manage-wifi-and-cpu-1/
    // and https://github.com/espressif/arduino-esp32/issues/1077
    // Stop Bluetooth for power saving
    btStop();
    // Disable wifi to save energy
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    // Decrease CPU Frequency, we don't need so much computing power
    setCpuFrequencyMhz(80);

    pinMode(26, INPUT_PULLUP);

    pinMode(irOutPin, OUTPUT);

    // Attach to interrupt
    attachInterrupt(digitalPinToInterrupt(26), datacount, RISING);


    /****************************************************************************
     * Initialize RFM95 LoRa Chip
     ****************************************************************************/
    SPI.begin(14, 12, 13, 15);
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

    do_send(&sendjob);
    dprintln("Finished initialisation!");
}

void loop() 
{
    unsigned long now = millis();
    // Do nothing which could ruin the lora transmission - Just wait till the flag is set to true again.
    if (flag_TXCOMPLETE && (oil_last_measure_time == 0 || now - oil_last_measure_time >= OIL_DELTA_MEASURE_TIME))
    {
        messung();
        oil_last_measure_time = millis();
    }
    /**************************************************************************
     * Measure the magnetic field if no transmission is pending to not ruin the transmission
     *************************************************************************/
    if (flag_TXCOMPLETE && (water_last_measure_time == 0 || now - water_last_measure_time >= WATER_DELTA_MEASURE_TIME))
    {
        watercount += irprobe_measurement();
    }

    if (data_fetch_time == 0 || now - data_fetch_time >= DATA_SUMMATION_PERIOD)
    {
        dprintln("Get Data!");

        // Get data
        int datatmp = 0;
        ATOMIC()
        {
            datatmp = datacounter;
            datacounter = 0;
        }

        // Use the watercount instead of the interrupt
        data_count_sum += watercount;

        lpp.addAnalogOutput(0, DATA_SUMMATION_PERIOD/1000); //0 is the delay between every measurement in seconds
        lpp.addAnalogOutput(array_counter, watercount);

        //For testing purposes
        lpp.addAnalogInput(DATA_ARRAY_SIZE + 5, triggerLevelHigh.value);
        lpp.addAnalogInput(DATA_ARRAY_SIZE + 4, triggerLevelLow.value);
        lpp.addDigitalInput(DATA_ARRAY_SIZE + 6, autoOptimizeTrigger);


        // Transmit the oil values just every hour - we do not need these values more often
        if (oil_last_transmit_time == 0 || now - oil_last_transmit_time >= OIL_DELTA_TRANSMIT_TIME)
        {
            lpp.addTemperature(DATA_ARRAY_SIZE + 1,temp);
            lpp.addAnalogOutput(DATA_ARRAY_SIZE + 2, level);
            lpp.addAnalogOutput(DATA_ARRAY_SIZE + 3, (level * LITER_PER_MM));
            oil_last_transmit_time = millis();
        }

        // Transmit the value only if the sum is != 0 and the arraycounter is bigger than it should be (this is depended of the Lora DataRate)
        // OR transmit if the array_counter is bigger than it's max allowed size (defaults currently to 5)
        // OR transmit directly if the measured data is bigger than a user defined treshhold
        //
        // Especially the first check prevents transfers of data without any need as zero measurements will be delayed until the array size is filled up till maximum
        // So if there is no flow at all the data will be transferred only every 5 minutes. With DR5 (SF7) we are allowed to transmit ~18 messages per hour on average
        //(every three minutes) to fullfill the TTN fair usage policy. This change will allow us to do so if there is not that much flow most of the day.
        if ((array_counter >= data_array_size && data_count_sum != 0) || array_counter > DATA_ARRAY_SIZE || datatmp > data_period_exceed_alarm.value)
        {
            dprintln("Send data to gateway");
            do_send(&sendjob);

            // Reset Counter
            array_counter = 1;
            // Reset Data Count Summation
            data_count_sum = 0;
            watercount = 0;
        }
        else
        {
            // Increase counter if not sending data to gateway
            array_counter++;
        }

        // Set time where the next data storage should occur
        data_fetch_time = millis();
    }

    os_runloop_once();
}