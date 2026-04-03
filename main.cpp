// BMW F32 435i N55 Boost Gauge — BLE OBD2 via KW901
// Based on: fbiego/car-hud (proven working on VIEWE SmartRing)
// Display: VIEWE SmartRing 1.75" 466x466 AMOLED

#include <Arduino.h>
#include <USB.h>
#include <USBCDC.h>
#include <math.h>
#include <NimBLEDevice.h>

#include "viewe.hpp"

USBCDC USBSerial;

// ── Screen ─────────────────────────────────────────────
#define CX  233
#define CY  233

// ── Colors ─────────────────────────────────────────────
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_BMW     0xFAA0   // orange
#define C_BMW_DIM 0x50E0   // dim orange
#define C_RED     rgb565(220, 40, 40)
#define C_BLUE    rgb565(28, 139, 212)
#define C_GREEN   rgb565(29, 158, 117)

// ── Arc ────────────────────────────────────────────────
#define ARC_R      195
#define ARC_W       18
#define ARC_START  135.0f
#define ARC_END    405.0f
#define BOOST_MIN  -0.5f
#define BOOST_MAX   2.0f

// ── OBD UUIDs — KW901 BLE ──────────────────────────────
static NimBLEUUID SVC_UUID("FFF0");
static NimBLEUUID CHR_UUID("FFF1");

// ── OBD Commands (ASCII + CR, from car-hud) ────────────
const uint8_t CMD_ATZ[]   = {0x41,0x54,0x5A,0x0D};                         // ATZ
const uint8_t CMD_ATE0[]  = {0x41,0x54,0x45,0x30,0x0D};                    // ATE0
const uint8_t CMD_ATSP6[] = {0x41,0x54,0x53,0x50,0x36,0x0D};               // ATSP6
const uint8_t CMD_ATH1[]  = {0x41,0x54,0x48,0x31,0x0D};                    // ATH1 headers on
// BMW N55 boost: AT SH 6F1 sets header, then 2C 10 01 F4
const uint8_t CMD_ATSH[]  = {0x41,0x54,0x53,0x48,0x36,0x46,0x31,0x0D};    // AT SH 6F1
const uint8_t CMD_BOOST[] = {0x32,0x43,0x31,0x30,0x30,0x31,0x46,0x34,0x0D}; // 2C1001F4
// Standard coolant temp
const uint8_t CMD_TEMP[]  = {0x30,0x31,0x30,0x35,0x0D};                    // 0105

// ── Live values ────────────────────────────────────────
volatile float g_boost    = 0.0f;
volatile int   g_temp     = 0;
volatile bool  g_connected = false;
volatile bool  g_obdReady  = false;

// ── BLE handles ────────────────────────────────────────
static const NimBLEAdvertisedDevice *obdDevice = nullptr;
static NimBLEClient                 *bleClient = nullptr;
static NimBLERemoteCharacteristic   *obdChar   = nullptr;
static NimBLEScan                   *bleScan   = nullptr;

uint32_t lastBoostMs = 0;
uint32_t lastTempMs  = 0;

// ── Brightness / touch ─────────────────────────────────
int  brightness        = 200;
int  lastTouchY        = -1;
unsigned long lastTapTime    = 0;
unsigned long touchStartTime = 0;
bool touching          = false;
bool longPressTriggered = false;

// ── Color helpers ──────────────────────────────────────
inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t gradientColor(float boost) {
    struct Stop { float v; uint8_t r, g, b; };
    static const Stop s[] = {
        {-0.5f,   0,  60, 255},
        { 0.0f,   0, 210, 180},
        { 0.5f,   0, 220,  50},
        { 1.0f, 240, 220,   0},
        { 1.5f, 255,  90,   0},
        { 2.0f, 255,   0,   0},
    };
    for (int i = 1; i < 6; i++) {
        if (boost <= s[i].v) {
            float t = (boost - s[i-1].v) / (s[i].v - s[i-1].v);
            return rgb565(
                s[i-1].r + t*(s[i].r - s[i-1].r),
                s[i-1].g + t*(s[i].g - s[i-1].g),
                s[i-1].b + t*(s[i].b - s[i-1].b));
        }
    }
    return rgb565(255,0,0);
}

uint16_t dimColor(uint16_t c) {
    uint8_t r = ((c>>11)&0x1F)/5;
    uint8_t g = ((c>> 5)&0x3F)/5;
    uint8_t b = ( c     &0x1F)/5;
    return (r<<11)|(g<<5)|b;
}

// ── HEX utils ──────────────────────────────────────────
static uint8_t h2b(uint8_t hi, uint8_t lo) {
    hi = (hi<='9') ? hi-'0' : (hi|0x20)-'a'+10;
    lo = (lo<='9') ? lo-'0' : (lo|0x20)-'a'+10;
    return (hi<<4)|lo;
}

// ── Parse OBD notify ───────────────────────────────────
// car-hud style: responses come as ASCII hex strings
void parseObd(const uint8_t *d, size_t len) {
    if (len < 4) return;

    // Print raw for debugging
    char buf[128]={};
    memcpy(buf, d, min(len,(size_t)127));
    USBSerial.printf("OBD< %s\n", buf);

    // Standard response: "41 05 XX" for coolant
    // Find "41 05" in response
    for (size_t i = 0; i+6 < len; i++) {
        if (d[i]=='4' && d[i+1]=='1' && d[i+2]==' ' &&
            d[i+3]=='0' && d[i+4]=='5' && d[i+5]==' ') {
            uint8_t A = h2b(d[i+6], d[i+7]);
            g_temp = (int)A - 40;
            USBSerial.printf("Temp: %d C\n", g_temp);
        }
    }

    // BMW N55 boost response to "2C 10 01 F4":
    // Positive response PID = 2C+0x40 = 6C, service=50, PID=01F4
    // Response looks like: "6C 50 01 F4 XX YY ..."
    for (size_t i = 0; i+10 < len; i++) {
        if (d[i]=='6' && d[i+1]=='C' && d[i+2]==' ' &&
            d[i+3]=='5' && d[i+4]=='0') {
            // Skip to data bytes after "6C 50 01 F4 "
            // "6C 50 01 F4 " = 12 chars, so data at i+12
            size_t vi = i + 12;
            if (vi + 4 < len) {
                uint8_t hi_b = h2b(d[vi],   d[vi+1]);
                uint8_t lo_b = h2b(d[vi+3], d[vi+4]);
                uint16_t raw = ((uint16_t)hi_b << 8) | lo_b;
                float hPa = raw * 91.554f;
                float bar  = (hPa - 1013.25f) / 100.0f;
                g_boost = constrain(bar, -1.0f, 3.0f);
                USBSerial.printf("Boost: %.2f bar\n", g_boost);
            }
        }
    }
}

// ── BLE notify callback ────────────────────────────────
void notifyCB(NimBLERemoteCharacteristic*, uint8_t *data, size_t len, bool isNotify) {
    if (isNotify) parseObd(data, len);
}

// ── BLE scan callback ──────────────────────────────────
class ScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        USBSerial.printf("Found: %s\n", dev->getName().c_str());
        if (dev->haveServiceUUID() && dev->isAdvertisingService(SVC_UUID)) {
            USBSerial.println(">>> KW901 found! Connecting...");
            obdDevice = dev;
            NimBLEDevice::getScan()->stop();
        }
    }
};

// ── BLE client callbacks ───────────────────────────────
class ClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient*) override {
        g_connected = true;
        USBSerial.println("BLE connected");
    }
    void onDisconnect(NimBLEClient*, int reason) override {
        g_connected = false;
        g_obdReady  = false;
        obdChar     = nullptr;
        obdDevice   = nullptr;
        if (bleClient) { NimBLEDevice::deleteClient(bleClient); bleClient = nullptr; }
        USBSerial.printf("BLE disconnected (%d) — restarting scan\n", reason);
        if (bleScan) { bleScan->clearResults(); bleScan->start(0, false); }
    }
} clientCB;

// ── OBD write ──────────────────────────────────────────
void obdWrite(const uint8_t *cmd, size_t len) {
    if (obdChar && obdChar->canWrite())
        obdChar->writeValue(cmd, len, false);
}

// ── Connect to KW901 ───────────────────────────────────
bool connectObd() {
    bleClient = NimBLEDevice::createClient();
    bleClient->setClientCallbacks(&clientCB, false);
    if (!bleClient->connect(obdDevice)) {
        USBSerial.println("Connect failed");
        NimBLEDevice::deleteClient(bleClient);
        bleClient = nullptr;
        return false;
    }
    auto svc = bleClient->getService(SVC_UUID);
    if (!svc) { USBSerial.println("Service FFF0 not found"); return false; }
    obdChar = svc->getCharacteristic(CHR_UUID);
    if (!obdChar) { USBSerial.println("Char FFF1 not found"); return false; }
    if (obdChar->canNotify()) obdChar->subscribe(true, notifyCB);
    return true;
}

// ── Arc drawing ────────────────────────────────────────
void drawArc(float boost) {
    float activeAngle = ARC_START +
        (constrain(boost, BOOST_MIN, BOOST_MAX) - BOOST_MIN)
        / (BOOST_MAX - BOOST_MIN) * (ARC_END - ARC_START);

    for (float a = ARC_START; a <= ARC_END; a += 0.2f) {
        float rad = a * DEG_TO_RAD;
        int x = CX + ARC_R * cosf(rad);
        int y = CY + ARC_R * sinf(rad);
        float val = BOOST_MIN + (a-ARC_START)/(ARC_END-ARC_START)*(BOOST_MAX-BOOST_MIN);
        uint16_t col = gradientColor(val);
        if (a > activeAngle) col = dimColor(col);
        tft.gfx->fillCircle(x, y, ARC_W/2, col);
    }
}

// ── Centered text ──────────────────────────────────────
void drawCentered(const char* txt, int y, uint16_t color, int size) {
    int w = strlen(txt) * 6 * size;
    int x = CX - w/2;
    tft.gfx->setTextColor(0x0000);
    tft.gfx->setCursor(x+1, y+1);
    tft.gfx->setTextSize(size);
    tft.gfx->print(txt);
    tft.gfx->setTextColor(color);
    tft.gfx->setCursor(x, y);
    tft.gfx->setTextSize(size);
    tft.gfx->print(txt);
}

// ── Boost display ──────────────────────────────────────
void drawBoost(float boost) {
    drawCentered("BOOST", CY-76, C_BMW_DIM, 2);
    char buf[10];
    if (boost >= 0) snprintf(buf, sizeof(buf), "+%.2f", boost);
    else            snprintf(buf, sizeof(buf), "%.2f",  boost);
    drawCentered(buf, CY-36, C_BMW, 8);
    drawCentered("bar", CY+42, C_BMW_DIM, 2);
}

// ── Water temp display ─────────────────────────────────
void drawTemp(int temp) {
    tft.gfx->fillRect(CX-120, CY+130, 240, 90, C_BLACK);
    drawCentered("WATER TEMP", CY+135, C_BMW_DIM, 2);

    // Color by temp range
    uint16_t tc;
    if      (temp < 60)  tc = C_BLUE;
    else if (temp < 80)  tc = C_GREEN;
    else if (temp < 100) tc = C_BMW;
    else                 tc = C_RED;

    char buf[10];
    snprintf(buf, sizeof(buf), "%d C", temp);
    drawCentered(buf, CY+162, tc, 6);
}

// ── Status indicator ───────────────────────────────────
void drawStatus() {
    // Small dot top-right of center to show BT state
    uint16_t col = g_connected ?
        (g_obdReady ? rgb565(0,220,50) : rgb565(255,160,0)) :
        rgb565(80,80,80);
    tft.gfx->fillCircle(CX+85, CY-95, 6, col);
}

// ── Full gauge ─────────────────────────────────────────
void drawGauge(float boost, int temp) {
    tft.gfx->fillScreen(C_BLACK);
    drawArc(boost);
    drawBoost(boost);
    drawTemp(temp);
    drawStatus();
}

// ── Touch / brightness ────────────────────────────────
void handleBrightnessGesture() {
    uint16_t x, y;
    bool isTouching = tft.getTouch(&x, &y);

    if (isTouching && !touching) {
        touching = true;
        touchStartTime = millis();
        longPressTriggered = false;
        if (millis() - lastTapTime < 300) { brightness = 255; tft.setBrightness(brightness); }
        lastTapTime = millis();
        lastTouchY = y;
    }
    if (isTouching && !longPressTriggered) {
        int dy = lastTouchY - y;
        brightness = constrain(brightness + (int)(dy * 0.12f), 0, 255);
        tft.setBrightness(brightness);
        lastTouchY = y;
    }
    if (isTouching && !longPressTriggered && millis()-touchStartTime > 600)
        longPressTriggered = true;
    if (isTouching && longPressTriggered && brightness > 0) {
        brightness = max(0, brightness - 2);
        tft.setBrightness(brightness);
        delay(20);
    }
    if (!isTouching) { touching = false; lastTouchY = -1; }
}

// ── Setup ──────────────────────────────────────────────
void setup() {
    USB.begin();
    USBSerial.begin(115200);
    delay(2000);

    tft.init();
    tft.setBrightness(brightness);

    // Show startup screen
    tft.gfx->fillScreen(C_BLACK);
    drawCentered("BMW F32 435i", CY-20, C_BMW, 2);
    drawCentered("Searching BLE...", CY+10, C_BMW_DIM, 1);

    USBSerial.println("BMW Boost Gauge — starting BLE scan");

    // Init BLE
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    bleScan = NimBLEDevice::getScan();
    bleScan->setScanCallbacks(new ScanCB());
    bleScan->setActiveScan(true);
    bleScan->start(0);  // scan indefinitely
}

// ── Loop ───────────────────────────────────────────────
void loop() {
    handleBrightnessGesture();

    // Connect once KW901 is found
    if (obdDevice && !bleClient) {
        if (connectObd()) {
            delay(1000);
            // ELM327 init sequence — same as car-hud
            obdWrite(CMD_ATZ,   sizeof(CMD_ATZ));   delay(500);
            obdWrite(CMD_ATE0,  sizeof(CMD_ATE0));  delay(200);
            obdWrite(CMD_ATSP6, sizeof(CMD_ATSP6)); delay(200);
            obdWrite(CMD_ATH1,  sizeof(CMD_ATH1));  delay(200);
            obdWrite(CMD_ATSH,  sizeof(CMD_ATSH));  delay(200); // set BMW header
            g_obdReady = true;
            USBSerial.println("OBD init done — polling started");
        } else {
            obdDevice = nullptr;
            if (bleScan) { bleScan->clearResults(); bleScan->start(0, false); }
        }
    }

    // Poll OBD when ready
    if (g_obdReady) {
        uint32_t now = millis();

        // Boost every 200ms
        if (now - lastBoostMs >= 200) {
            lastBoostMs = now;
            obdWrite(CMD_BOOST, sizeof(CMD_BOOST));
        }

        // Coolant every 5s
        if (now - lastTempMs >= 5000) {
            lastTempMs = now;
            obdWrite(CMD_TEMP, sizeof(CMD_TEMP));
        }
    }

    // Redraw gauge
    drawGauge(g_boost, g_temp);

    delay(50);
}
