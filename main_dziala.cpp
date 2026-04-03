#include <Arduino.h>
#include <USB.h>
#include <USBCDC.h>
#include <math.h>

#include "viewe.hpp"

USBCDC USBSerial;

// ── Screen ─────────────────────────────────────────────
#define SCREEN_W  466
#define SCREEN_H  466
#define CX        233
#define CY        233

// ── Colors ─────────────────────────────────────────────
#define C_BLACK   0x0000
#define C_BMW     0xFAA0
#define C_BMW_DIM 0x50E0

// ── Gauge arc ──────────────────────────────────────────
#define ARC_R      195
#define ARC_W       18
#define ARC_START  135.0f
#define ARC_END    405.0f
#define BOOST_MIN  -0.5f
#define BOOST_MAX   2.0f

// ── Brightness ─────────────────────────────────────────
int brightness = 200;
int lastTouchY = -1;

unsigned long lastTapTime = 0;
unsigned long touchStartTime = 0;

bool touching = false;
bool longPressTriggered = false;

// ── Helpers ────────────────────────────────────────────
inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t gradientColor(float boost) {
    struct Stop { float v; uint8_t r, g, b; };
    static const Stop stops[] = {
        {-0.5f,   0,  60, 255},
        { 0.0f,   0, 210, 180},
        { 0.5f,   0, 220,  50},
        { 1.0f, 240, 220,   0},
        { 1.5f, 255,  90,   0},
        { 2.0f, 255,   0,   0},
    };

    for (int i = 1; i < 6; i++) {
        if (boost <= stops[i].v) {
            float t = (boost - stops[i-1].v) / (stops[i].v - stops[i-1].v);
            return rgb565(
                stops[i-1].r + t * (stops[i].r - stops[i-1].r),
                stops[i-1].g + t * (stops[i].g - stops[i-1].g),
                stops[i-1].b + t * (stops[i].b - stops[i-1].b)
            );
        }
    }
    return rgb565(255, 0, 0);
}

uint16_t dimColor(uint16_t c) {
    uint8_t r = ((c >> 11) & 0x1F) / 5;
    uint8_t g = ((c >>  5) & 0x3F) / 5;
    uint8_t b = ( c        & 0x1F) / 5;
    return (r << 11) | (g << 5) | b;
}

// ── Arc ────────────────────────────────────────────────
void drawArc(float boost) {
    float activeAngle = ARC_START +
        (constrain(boost, BOOST_MIN, BOOST_MAX) - BOOST_MIN)
        / (BOOST_MAX - BOOST_MIN) * (ARC_END - ARC_START);

    for (float a = ARC_START; a <= ARC_END; a += 0.2f) {
        float rad = a * DEG_TO_RAD;

        int x = CX + ARC_R * cosf(rad);
        int y = CY + ARC_R * sinf(rad);

        float val = BOOST_MIN +
            (a - ARC_START) / (ARC_END - ARC_START) * (BOOST_MAX - BOOST_MIN);

        uint16_t col = gradientColor(val);
        if (a > activeAngle) col = dimColor(col);

        tft.gfx->fillCircle(x, y, ARC_W / 2, col);
    }
}

// ── Text ───────────────────────────────────────────────
void drawCentered(const char* txt, int y, uint16_t color, int size) {
    int w = strlen(txt) * 6 * size;
    int x = CX - w / 2;

    tft.gfx->setTextColor(0x0000);
    tft.gfx->setCursor(x + 1, y + 1);
    tft.gfx->setTextSize(size);
    tft.gfx->print(txt);

    tft.gfx->setTextColor(color);
    tft.gfx->setCursor(x, y);
    tft.gfx->setTextSize(size);
    tft.gfx->print(txt);
}

// ── Boost ──────────────────────────────────────────────
void drawBoost(float boost) {
    drawCentered("BOOST", CY - 76, C_BMW_DIM, 2);

    char buf[10];
    if (boost >= 0) snprintf(buf, sizeof(buf), "+%.2f", boost);
    else            snprintf(buf, sizeof(buf), "%.2f", boost);

    drawCentered(buf, CY - 36, C_BMW, 8);
    drawCentered("bar", CY + 42, C_BMW_DIM, 2);
}

// ── Temp ───────────────────────────────────────────────
void drawTemp(int temp) {
    tft.gfx->fillRect(CX - 120, CY + 130, 240, 90, C_BLACK);

    drawCentered("WATER TEMP", CY + 135, C_BMW_DIM, 2);

    char buf[10];
    snprintf(buf, sizeof(buf), "%d C", temp);
    drawCentered(buf, CY + 162, C_BMW, 6);
}

// ── Full gauge ─────────────────────────────────────────
void drawGauge(float boost, int temp) {
    tft.gfx->fillScreen(C_BLACK);
    drawArc(boost);
    drawBoost(boost);
    drawTemp(temp);
}

// ── Gestures ───────────────────────────────────────────
void handleBrightnessGesture() {
    uint16_t x, y;
    bool isTouching = tft.getTouch(&x, &y);

    // TOUCH START
    if (isTouching && !touching) {
        touching = true;
        touchStartTime = millis();
        longPressTriggered = false;

        // double tap → 100%
        if (millis() - lastTapTime < 300) {
            brightness = 255;
            tft.setBrightness(brightness);
        }

        lastTapTime = millis();
        lastTouchY = y;
    }

    // SWIPE
    if (isTouching && !longPressTriggered) {
        int dy = lastTouchY - y;

        float sensitivity = 0.12f;
        brightness += dy * sensitivity;

        if (brightness < 0) brightness = 0;
        if (brightness > 255) brightness = 255;

        tft.setBrightness(brightness);
        lastTouchY = y;
    }

    // LONG PRESS trigger
    if (isTouching && !longPressTriggered) {
        if (millis() - touchStartTime > 600) {
            longPressTriggered = true;
        }
    }

    // HOLD → dim to 0%
    if (isTouching && longPressTriggered) {
        if (brightness > 0) {
            brightness -= 2;
            if (brightness < 0) brightness = 0;
            tft.setBrightness(brightness);
            delay(20);
        }
    }

    // TOUCH END
    if (!isTouching) {
        touching = false;
        lastTouchY = -1;
    }
}

// ── Setup ──────────────────────────────────────────────
void setup() {
    USB.begin();
    USBSerial.begin(115200);
    delay(2000);

    tft.init();
    tft.setBrightness(brightness);

    drawGauge(1.2f, 67);
}

// ── Loop ───────────────────────────────────────────────
void loop() {
    handleBrightnessGesture();
}