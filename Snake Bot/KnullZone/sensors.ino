/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║               K N U L L   Z O N E                       ║
 * ║     ESP32-C3 Environmental Monitoring Dashboard          ║
 * ╠══════════════════════════════════════════════════════════╣
 * ║  Sensors:                                                ║
 * ║    MQ-4   (Methane)        → GPIO 2  (ADC)              ║
 * ║    MQ-7   (Carbon Monoxide)→ GPIO 3  (ADC)              ║
 * ║    MQ-135 (Air Quality)    → GPIO 4  (ADC)              ║
 * ║    DHT22  (Temp & Humidity)→ GPIO 5  (Digital)           ║
 * ║                                                          ║
 * ║  Display:                                                ║
 * ║    1.3" OLED SH1106 I2C    → SDA=GPIO8, SCL=GPIO9      ║
 * ║                                                          ║
 * ║  Alerts:                                                 ║
 * ║    Buzzer + LED (shared)   → GPIO 6  (Digital OUT)      ║
 * ║                                                          ║
 * ║  WiFi AP: KNULL_ZONE  |  Password: knull1234            ║
 * ║  Dashboard: http://192.168.4.1                           ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 *  Libraries Required:
 *    - DHT sensor library by Adafruit
 *    - Adafruit Unified Sensor
 *    - U8g2 by olikraus
 *
 *  Board: ESP32-C3 Dev Module
 */
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <Wire.h>
#include <U8g2lib.h>
// ─── Pin Definitions ───────────────────────────────────────
#define MQ4_PIN    2    // MQ-4   Methane sensor (Analog)
#define MQ7_PIN    3    // MQ-7   CO sensor (Analog)
#define MQ135_PIN  4    // MQ-135 Air Quality sensor (Analog)
#define DHT_PIN    5    // DHT22  Temp & Humidity (Digital)
#define ALERT_PIN  6    // Buzzer + LED alert output (Digital)
#define DHTTYPE    DHT22
// ─── OLED I2C Pins (ESP32-C3 defaults) ─────────────────────
#define OLED_SDA   8    // I2C SDA → GPIO 8
#define OLED_SCL   9    // I2C SCL → GPIO 9
// ─── Access Point Credentials ──────────────────────────────
const char* ap_ssid = "KNULL_ZONE";
const char* ap_pass = "knull1234";
// ─── Objects ───────────────────────────────────────────────
DHT dht(DHT_PIN, DHTTYPE);
WebServer server(80);
// SH1106 1.3" OLED – Full buffer, Hardware I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// ─── Sensor Values ─────────────────────────────────────────
float mq4Val   = 0;
float mq7Val   = 0;
unsigned long alertStartTime  = 0;
bool alertActive = false;
// ─── OLED Eye Animation State ──────────────────────────────
unsigned long lastExprChange = 0;
const unsigned long EXPR_INTERVAL = 3000; // Change expression every 3s
int currentExpr = 0;
const int NUM_EXPRESSIONS = 8;
// Blink state
bool isBlinking = false;
unsigned long blinkStart = 0;
const unsigned long BLINK_DURATION = 150; // ms for a blink
unsigned long lastBlinkTime = 0;
unsigned long nextBlinkDelay = 4000; // random interval between blinks
// ─── Forward Declarations ──────────────────────────────────
void readSensors();
void handleRoot();
void handleData();
void triggerAlert();
void updateAlert();
void updateOLED();
void drawMochiEyes(int expression, float blinkFactor);
// ═══════════════════════════════════════════════════════════
//  HTML Page (stored in flash via PROGMEM)
// ═══════════════════════════════════════════════════════════
//  OLED MOCHI EYE ANIMATIONS
// ═══════════════════════════════════════════════════════════
/*
 * Expression IDs:
 *   0 = Normal       (round cute eyes)
 *   1 = Happy        (^_^ squint-smile)
 *   2 = Blink        (thin line — handled by blinkFactor)
 *   3 = Surprised    (wide O_O big pupils)
 *   4 = Sleepy       (droopy half-closed)
 *   5 = Love         (heart-shaped eyes)
 *   6 = Wink         (left open, right closed ^)
 *   7 = Angry        (angled brows, sharp eyes)
 */
// Helper: draw a filled rounded rectangle (pill shape)
void fillRoundRect(int x, int y, int w, int h, int r) {
  oled.drawRBox(x, y, w, h, r);
}
// Helper: draw a heart shape at (cx, cy) with given size
void drawHeart(int cx, int cy, int size) {
  // Two circles for top bumps + triangle for bottom
  int r = size / 2;
  oled.drawDisc(cx - r + 1, cy - r / 2, r);
  oled.drawDisc(cx + r - 1, cy - r / 2, r);
  oled.drawTriangle(cx - size, cy, cx + size, cy, cx, cy + size + r / 2);
}
void drawMochiEyes(int expression, float blinkFactor) {
  // blinkFactor: 1.0 = fully open, 0.0 = fully closed
  // Eye centers
  int lx = 32;  // Left eye center X
  int rx = 96;  // Right eye center X
  int ey = 28;  // Eye center Y
  // Base eye size
  int ew = 24;  // Eye width
  int eh = 28;  // Eye height (fully open)
  // Apply blink squeeze
  int actualH = (int)(eh * blinkFactor);
  if (actualH < 2) actualH = 2;
  int yOffset = (eh - actualH) / 2;
  switch (expression) {
    case 0: // ── Normal: round cute eyes with pupils ──
    {
      // White of eyes
      oled.drawRBox(lx - ew/2, ey - actualH/2 + yOffset, ew, actualH, actualH/3);
      oled.drawRBox(rx - ew/2, ey - actualH/2 + yOffset, ew, actualH, actualH/3);
      // Clear inside for "white" effect, then draw pupils
      if (blinkFactor > 0.3) {
        int pr = 5; // pupil radius
        // Pupils (filled circles)
        oled.drawDisc(lx, ey + 1, pr);
        oled.drawDisc(rx, ey + 1, pr);
        // Tiny highlight dot
        oled.setDrawColor(0);
        oled.drawDisc(lx - 2, ey - 2, 2);
        oled.drawDisc(rx - 2, ey - 2, 2);
        oled.setDrawColor(1);
      }
      // Cute little mouth
      oled.drawPixel(62, 48);
      oled.drawPixel(63, 49);
      oled.drawPixel(64, 49);
      oled.drawPixel(65, 48);
      break;
    }
    case 1: // ── Happy: ^_^ squinted smile ──
    {
      int smileH = max(4, (int)(8 * blinkFactor));
      // Left eye: upward arc (^)
      for (int i = -ew/2; i <= ew/2; i++) {
        int yy = ey - (smileH - abs(i) * smileH / (ew/2));
        oled.drawPixel(lx + i, yy);
        oled.drawPixel(lx + i, yy + 1);
      }
      // Right eye: upward arc (^)
      for (int i = -ew/2; i <= ew/2; i++) {
        int yy = ey - (smileH - abs(i) * smileH / (ew/2));
        oled.drawPixel(rx + i, yy);
        oled.drawPixel(rx + i, yy + 1);
      }
      // Big smile
      for (int i = -10; i <= 10; i++) {
        int yy = 48 + (i * i) / 15;
        oled.drawPixel(64 + i, yy);
        oled.drawPixel(64 + i, yy + 1);
      }
      // Blush circles
      oled.drawCircle(20, 42, 4);
      oled.drawCircle(108, 42, 4);
      break;
    }
    case 2: // ── Blink: thin horizontal lines ──
    {
      oled.drawHLine(lx - ew/2, ey, ew);
      oled.drawHLine(lx - ew/2, ey + 1, ew);
      oled.drawHLine(rx - ew/2, ey, ew);
      oled.drawHLine(rx - ew/2, ey + 1, ew);
      break;
    }
    case 3: // ── Surprised: big O_O eyes ──
    {
      int bigH = (int)(34 * blinkFactor);
      if (bigH < 2) bigH = 2;
      int bigW = 28;
      int bigR = bigH / 2;
      if (bigR < 1) bigR = 1;
      // Big round eyes
      oled.drawRBox(lx - bigW/2, ey - bigH/2, bigW, bigH, bigR);
      oled.drawRBox(rx - bigW/2, ey - bigH/2, bigW, bigH, bigR);
      if (blinkFactor > 0.3) {
        // Tiny pupils (surprised = small pupils)
        oled.setDrawColor(0);
        oled.drawDisc(lx, ey, bigH/2 - 3);
        oled.drawDisc(rx, ey, bigH/2 - 3);
        oled.setDrawColor(1);
        oled.drawDisc(lx, ey, 3);
        oled.drawDisc(rx, ey, 3);
        oled.setDrawColor(0);
        oled.drawDisc(lx - 1, ey - 2, 1);
        oled.drawDisc(rx - 1, ey - 2, 1);
        oled.setDrawColor(1);
      }
      // Small "o" mouth
      oled.drawCircle(64, 50, 4);
      break;
    }
    case 4: // ── Sleepy: half-closed droopy eyes ──
    {
      int sleepH = max(2, (int)(12 * blinkFactor));
      // Half-closed eyes (lower half only)
      oled.drawRBox(lx - ew/2, ey, ew, sleepH, sleepH/3);
      oled.drawRBox(rx - ew/2, ey, ew, sleepH, sleepH/3);
      // Eyelid line on top
      oled.drawHLine(lx - ew/2 - 2, ey, ew + 4);
      oled.drawHLine(rx - ew/2 - 2, ey, ew + 4);
      if (blinkFactor > 0.3 && sleepH > 4) {
        oled.drawDisc(lx, ey + sleepH/2, 2);
        oled.drawDisc(rx, ey + sleepH/2, 2);
      }
      // Zzz
      oled.setFont(u8g2_font_5x7_tr);
      oled.drawStr(105, 14, "Z");
      oled.drawStr(112, 8, "z");
      oled.drawStr(118, 4, "z");
      break;
    }
    case 5: // ── Love: heart-shaped eyes ──
    {
      if (blinkFactor > 0.2) {
        drawHeart(lx, ey, (int)(8 * blinkFactor));
        drawHeart(rx, ey, (int)(8 * blinkFactor));
      } else {
        oled.drawHLine(lx - ew/2, ey, ew);
        oled.drawHLine(rx - ew/2, ey, ew);
      }
      // Happy blush marks
      oled.drawCircle(18, 44, 3);
      oled.drawCircle(110, 44, 3);
      // Smile
      for (int i = -8; i <= 8; i++) {
        int yy = 50 + (i * i) / 12;
        oled.drawPixel(64 + i, yy);
      }
      break;
    }
    case 6: // ── Wink: left eye open, right eye ^_~ ──
    {
      // Left eye: normal open
      oled.drawRBox(lx - ew/2, ey - actualH/2 + yOffset, ew, actualH, actualH/3);
      if (blinkFactor > 0.3) {
        oled.drawDisc(lx, ey + 1, 5);
        oled.setDrawColor(0);
        oled.drawDisc(lx - 2, ey - 2, 2);
        oled.setDrawColor(1);
      }
      // Right eye: wink arc
      for (int i = -ew/2; i <= ew/2; i++) {
        int winkH = 6;
        int yy = ey + (winkH - abs(i) * winkH / (ew/2));
        oled.drawPixel(rx + i, yy);
        oled.drawPixel(rx + i, yy - 1);
      }
      // Cheeky smile (offset)
      for (int i = -10; i <= 6; i++) {
        int yy = 48 + (i * i) / 18;
        oled.drawPixel(60 + i, yy);
      }
      // Blush on wink side
      oled.drawCircle(108, 42, 4);
      break;
    }
    case 7: // ── Angry: angled brows, sharp eyes ──
    {
      int angryH = max(2, (int)(20 * blinkFactor));
      // Narrowed eyes
      oled.drawRBox(lx - ew/2, ey - angryH/4, ew, angryH/2, 2);
      oled.drawRBox(rx - ew/2, ey - angryH/4, ew, angryH/2, 2);
      if (blinkFactor > 0.3) {
        oled.drawDisc(lx, ey, 3);
        oled.drawDisc(rx, ey, 3);
      }
      // Angry eyebrows
      // Left brow: \  (angled down-right)
      oled.drawLine(lx - ew/2 - 2, ey - angryH/2 - 6, lx + ew/2 + 2, ey - angryH/2 - 2);
      oled.drawLine(lx - ew/2 - 2, ey - angryH/2 - 7, lx + ew/2 + 2, ey - angryH/2 - 3);
      // Right brow: /  (angled down-left)
      oled.drawLine(rx - ew/2 - 2, ey - angryH/2 - 2, rx + ew/2 + 2, ey - angryH/2 - 6);
      oled.drawLine(rx - ew/2 - 2, ey - angryH/2 - 3, rx + ew/2 + 2, ey - angryH/2 - 7);
      // Grumpy mouth (flat line with slight frown)
      oled.drawHLine(56, 50, 16);
      oled.drawPixel(55, 49);
      oled.drawPixel(72, 49);
      break;
    }
  }
  // Cute cheek dots (always visible for mochi look, except angry)
  if (expression != 7 && expression != 1 && expression != 5 && expression != 6) {
    oled.drawPixel(16, 40);
    oled.drawPixel(17, 41);
    oled.drawPixel(18, 40);
    oled.drawPixel(110, 40);
    oled.drawPixel(111, 41);
    oled.drawPixel(112, 40);
  }
}
void updateOLED() {
  unsigned long now = millis();
  // ─── Handle periodic expression change ───
  if (now - lastExprChange >= EXPR_INTERVAL) {
    lastExprChange = now;
    // Pick a new random expression (avoid repeating the same one)
    int newExpr;
    do {
      newExpr = random(NUM_EXPRESSIONS);
    } while (newExpr == currentExpr);
    currentExpr = newExpr;
  }
  // ─── Handle natural blink ───
  float blinkFactor = 1.0; // 1.0 = fully open
  if (!isBlinking) {
    // Check if it's time for a spontaneous blink
    if (now - lastBlinkTime >= nextBlinkDelay) {
      isBlinking = true;
      blinkStart = now;
    }
  }
  if (isBlinking) {
    unsigned long elapsed = now - blinkStart;
    if (elapsed < BLINK_DURATION / 2) {
      // Closing
      blinkFactor = 1.0 - (float)elapsed / (BLINK_DURATION / 2);
    } else if (elapsed < BLINK_DURATION) {
      // Opening
      blinkFactor = (float)(elapsed - BLINK_DURATION / 2) / (BLINK_DURATION / 2);
    } else {
      // Blink finished
      isBlinking = false;
      blinkFactor = 1.0;
      lastBlinkTime = now;
      nextBlinkDelay = 3000 + random(4000); // 3-7s between blinks
    }
  }
  // ─── Draw ───
  oled.clearBuffer();
  drawMochiEyes(currentExpr, blinkFactor);
  oled.sendBuffer();
}
// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  dht.begin();
  Serial.println("[INIT] DHT22 sensor initialized on GPIO 5");
  // ─── Initialize OLED Display ───────────────────────────
  Wire.setPins(OLED_SDA, OLED_SCL);
  oled.begin();
  oled.setContrast(200);
  oled.setFont(u8g2_font_5x7_tr); // Small font for any text
  oled.clearBuffer();
  // Boot splash
  oled.setFont(u8g2_font_helvB10_tr);
  oled.drawStr(14, 28, "KNULL ZONE");
  oled.setFont(u8g2_font_5x7_tr);
  oled.drawStr(28, 46, "Booting up...");
  oled.sendBuffer();
  Serial.println("[INIT] OLED SH1106 initialized (SDA=GPIO8, SCL=GPIO9)");
  delay(1500);
  // Seed random for expression variety
  randomSeed(analogRead(0));
  // Configure WiFi as Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  server.handleClient();
  readSensors();
  updateAlert();   // Turn off buzzer+LED after pulse duration
  updateOLED();    // Update animated mochi eyes on OLED
}
