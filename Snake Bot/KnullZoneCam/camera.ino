/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║                 K N U L L   Z O N E   C A M                 ║
 * ║       ESP32-CAM Surveillance + Animated OLED Face           ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Hardware:                                                   ║
 * ║    ESP32-CAM (AI-Thinker) with OV2640 camera                ║
 * ║    1.3" OLED Display (SH1106 / SSD1306) 128x64 I2C         ║
 * ║                                                              ║
 * ║  OLED Wiring:                                                ║
 * ║    SDA → GPIO 14    SCL → GPIO 15                           ║
 * ║    VCC → 3.3V       GND → GND                              ║
 * ║                                                              ║
 * ║  Flash LED: GPIO 4 (built-in)                               ║
 * ║                                                              ║
 * ║  WiFi AP : KNULL_ZONE_CAM  |  Password: knull1234          ║
 * ║  Page    : http://192.168.4.1                               ║
 * ║  Stream  : http://192.168.4.1:81/stream                     ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Libraries Required:                                         ║
 * ║    - U8g2  (by oliver)  — for OLED display                  ║
 * ║    - esp32 board package (by Espressif)                      ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════
//  CAMERA PIN DEFINITIONS  (AI-Thinker ESP32-CAM)
// ═══════════════════════════════════════════════════════════════
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ─── Flash LED & OLED Pins ────────────────────────────────────
#define FLASH_PIN    4     // Built-in flash LED
#define OLED_SDA    14     // OLED I2C Data
#define OLED_SCL    15     // OLED I2C Clock

// ═══════════════════════════════════════════════════════════════
//  CONFIGURATION
// ═══════════════════════════════════════════════════════════════
const char* ap_ssid = "KNULL_ZONE_CAM";
const char* ap_pass = "knull1234";

// Motion detection threshold (fraction of JPEG size change)
#define MOTION_THRESHOLD  0.12f
#define MOTION_COOLDOWN   5000   // ms before motion flag clears

// OLED face geometry (128x64 display)
#define SCREEN_W      128
#define SCREEN_H       64
#define EYE_L_CX       42    // Left eye center X
#define EYE_R_CX       86    // Right eye center X
#define EYE_CY         24    // Eye center Y
#define EYE_RADIUS      9    // Normal eye radius
#define PUPIL_RADIUS    4    // Pupil radius
#define MOUTH_CX       64    // Mouth center X
#define MOUTH_CY       48    // Mouth center Y

// ═══════════════════════════════════════════════════════════════
//  GLOBAL OBJECTS & STATE
// ═══════════════════════════════════════════════════════════════

// OLED — 1.3" SH1106 128x64 via Software I2C
// ► If your display uses SSD1306, comment the SH1106 line and
//   uncomment the SSD1306 line below:
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

// HTTP server handles
httpd_handle_t camera_httpd = NULL;   // Port 80 — page, API
httpd_handle_t stream_httpd = NULL;   // Port 81 — MJPEG stream

// Flash state
volatile bool flashOn = false;

// Motion detection
volatile bool  motionDetected  = false;
volatile bool  motionTriggered = false;   // One-shot flag for OLED
unsigned long  motionTimestamp  = 0;
float          prevFrameSize   = 0;

// FPS tracking
volatile uint32_t frameCount    = 0;
volatile float    currentFps    = 0;
uint32_t          lastFrameCount = 0;
unsigned long     lastFpsCalc    = 0;

// Face animation state
enum FaceState {
  FACE_IDLE,
  FACE_BLINK,
  FACE_SURPRISED,
  FACE_HAPPY
};
volatile FaceState currentFaceState = FACE_IDLE;


// ═══════════════════════════════════════════════════════════════
//  OLED FACE DRAWING FUNCTIONS
// ═══════════════════════════════════════════════════════════════

// ─── Draw a smile arc (bottom semicircle) ─────────────────────
void drawSmile(int cx, int cy, int radius, int thickness) {
  for (int t = 0; t < thickness; t++) {
    int r = radius + t;
    for (int angle = 20; angle <= 160; angle += 2) {
      float rad = angle * 3.14159f / 180.0f;
      int x = cx + (int)(r * cos(rad));
      int y = cy + (int)(r * sin(rad));
      if (x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H) {
        u8g2.drawPixel(x, y);
      }
    }
  }
}

// ─── Draw open-mouth circle (surprise) ───────────────────────
void drawOpenMouth(int cx, int cy, int radius) {
  u8g2.drawCircle(cx, cy, radius);
  u8g2.drawCircle(cx, cy, radius - 1);
}

// ─── Draw happy eyes (^_^  arc eyes) ─────────────────────────
void drawHappyEye(int cx, int cy, int radius) {
  // Draw top arc of circle, then mask bottom half
  u8g2.drawCircle(cx, cy + 2, radius);
  u8g2.drawCircle(cx, cy + 2, radius - 1);
  // Mask bottom half with black box
  u8g2.setDrawColor(0);
  u8g2.drawBox(cx - radius - 2, cy + 3, radius * 2 + 4, radius + 4);
  u8g2.setDrawColor(1);
}

// ─── Normal eye with pupil and shine ─────────────────────────
void drawNormalEye(int cx, int cy, int eyeR, int pupilOffX, int pupilOffY) {
  // White filled eye
  u8g2.drawDisc(cx, cy, eyeR);
  // Black pupil
  u8g2.setDrawColor(0);
  u8g2.drawDisc(cx + pupilOffX, cy + pupilOffY, PUPIL_RADIUS);
  u8g2.setDrawColor(1);
  // Shine highlight
  u8g2.drawDisc(cx + pupilOffX - 2, cy + pupilOffY - 2, 1);
}

// ─── Blink eye (thin horizontal line) ────────────────────────
void drawBlinkEye(int cx, int cy, int width) {
  u8g2.drawBox(cx - width / 2, cy - 1, width, 3);
}

// ─── Surprised eye (large circle with tiny pupil) ────────────
void drawSurprisedEye(int cx, int cy, int radius) {
  u8g2.drawCircle(cx, cy, radius);
  u8g2.drawCircle(cx, cy, radius - 1);
  // Tiny pupil
  u8g2.drawDisc(cx, cy, 3);
  // Large shine
  u8g2.setDrawColor(0);
  u8g2.drawDisc(cx - 3, cy - 3, 2);
  u8g2.setDrawColor(1);
  u8g2.drawDisc(cx - 3, cy - 3, 1);
}

// ─── Draw exclamation marks (for surprise) ───────────────────
void drawExclamation(int x, int y) {
  u8g2.drawBox(x, y, 2, 6);
  u8g2.drawBox(x, y + 8, 2, 2);
}

// ─── Draw blush marks (for happy) ────────────────────────────
void drawBlush(int cx, int cy) {
  // Small diagonal lines for blush effect
  u8g2.drawLine(cx - 3, cy, cx + 3, cy);
  u8g2.drawLine(cx - 2, cy + 1, cx + 2, cy + 1);
}

// ─── Draw status bar at bottom ───────────────────────────────
void drawStatusBar(const char* text) {
  u8g2.setFont(u8g2_font_5x7_tr);
  int textW = u8g2.getStrWidth(text);
  int x = (SCREEN_W - textW) / 2;
  // Divider line
  u8g2.drawLine(10, 55, 118, 55);
  u8g2.drawStr(x, 63, text);
}

// ─── Draw rounded frame border ───────────────────────────────
void drawFaceFrame() {
  u8g2.drawRFrame(0, 0, 128, 53, 8);
}


// ═══════════════════════════════════════════════════════════════
//  COMPLETE FACE RENDERERS
// ═══════════════════════════════════════════════════════════════

void renderIdleFace(int lookX, int lookY, float breathY) {
  int by = (int)breathY;
  drawFaceFrame();
  // Eyes
  drawNormalEye(EYE_L_CX, EYE_CY + by, EYE_RADIUS, lookX, lookY);
  drawNormalEye(EYE_R_CX, EYE_CY + by, EYE_RADIUS, lookX, lookY);
  // Smile
  drawSmile(MOUTH_CX, MOUTH_CY - 6 + by, 12, 2);
  drawStatusBar("KNULL ZONE");
}

void renderBlinkFace(float breathY) {
  int by = (int)breathY;
  drawFaceFrame();
  // Closed eyes
  drawBlinkEye(EYE_L_CX, EYE_CY + by, 16);
  drawBlinkEye(EYE_R_CX, EYE_CY + by, 16);
  // Smile
  drawSmile(MOUTH_CX, MOUTH_CY - 6 + by, 12, 2);
  drawStatusBar("KNULL ZONE");
}

void renderSurprisedFace(float breathY, unsigned long elapsed) {
  int by = (int)breathY;
  // Shake effect in first 500ms
  int shakeX = 0;
  if (elapsed < 500) {
    shakeX = (int)(sin(elapsed / 30.0f) * 2);
  }
  drawFaceFrame();
  // Big surprised eyes
  drawSurprisedEye(EYE_L_CX + shakeX, EYE_CY + by, 12);
  drawSurprisedEye(EYE_R_CX + shakeX, EYE_CY + by, 12);
  // Exclamation marks
  drawExclamation(EYE_L_CX - 18 + shakeX, EYE_CY - 12 + by);
  drawExclamation(EYE_R_CX + 14 + shakeX, EYE_CY - 12 + by);
  // Open mouth
  drawOpenMouth(MOUTH_CX + shakeX, MOUTH_CY - 2 + by, 6);
  drawStatusBar("! DETECTED !");
}

void renderHappyFace(float breathY) {
  int by = (int)breathY;
  drawFaceFrame();
  // Happy arc eyes ^_^
  drawHappyEye(EYE_L_CX, EYE_CY - 2 + by, EYE_RADIUS);
  drawHappyEye(EYE_R_CX, EYE_CY - 2 + by, EYE_RADIUS);
  // Blush marks
  drawBlush(EYE_L_CX - 6, EYE_CY + 8 + by);
  drawBlush(EYE_R_CX + 6, EYE_CY + 8 + by);
  // Wide smile
  drawSmile(MOUTH_CX, MOUTH_CY - 8 + by, 16, 2);
  drawStatusBar("~ HELLO ~");
}


// ═══════════════════════════════════════════════════════════════
//  OLED ANIMATION TASK  (runs on Core 0 via FreeRTOS)
// ═══════════════════════════════════════════════════════════════
void oledAnimationTask(void *pvParameters) {
  FaceState state        = FACE_IDLE;
  unsigned long stateStart   = millis();
  unsigned long lastBlink    = millis();
  unsigned long nextBlink    = 3500;
  unsigned long lastLookChange = millis();
  unsigned long nextLook     = 6000;
  int lookDirX = 0;
  int lookDirY = 0;

  // Boot splash
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawStr(10, 28, "KNULL ZONE");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(28, 44, "CAM  STARTING...");
  u8g2.sendBuffer();
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  while (true) {
    unsigned long now = millis();

    // ─── Check for motion trigger from main loop ───
    if (motionTriggered) {
      motionTriggered = false;
      state = FACE_SURPRISED;
      stateStart = now;
    }

    // ─── State machine transitions ───
    switch (state) {
      case FACE_IDLE:
        // Random blink
        if (now - lastBlink > nextBlink) {
          state = FACE_BLINK;
          stateStart = now;
          lastBlink = now;
          nextBlink = 2500 + esp_random() % 4000;  // 2.5–6.5s
        }
        // Random look direction
        if (now - lastLookChange > nextLook) {
          int r = esp_random() % 5;
          if (r == 0)      { lookDirX = -3; lookDirY = 0; }
          else if (r == 1) { lookDirX = 3;  lookDirY = 0; }
          else if (r == 2) { lookDirX = 0;  lookDirY = -2; }
          else             { lookDirX = 0;  lookDirY = 0; }
          lastLookChange = now;
          nextLook = 4000 + esp_random() % 6000;
        }
        break;

      case FACE_BLINK:
        if (now - stateStart > 180) {
          state = FACE_IDLE;
        }
        break;

      case FACE_SURPRISED:
        if (now - stateStart > 2200) {
          state = FACE_HAPPY;
          stateStart = now;
        }
        break;

      case FACE_HAPPY:
        if (now - stateStart > 1800) {
          state = FACE_IDLE;
          stateStart = now;
          lookDirX = 0;
          lookDirY = 0;
        }
        break;
    }

    // ─── Breathing animation (subtle Y oscillation) ───
    float breathY = sin(now / 800.0f) * 1.5f;

    // ─── Render current face ───
    u8g2.clearBuffer();

    switch (state) {
      case FACE_IDLE:
        renderIdleFace(lookDirX, lookDirY, breathY);
        break;
      case FACE_BLINK:
        renderBlinkFace(breathY);
        break;
      case FACE_SURPRISED:
        renderSurprisedFace(breathY, now - stateStart);
        break;
      case FACE_HAPPY:
        renderHappyFace(breathY);
        break;
    }

    // Update shared state for web API
    currentFaceState = state;

    u8g2.sendBuffer();

    vTaskDelay(50 / portTICK_PERIOD_MS);  // ~20 FPS
  }
}


// ═══════════════════════════════════════════════════════════════
//  MOTION DETECTION  (uses JPEG frame size heuristic)
// ═══════════════════════════════════════════════════════════════
void checkMotion() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  float currentSize = (float)fb->len;
  esp_camera_fb_return(fb);

  if (prevFrameSize > 0) {
    float diff = fabs(currentSize - prevFrameSize) / prevFrameSize;
    if (diff > MOTION_THRESHOLD) {
      motionDetected  = true;
      motionTriggered = true;   // Tell OLED task
      motionTimestamp  = millis();
      Serial.printf("[MOTION] Detected! delta=%.1f%%\n", diff * 100);
    }
  }
  prevFrameSize = currentSize;

  // Auto-clear motion flag after cooldown
  if (motionDetected && millis() - motionTimestamp > MOTION_COOLDOWN) {
    motionDetected = false;
  }
}


// ═══════════════════════════════════════════════════════════════
//  HTML DASHBOARD PAGE  (stored in flash via PROGMEM)
// ═══════════════════════════════════════════════════════════════
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<meta name="description" content="KNULL ZONE CAM - ESP32-CAM Surveillance Dashboard">
<title>KNULL ZONE CAM</title>
<style>
:root{
  --bg-primary:#0c0a1d;--bg-card:#1c1a3a;--bg-card-light:#252350;
  --text-primary:#e8e6ff;--text-secondary:#8b89b0;--text-muted:#5a587a;
  --accent-red:#ff6b6b;--accent-green:#51cf66;--accent-cyan:#22d3ee;
  --accent-amber:#ffc107;--accent-purple:#a78bfa;
  --shadow-dark:rgba(0,0,0,0.6);--shadow-light:rgba(50,48,100,0.12);
  --radius:28px;
}
*{margin:0;padding:0;box-sizing:border-box}
html{scroll-behavior:smooth}
body{
  font-family:'Segoe UI',system-ui,-apple-system,sans-serif;
  background:var(--bg-primary);
  background-image:
    radial-gradient(ellipse at 20% 50%,rgba(167,139,250,0.06)0%,transparent 50%),
    radial-gradient(ellipse at 80% 20%,rgba(34,211,238,0.05)0%,transparent 50%),
    radial-gradient(ellipse at 50% 100%,rgba(255,107,107,0.04)0%,transparent 40%);
  min-height:100vh;color:var(--text-primary);overflow-x:hidden;
}
body::before{
  content:'';position:fixed;inset:0;
  background-image:
    linear-gradient(rgba(255,255,255,0.015)1px,transparent 1px),
    linear-gradient(90deg,rgba(255,255,255,0.015)1px,transparent 1px);
  background-size:60px 60px;pointer-events:none;z-index:0;
}

/* ─── Header ─── */
.header{text-align:center;padding:28px 20px 8px;position:relative;z-index:1}
.logo-wrap{display:inline-block;position:relative}
.logo-wrap::before{
  content:'';position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
  width:300px;height:80px;
  background:radial-gradient(ellipse,rgba(167,139,250,0.15),transparent 70%);
  filter:blur(20px);pointer-events:none;
}
.header h1{
  font-size:2.6rem;font-weight:900;letter-spacing:10px;text-transform:uppercase;
  background:linear-gradient(135deg,#ff6b6b 0%,#ffa726 20%,#ffd93d 40%,#51cf66 60%,#22d3ee 80%,#a78bfa 100%);
  background-size:200% auto;
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
  animation:shimmer 4s linear infinite;
}
@keyframes shimmer{0%{background-position:0% center}100%{background-position:200% center}}
.subtitle{font-size:.7rem;color:var(--text-muted);letter-spacing:5px;text-transform:uppercase;margin-top:4px}

.status-bar{
  display:flex;justify-content:center;align-items:center;gap:14px;
  margin-top:12px;flex-wrap:wrap;
}
.status-item{
  display:flex;align-items:center;gap:6px;font-size:.65rem;letter-spacing:1px;
  color:var(--text-secondary);padding:5px 12px;border-radius:20px;
  background:rgba(255,255,255,0.03);border:1px solid rgba(255,255,255,0.05);
}
.pulse-dot{
  width:7px;height:7px;border-radius:50%;background:var(--accent-green);
  animation:pulse 2s ease-in-out infinite;flex-shrink:0;
}
@keyframes pulse{
  0%,100%{box-shadow:0 0 0 0 rgba(81,207,102,0.6)}
  50%{box-shadow:0 0 0 8px rgba(81,207,102,0)}
}

/* ─── Clay Card ─── */
.clay{
  background:linear-gradient(145deg,var(--bg-card-light),var(--bg-card));
  border-radius:var(--radius);padding:24px;
  box-shadow:
    10px 10px 24px var(--shadow-dark),
    -5px -5px 15px var(--shadow-light),
    inset 2px 2px 5px rgba(255,255,255,0.04),
    inset -2px -2px 5px rgba(0,0,0,0.25);
  border:1px solid rgba(255,255,255,0.04);
  transition:transform .35s cubic-bezier(.25,.46,.45,.94),box-shadow .35s ease;
  position:relative;overflow:hidden;
}
.clay:hover{
  transform:translateY(-4px);
  box-shadow:14px 14px 32px var(--shadow-dark),-8px -8px 20px var(--shadow-light),
    inset 2px 2px 5px rgba(255,255,255,0.06),inset -2px -2px 5px rgba(0,0,0,0.25);
}

/* ─── Layout ─── */
.container{max-width:900px;margin:0 auto;padding:20px 20px 40px;position:relative;z-index:1}

/* ─── Camera Feed Card ─── */
.cam-card{padding:0;overflow:hidden;margin-bottom:24px}
.cam-card .cam-header{
  display:flex;align-items:center;justify-content:space-between;
  padding:18px 24px 12px;
}
.cam-label{
  font-size:.7rem;font-weight:700;letter-spacing:2.5px;text-transform:uppercase;
  color:var(--text-secondary);display:flex;align-items:center;gap:8px;
}
.cam-label .icon{font-size:1.2rem}
.live-badge{
  font-size:.55rem;font-weight:800;letter-spacing:2px;padding:4px 10px;
  border-radius:12px;background:rgba(255,107,107,0.15);color:var(--accent-red);
  border:1px solid rgba(255,107,107,0.2);display:flex;align-items:center;gap:5px;
}
.live-badge .dot{
  width:6px;height:6px;border-radius:50%;background:var(--accent-red);
  animation:pulse-red 1.5s ease-in-out infinite;
}
@keyframes pulse-red{
  0%,100%{box-shadow:0 0 0 0 rgba(255,107,107,0.6)}
  50%{box-shadow:0 0 0 6px rgba(255,107,107,0)}
}
.stream-wrap{
  position:relative;width:100%;background:#000;
  min-height:280px;display:flex;align-items:center;justify-content:center;
}
.stream-wrap img{width:100%;height:auto;display:block}
.stream-wrap .loading{
  position:absolute;color:var(--text-muted);font-size:.75rem;letter-spacing:2px;
}
.fps-overlay{
  position:absolute;bottom:10px;right:14px;
  font-size:.6rem;font-weight:700;letter-spacing:1px;
  background:rgba(0,0,0,0.6);color:var(--accent-green);
  padding:3px 8px;border-radius:8px;
}

/* ─── Control Grid ─── */
.ctrl-grid{
  display:grid;grid-template-columns:repeat(3,1fr);gap:18px;
}

/* ─── Card Internals ─── */
.card-top{display:flex;align-items:center;gap:10px;margin-bottom:14px}
.card-icon{
  width:44px;height:44px;border-radius:14px;display:flex;align-items:center;
  justify-content:center;font-size:1.3rem;
  background:rgba(255,255,255,0.04);
  box-shadow:inset 1px 1px 3px rgba(255,255,255,0.06),
    inset -1px -1px 3px rgba(0,0,0,0.2),3px 3px 8px rgba(0,0,0,0.3);
}
.card-title{
  font-size:.65rem;font-weight:700;letter-spacing:2px;text-transform:uppercase;
  color:var(--text-secondary);
}
.card-sub{font-size:.55rem;color:var(--text-muted);letter-spacing:1px;margin-top:1px}

/* ─── Flash Toggle ─── */
.toggle-row{display:flex;align-items:center;gap:14px;margin-top:6px}
.toggle-track{
  width:56px;height:28px;border-radius:14px;cursor:pointer;position:relative;
  background:rgba(0,0,0,0.35);
  box-shadow:inset 2px 2px 5px rgba(0,0,0,0.4),inset -1px -1px 3px rgba(255,255,255,0.04);
  transition:background .3s ease;
}
.toggle-track.on{background:rgba(255,193,7,0.25);box-shadow:inset 2px 2px 5px rgba(0,0,0,0.2),0 0 12px rgba(255,193,7,0.15)}
.toggle-knob{
  position:absolute;top:3px;left:3px;width:22px;height:22px;border-radius:50%;
  background:linear-gradient(145deg,#3a3860,#2a2850);
  box-shadow:2px 2px 5px rgba(0,0,0,0.4),-1px -1px 3px rgba(255,255,255,0.06);
  transition:transform .3s cubic-bezier(.4,0,.2,1),background .3s;
}
.toggle-track.on .toggle-knob{transform:translateX(28px);background:linear-gradient(145deg,#ffc107,#e0a800)}
.toggle-label{font-size:.8rem;font-weight:700;color:var(--text-muted);transition:color .3s;letter-spacing:1px}
.toggle-label.on{color:var(--accent-amber)}

/* ─── Capture Button ─── */
.clay-btn{
  display:block;width:100%;padding:12px;margin-top:6px;
  border:none;cursor:pointer;font-size:.7rem;font-weight:700;letter-spacing:2px;
  text-transform:uppercase;color:var(--accent-cyan);
  background:rgba(34,211,238,0.08);border-radius:16px;
  box-shadow:4px 4px 10px rgba(0,0,0,0.3),-2px -2px 6px rgba(255,255,255,0.03),
    inset 1px 1px 2px rgba(255,255,255,0.05);
  transition:transform .2s,box-shadow .2s,background .2s;
}
.clay-btn:hover{background:rgba(34,211,238,0.14);transform:translateY(-2px)}
.clay-btn:active{transform:scale(.96);box-shadow:inset 2px 2px 5px rgba(0,0,0,0.3)}

/* ─── Motion Indicator ─── */
.motion-row{display:flex;align-items:center;gap:10px;margin-top:8px}
.motion-dot{
  width:12px;height:12px;border-radius:50%;
  background:var(--text-muted);transition:all .4s ease;flex-shrink:0;
}
.motion-dot.active{background:var(--accent-red);box-shadow:0 0 10px rgba(255,107,107,0.5);animation:pulse-red 1s infinite}
.motion-text{font-size:.8rem;font-weight:700;letter-spacing:1.5px;color:var(--text-muted);transition:color .4s}
.motion-text.active{color:var(--accent-red)}

/* ─── Footer ─── */
.footer{text-align:center;padding:8px 20px 28px;position:relative;z-index:1}
.footer p{font-size:.6rem;color:var(--text-muted);letter-spacing:2px}

/* ─── Card entrance ─── */
.clay{animation:cardIn .6s ease-out backwards}
.cam-card{animation-delay:.05s}
.ctrl-grid .clay:nth-child(1){animation-delay:.1s}
.ctrl-grid .clay:nth-child(2){animation-delay:.15s}
.ctrl-grid .clay:nth-child(3){animation-delay:.2s}
@keyframes cardIn{from{opacity:0;transform:translateY(24px) scale(.97)}to{opacity:1;transform:translateY(0) scale(1)}}

/* ─── Responsive ─── */
@media(max-width:700px){
  .header h1{font-size:1.8rem;letter-spacing:6px}
  .ctrl-grid{grid-template-columns:1fr}
  .container{padding:14px 14px 30px}
  .clay{border-radius:22px;padding:20px}
  .cam-card .cam-header{padding:14px 18px 10px}
}
</style>
</head>
<body>

<header class="header">
  <div class="logo-wrap"><h1>KNULL ZONE</h1></div>
  <p class="subtitle">ESP32-CAM Surveillance System</p>
  <div class="status-bar">
    <div class="status-item"><span class="pulse-dot"></span><span>LIVE STREAM</span></div>
    <div class="status-item" id="uptime-el">&#9201; UPTIME: 0s</div>
    <div class="status-item" id="fps-el">&#127916; 0 FPS</div>
  </div>
</header>

<div class="container">

  <!-- Camera Feed -->
  <div class="clay cam-card">
    <div class="cam-header">
      <span class="cam-label"><span class="icon">&#128247;</span> CAMERA FEED</span>
      <span class="live-badge"><span class="dot"></span>REC</span>
    </div>
    <div class="stream-wrap">
      <span class="loading" id="load-txt">CONNECTING...</span>
      <img id="cam-stream" alt="Camera Feed" style="display:none"
           onload="this.style.display='block';document.getElementById('load-txt').style.display='none'"
           onerror="this.style.display='none';document.getElementById('load-txt').style.display='block';var s=this;setTimeout(function(){s.src='http://'+location.hostname+':81/stream?r='+Date.now()},3000)">
      <span class="fps-overlay" id="fps-badge">-- FPS</span>
    </div>
  </div>

  <!-- Control Cards -->
  <div class="ctrl-grid">

    <!-- Flash Control -->
    <div class="clay">
      <div class="card-top">
        <div class="card-icon">&#9889;</div>
        <div>
          <div class="card-title">Flash Light</div>
          <div class="card-sub">GPIO 4 &bull; White LED</div>
        </div>
      </div>
      <div class="toggle-row">
        <div class="toggle-track" id="flash-track" onclick="toggleFlash()">
          <div class="toggle-knob"></div>
        </div>
        <span class="toggle-label" id="flash-lbl">OFF</span>
      </div>
    </div>

    <!-- Snapshot -->
    <div class="clay">
      <div class="card-top">
        <div class="card-icon">&#128248;</div>
        <div>
          <div class="card-title">Snapshot</div>
          <div class="card-sub">Capture JPEG Frame</div>
        </div>
      </div>
      <button class="clay-btn" onclick="takeSnap()">&#128247; CAPTURE</button>
    </div>

    <!-- Motion Detection -->
    <div class="clay">
      <div class="card-top">
        <div class="card-icon">&#128269;</div>
        <div>
          <div class="card-title">Motion Detect</div>
          <div class="card-sub">Frame-diff Analysis</div>
        </div>
      </div>
      <div class="motion-row">
        <span class="motion-dot" id="m-dot"></span>
        <span class="motion-text" id="m-txt">IDLE</span>
      </div>
    </div>

  </div>
</div>

<footer class="footer">
  <p>KNULL ZONE &bull; ESP32-CAM &bull; POWERED BY KALYANI</p>
</footer>

<script>
(function(){
  'use strict';

  // ─── Start stream ───
  var img = document.getElementById('cam-stream');
  img.src = 'http://' + location.hostname + ':81/stream';

  // ─── Flash toggle ───
  var flashState = false;
  window.toggleFlash = function() {
    flashState = !flashState;
    fetch('/flash?state=' + (flashState ? '1' : '0'))
      .then(function(r){ return r.json(); })
      .then(function(d){
        flashState = d.flash;
        updateFlashUI();
      })
      .catch(function(){ flashState = !flashState; });
    updateFlashUI();
  };
  function updateFlashUI() {
    var track = document.getElementById('flash-track');
    var lbl = document.getElementById('flash-lbl');
    if (flashState) {
      track.classList.add('on');
      lbl.textContent = 'ON';
      lbl.classList.add('on');
    } else {
      track.classList.remove('on');
      lbl.textContent = 'OFF';
      lbl.classList.remove('on');
    }
  }

  // ─── Snapshot ───
  window.takeSnap = function() {
    window.open('http://' + location.hostname + '/capture', '_blank');
  };

  // ─── Format uptime ───
  function fmtUp(s) {
    var d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60),sec=s%60;
    if(d>0)return d+'d '+h+'h '+m+'m';
    if(h>0)return h+'h '+m+'m '+sec+'s';
    if(m>0)return m+'m '+sec+'s';
    return sec+'s';
  }

  // ─── Fetch status data every 3s ───
  function fetchStatus() {
    fetch('/data')
      .then(function(r){ return r.json(); })
      .then(function(d){
        // Flash
        flashState = d.flash;
        updateFlashUI();
        // Motion
        var dot = document.getElementById('m-dot');
        var txt = document.getElementById('m-txt');
        if (d.motion) {
          dot.classList.add('active');
          txt.classList.add('active');
          txt.textContent = 'DETECTED!';
        } else {
          dot.classList.remove('active');
          txt.classList.remove('active');
          txt.textContent = 'IDLE';
        }
        // Uptime
        document.getElementById('uptime-el').innerHTML = '&#9201; UPTIME: ' + fmtUp(d.up);
        // FPS
        var fpsVal = d.fps.toFixed(1);
        document.getElementById('fps-el').innerHTML = '&#127916; ' + fpsVal + ' FPS';
        document.getElementById('fps-badge').textContent = fpsVal + ' FPS';
      })
      .catch(function(e){ console.warn('Status fetch error:', e); });
  }
  setInterval(fetchStatus, 3000);
  fetchStatus();

})();
</script>
</body>
</html>
)rawliteral";


// ═══════════════════════════════════════════════════════════════
//  CAMERA INITIALIZATION
// ═══════════════════════════════════════════════════════════════
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  // Use higher quality if PSRAM is available
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;    // 640x480
    config.jpeg_quality = 12;               // 0-63, lower=better
    config.fb_count     = 2;
    Serial.println("[CAM] PSRAM found — VGA, 2 frame buffers");
  } else {
    config.frame_size   = FRAMESIZE_QVGA;   // 320x240
    config.jpeg_quality = 15;
    config.fb_count     = 1;
    Serial.println("[CAM] No PSRAM — QVGA, 1 frame buffer");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init FAILED: 0x%x\n", err);
    return false;
  }

  // Fine-tune sensor settings
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
  }

  Serial.println("[CAM] Camera initialized successfully");
  return true;
}


// ═══════════════════════════════════════════════════════════════
//  HTTP HANDLERS
// ═══════════════════════════════════════════════════════════════

// ─── Serve main dashboard page ────────────────────────────────
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, index_html, strlen(index_html));
}

// ─── MJPEG stream (port 81) ──────────────────────────────────
#define PART_BOUNDARY "knullzone123boundary"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  char part_buf[80];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  Serial.println("[STREAM] Client connected");

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[STREAM] Frame capture failed");
      res = ESP_FAIL;
      break;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    frameCount++;

    if (res != ESP_OK) {
      Serial.println("[STREAM] Client disconnected");
      break;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);  // ~30 FPS cap
  }

  return res;
}

// ─── Single JPEG capture (snapshot) ──────────────────────────
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=knullzone_capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  Serial.println("[CAPTURE] Snapshot taken");
  return res;
}

// ─── Flash LED control ───────────────────────────────────────
static esp_err_t flash_handler(httpd_req_t *req) {
  char query[32];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (strstr(query, "state=1")) {
      flashOn = true;
      digitalWrite(FLASH_PIN, HIGH);
    } else {
      flashOn = false;
      digitalWrite(FLASH_PIN, LOW);
    }
  } else {
    // Toggle if no parameter
    flashOn = !flashOn;
    digitalWrite(FLASH_PIN, flashOn ? HIGH : LOW);
  }

  char resp[32];
  snprintf(resp, sizeof(resp), "{\"flash\":%s}", flashOn ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, resp, strlen(resp));
}

// ─── Status data JSON ────────────────────────────────────────
static esp_err_t data_handler(httpd_req_t *req) {
  const char *faceNames[] = {"idle", "blink", "surprised", "happy"};

  char json[200];
  snprintf(json, sizeof(json),
    "{\"flash\":%s,\"motion\":%s,\"face\":\"%s\",\"fps\":%.1f,\"up\":%lu}",
    flashOn ? "true" : "false",
    motionDetected ? "true" : "false",
    faceNames[currentFaceState],
    currentFps,
    (unsigned long)(millis() / 1000)
  );

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}


// ═══════════════════════════════════════════════════════════════
//  START HTTP SERVERS
// ═══════════════════════════════════════════════════════════════
void startServers() {
  // ─── Main server (port 80) ───
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;

  httpd_uri_t uri_index   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler,   .user_ctx = NULL };
  httpd_uri_t uri_capture = { .uri = "/capture",  .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };
  httpd_uri_t uri_flash   = { .uri = "/flash",    .method = HTTP_GET, .handler = flash_handler,   .user_ctx = NULL };
  httpd_uri_t uri_data    = { .uri = "/data",     .method = HTTP_GET, .handler = data_handler,    .user_ctx = NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &uri_index);
    httpd_register_uri_handler(camera_httpd, &uri_capture);
    httpd_register_uri_handler(camera_httpd, &uri_flash);
    httpd_register_uri_handler(camera_httpd, &uri_data);
    Serial.println("[HTTP] Main server started on port 80");
  } else {
    Serial.println("[HTTP] Failed to start main server!");
  }

  // ─── Stream server (port 81) ───
  config.server_port = 81;
  config.ctrl_port   = config.ctrl_port + 1;

  httpd_uri_t uri_stream = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &uri_stream);
    Serial.println("[HTTP] Stream server started on port 81");
  } else {
    Serial.println("[HTTP] Failed to start stream server!");
  }
}


// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║       K N U L L   Z O N E   C A M       ║");
  Serial.println("║     Surveillance + Animated OLED Face    ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println();

  // ─── Flash LED pin ───
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  // ─── Initialize camera ───
  if (!initCamera()) {
    Serial.println("[FATAL] Camera init failed! Restarting in 5s...");
    delay(5000);
    ESP.restart();
  }

  // ─── Initialize OLED ───
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(14, 24, "KNULL ZONE");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(22, 40, "Initializing WiFi...");
  u8g2.sendBuffer();
  Serial.println("[OLED] Display initialized (GPIO14=SDA, GPIO15=SCL)");

  // ─── WiFi Access Point ───
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  delay(300);

  IPAddress myIP = WiFi.softAPIP();
  Serial.println();
  Serial.print("[WiFi] SSID      : "); Serial.println(ap_ssid);
  Serial.print("[WiFi] Password  : "); Serial.println(ap_pass);
  Serial.print("[WiFi] Page      : http://"); Serial.println(myIP);
  Serial.print("[WiFi] Stream    : http://"); Serial.print(myIP); Serial.println(":81/stream");
  Serial.println();

  // Show IP on OLED
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(14, 20, "KNULL ZONE");
  u8g2.setFont(u8g2_font_5x7_tr);
  char ipBuf[30];
  snprintf(ipBuf, sizeof(ipBuf), "IP: %s", myIP.toString().c_str());
  u8g2.drawStr(20, 38, ipBuf);
  u8g2.drawStr(28, 52, "READY!");
  u8g2.sendBuffer();
  delay(1500);

  // ─── Start HTTP servers ───
  startServers();

  // ─── Start OLED animation task on Core 0 ───
  xTaskCreatePinnedToCore(
    oledAnimationTask,   // Task function
    "OLEDFace",          // Name
    8192,                // Stack size (bytes)
    NULL,                // Parameters
    1,                   // Priority
    NULL,                // Task handle
    0                    // Core 0
  );
  Serial.println("[OLED] Animation task started on Core 0");

  Serial.println();
  Serial.println("════════════════════════════════════════════");
  Serial.println("  KNULL ZONE CAM is LIVE!");
  Serial.println("  Connect to WiFi and open the dashboard.");
  Serial.println("════════════════════════════════════════════");
  Serial.println();
}


// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP  (Core 1)
// ═══════════════════════════════════════════════════════════════
void loop() {
  // ─── Motion detection every 3 seconds ───
  static unsigned long lastMotion = 0;
  if (millis() - lastMotion > 3000) {
    lastMotion = millis();
    checkMotion();
  }

  // ─── FPS calculation every second ───
  if (millis() - lastFpsCalc > 1000) {
    currentFps = (float)(frameCount - lastFrameCount) * 1000.0f / (float)(millis() - lastFpsCalc);
    lastFrameCount = frameCount;
    lastFpsCalc = millis();
  }

  delay(10);
}
