# 🐍 Snake Bot: Physical Telemetry & Locomotion Firmware

This folder contains the complete firmware for the **Snake Bot** (SERP-X physical platform)—an articulated, snake-like robot engineered for subterranean search-and-rescue navigation. It is built around a multi-controller architecture coordinating real-time gas monitoring, thermal/video surveillance, environmental feedback, and servo-articulated locomotion.

---

## 📂 Firmware Modules

The Snake Bot utilizes three dedicated firmware subsystems located in this directory:

### 1. 🎛️ [KnullZone](file:///c:/Users/Hrishikesh%20Jha/Desktop/Kalyani/Snake%20Bot/KnullZone/) (Environmental Node)
An **ESP32-C3** based system that tracks ambient safety parameters and visualizes status using animated eyes.
* **Firmware File**: [sensors.ino](file:///c:/Users/Hrishikesh%20Jha/Desktop/Kalyani/Snake%20Bot/KnullZone/sensors.ino)
* **Access Point SSID**: `KNULL_ZONE` | **Password**: `knull1234`
* **Local Webpage**: `http://192.168.4.1` (environmental monitors and logs)
* **Visuals**: Draws a 1.3" OLED face with animated *Mochi Eyes* containing 8 expressions (Normal, Happy, Blink, Surprised, Sleepy, Love, Wink, Angry).
* **Hardware Wiring**:
  | Device | Interface / Pin | Purpose |
  | :--- | :--- | :--- |
  | **MQ-4 Methane Sensor** | GPIO 2 (ADC) | Gas detection ($CH_4$) |
  | **MQ-7 CO Sensor** | GPIO 3 (ADC) | Gas detection ($CO$) |
  | **MQ-135 Gas Sensor** | GPIO 4 (ADC) | Air Quality / Multi-pollutant |
  | **DHT22 Sensor** | GPIO 5 (Digital) | Temperature & Humidity |
  | **Active Alert Module** | GPIO 6 (Digital Output) | Combined Buzzer + Warning LED |
  | **1.3" OLED Display** | GPIO 8 (SDA), GPIO 9 (SCL) | SH1106 Display via I2C |

---

### 2. 📹 [KnullZoneCam](file:///c:/Users/Hrishikesh%20Jha/Desktop/Kalyani/Snake%20Bot/KnullZoneCam/) (Surveillance Node)
An **ESP32-CAM (AI-Thinker)** unit managing video streaming, flash control, and local motion detection.
* **Firmware File**: [camera.ino](file:///c:/Users/Hrishikesh%20Jha/Desktop/Kalyani/Snake%20Bot/KnullZoneCam/camera.ino)
* **Access Point SSID**: `KNULL_ZONE_CAM` | **Password**: `knull1234`
* **Main Dashboard**: `http://192.168.4.1` (Cyberpunk-styled interface)
* **Video Stream**: `http://192.168.4.1:81/stream` (MJPEG video stream)
* **HTTP / JSON API Endpoints**:
  - `/capture` - Returns a single high-quality JPEG snapshot.
  - `/flash?state=1` or `/flash?state=0` - Enters flash LED state.
  - `/data` - Returns real-time JSON parameters:
    ```json
    {
      "flash": false,
      "motion": false,
      "face": "idle",
      "fps": 24.5,
      "up": 182
    }
    ```
* **Visuals**: Renders a state-dependent animated OLED face (Idle, Blink, Surprised on motion detection, and Happy after a successful detection event).
* **Hardware Wiring**:
  | Device | Interface / Pin | Purpose |
  | :--- | :--- | :--- |
  | **OV2640 Camera** | DVP Bus (Dedicated) | 2MP image capture |
  | **Flash LED** | GPIO 4 | Built-in high-intensity white light |
  | **1.3" OLED Display** | GPIO 14 (SDA), GPIO 15 (SCL) | Software I2C face animation |

---

### 3. ⚙️ [Locomotion Controller](file:///c:/Users/Hrishikesh%20Jha/Desktop/Kalyani/Snake%20Bot/KnullZone/) (Mobility Node)
An Arduino-compatible controller driving the H-Bridge crawler and steering joints.
* **Firmware File**: [mobility.ino](file:///c:/Users/Hrishikesh%20Jha/Desktop/Kalyani/Snake%20Bot/KnullZone/mobility.ino)
* **Baud Rate**: `9600` (Serial Interface)
* **Serial Commands**:
  - `F` / `R` / `S` - Forward propulsion, Reverse propulsion, or Stop.
  - `A` / `B` / `C` - Servo 1 Angle to $0^\circ$ / $90^\circ$ / $180^\circ$.
  - `D` / `E` / `G` - Servo 2 Angle to $0^\circ$ / $90^\circ$ / $180^\circ$.
* **Hardware Wiring**:
  | Device | Interface / Pin | Purpose |
  | :--- | :--- | :--- |
  | **Motor IN1 / IN2** | Pin 8 / Pin 9 | H-Bridge direction inputs |
  | **Motor Enable (ENA)** | Pin 10 | Speed control (PWM) |
  | **Locomotion Servo 1** | Pin 5 | Articulating segment Joint A |
  | **Locomotion Servo 2** | Pin 6 | Articulating segment Joint B |

---

## 🛠️ Calibration & Compilation Notes

1. **Libraries Required (via Arduino Library Manager)**:
   - `U8g2` by olikraus (for SH1106 OLED screens)
   - `DHT sensor library` by Adafruit
   - `Adafruit Unified Sensor`
2. **Board Configurations**:
   - For `sensors.ino`: Select **ESP32-C3 Dev Module** inside the Arduino IDE. Keep default flash sizes and partition schemes.
   - For `camera.ino`: Select **AI Thinker ESP32-CAM**. Ensure `PSRAM` is enabled in configuration if your board features it (speeds up frame rate and increases quality to VGA).
   - For `mobility.ino`: Select your standard controller board (e.g. **Arduino Uno** or **Nano**).
3. **Motion Detection Calibration**:
   - The camera uses a frame-size variation heuristic. If the system triggers a false motion alarm due to light changes, edit `#define MOTION_THRESHOLD 0.12f` in [camera.ino](file:///c:/Users/Hrishikesh%20Jha/Desktop/Kalyani/Snake%20Bot/KnullZoneCam/camera.ino) to a higher value (e.g. `0.18f`).
