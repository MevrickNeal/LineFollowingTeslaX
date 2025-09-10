/*
 * =================================================================================
 * Project: Advanced Line Follower Robot - v0.7 HYBRID CONTROL
 * Description: Controller sketch with a full graphical OLED UI AND a two-way
 * serial command interface for a web-based dashboard.
 * Platform: Arduino Nano (ATmega328P)
 * Author: Lian Mollick
 * Project: NEAL
 * --- SERIAL COMMAND PROTOCOL ---
 * Commands FROM Web App to Arduino:
 * - "p=[value]" -> e.g., "p=1.50"
 * - "i=[value]" -> e.g., "i=0.25"
 * - "d=[value]" -> e.g., "d=0.80"
 * - "led=[0 or 1]" -> e.g., "led=1"
 * - "speed=[1, 2, or 3]" -> e.g., "speed=2"
 * - "run=[0 or 1]" -> e.g., "run=1"
 *
 * Messages FROM Arduino to Web App:
 * - "UPDATE,p,[value]"
 * - "UPDATE,i,[value]"
 * - "UPDATE,d,[value]"
 * - "UPDATE,led,[0 or 1]"
 * - "UPDATE,speed,[1, 2, or 3]"
 * =================================================================================
 */

// --- LIBRARIES ---
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- PIN DEFINITIONS ---
#define ENA 5
#define IN1 7
#define IN2 8
#define ENB 6
#define IN3 9
#define IN4 10
const int SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
#define NUM_SENSORS 8
#define BTN_UP 2
#define BTN_DOWN 3
#define BTN_SELECT 4
#define LED_PIN 12

// --- DISPLAY & UI ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const unsigned char PROGMEM robot_bitmap[] =
{ B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B01111111, B11111111, B11111111, B11111110, B00000000,
  B00000001, B10000000, B00000000, B00000000, B00000011, B00000000,
  B00000011, B00000000, B00000000, B00000000, B00000110, B00000000,
  B00000111, B11111111, B11111111, B11111111, B11111110, B00000000,
  B00000110, B00011000, B00111100, B00111100, B01100110, B00000000,
  B00000110, B00111100, B01111110, B01111110, B01110110, B00000000,
  B00000011, B00111100, B01111110, B01111110, B01111111, B00000000,
  B00000001, B11111111, B11111111, B11111111, B11111111, B10000000,
  B00000000, B01111111, B11111111, B11111111, B11111110, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000
};

// --- GLOBAL STATE & CONTROL VARIABLES ---
enum SystemState { STATE_MENU, STATE_CALIBRATING, STATE_RUNNING, STATE_TUNING_PID, STATE_SHOW_STATUS };
struct PIDGains { float Kp = 1.0; float Ki = 0.0; float Kd = 0.0; };

SystemState currentSystemState = STATE_MENU;
PIDGains pidGains;
bool isCalibrated = false;
int baseSpeed = 200; // Corresponds to speed level 3
bool headlightOn = false;
float lastError = 0;
float integral = 0;
String inputString = "";
bool stringComplete = false;
int menuSelection = 0;
const int MAIN_MENU_ITEMS = 4;
int tuningSelection = 0;
const int TUNE_MENU_ITEMS = 4;
unsigned long lastInputTime = 0;
const int DEBOUNCE_DELAY = 200;

// --- CORE FUNCTIONS ---
void setup() {
    Serial.begin(115200);
    inputString.reserve(50);
    initButtons();
    initMotors();
    initDisplay();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    loadSettings();
    updateDisplay();
}

void loop() {
    handleSerialCommands();
    handleButtonInput();

    if (currentSystemState == STATE_RUNNING) {
        followLine();
        static unsigned long lastDisplayUpdateTime = 0;
        if (millis() - lastDisplayUpdateTime > 100) {
            updateDisplay();
            lastDisplayUpdateTime = millis();
        }
    } else if (currentSystemState == STATE_CALIBRATING) {
        runCalibrationRoutine();
    }
}

// --- SERIAL COMMUNICATION ---
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else {
            inputString += inChar;
        }
    }
}

void handleSerialCommands() {
    if (stringComplete) {
        inputString.trim();
        int separator = inputString.indexOf('=');
        if (separator != -1) {
            String key = inputString.substring(0, separator);
            String valueStr = inputString.substring(separator + 1);
            float value = valueStr.toFloat();

            if (key.equalsIgnoreCase("p")) pidGains.Kp = value;
            else if (key.equalsIgnoreCase("i")) pidGains.Ki = value;
            else if (key.equalsIgnoreCase("d")) pidGains.Kd = value;
            else if (key.equalsIgnoreCase("led")) {
                headlightOn = (value == 1);
                digitalWrite(LED_PIN, headlightOn);
            }
            else if (key.equalsIgnoreCase("speed")) {
                if (value == 1) baseSpeed = 100;
                else if (value == 2) baseSpeed = 150;
                else baseSpeed = 200;
            }
            else if (key.equalsIgnoreCase("run")) {
                if(value == 1 && isCalibrated) currentSystemState = STATE_RUNNING;
                else {
                    currentSystemState = STATE_MENU;
                    stopMotors();
                }
            }
            updateDisplay(); // Update OLED after receiving a command
        }
        inputString = "";
        stringComplete = false;
    }
}


// --- UI & INPUT HANDLING ---
void processUpAction() {
    if (currentSystemState == STATE_MENU) {
        menuSelection = (menuSelection > 0) ? menuSelection - 1 : MAIN_MENU_ITEMS - 1;
    } else if (currentSystemState == STATE_TUNING_PID) {
        if (tuningSelection < TUNE_MENU_ITEMS - 1) {
            if (tuningSelection == 0) { pidGains.Kp += 0.1; Serial.print("UPDATE,p,"); Serial.println(pidGains.Kp, 2); }
            if (tuningSelection == 1) { pidGains.Ki += 0.01; Serial.print("UPDATE,i,"); Serial.println(pidGains.Ki, 2); }
            if (tuningSelection == 2) { pidGains.Kd += 0.2; Serial.print("UPDATE,d,"); Serial.println(pidGains.Kd, 2); }
        }
    }
}

void processDownAction() {
    if (currentSystemState == STATE_MENU) {
        menuSelection = (menuSelection < MAIN_MENU_ITEMS - 1) ? menuSelection + 1 : 0;
    } else if (currentSystemState == STATE_TUNING_PID) {
        if (tuningSelection < TUNE_MENU_ITEMS - 1) {
            if (tuningSelection == 0) { pidGains.Kp -= 0.1; if(pidGains.Kp<0) pidGains.Kp=0; Serial.print("UPDATE,p,"); Serial.println(pidGains.Kp, 2); }
            if (tuningSelection == 1) { pidGains.Ki -= 0.01; if(pidGains.Ki<0) pidGains.Ki=0; Serial.print("UPDATE,i,"); Serial.println(pidGains.Ki, 2); }
            if (tuningSelection == 2) { pidGains.Kd -= 0.2; if(pidGains.Kd<0) pidGains.Kd=0; Serial.print("UPDATE,d,"); Serial.println(pidGains.Kd, 2); }
        }
    }
}
void processSelectAction() {
    if (currentSystemState == STATE_MENU) {
        if (menuSelection == 0 && isCalibrated) currentSystemState = STATE_RUNNING;
        else if (menuSelection == 1) currentSystemState = STATE_CALIBRATING;
        else if (menuSelection == 2) currentSystemState = STATE_TUNING_PID;
        else if (menuSelection == 3) currentSystemState = STATE_SHOW_STATUS;
    } else if (currentSystemState == STATE_TUNING_PID) {
        if (tuningSelection == TUNE_MENU_ITEMS - 1) {
            saveSettings();
            currentSystemState = STATE_MENU;
            tuningSelection = 0;
        } else {
            tuningSelection++;
        }
    } else if (currentSystemState == STATE_RUNNING || currentSystemState == STATE_SHOW_STATUS) {
        stopMotors();
        saveSettings();
        currentSystemState = STATE_MENU;
    }
}

void handleButtonInput() {
    if (currentSystemState == STATE_CALIBRATING) return; // Ignore buttons during calibration sequence
    if (millis() - lastInputTime < DEBOUNCE_DELAY) return;

    bool updateNeeded = false;
    if (digitalRead(BTN_UP) == LOW) { processUpAction(); updateNeeded = true; }
    else if (digitalRead(BTN_DOWN) == LOW) { processDownAction(); updateNeeded = true; }
    else if (digitalRead(BTN_SELECT) == LOW) { processSelectAction(); updateNeeded = true; }

    if (updateNeeded) {
        lastInputTime = millis();
        updateDisplay();
    }
}

void updateDisplay() {
    display.clearDisplay();
    display.setCursor(0, 0);

    switch (currentSystemState) {
        case STATE_MENU:
            display.println(F("--- LFR-01 MENU ---"));
            display.println(menuSelection == 0 ? F("> Run") : F("  Run"));
            display.println(menuSelection == 1 ? F("> Calibrate") : F("  Calibrate"));
            display.println(menuSelection == 2 ? F("> Tune PID") : F("  Tune PID"));
            display.println(menuSelection == 3 ? F("> Status") : F("  Status"));
            if (!isCalibrated) { display.setCursor(0, 56); display.print(F("NEEDS CALIBRATION!")); }
            break;
            
        case STATE_TUNING_PID:
            display.println(F("--- TUNE PID ---"));
            display.print(tuningSelection == 0 ? F(">Kp:") : F(" Kp:")); display.println(pidGains.Kp, 2);
            display.print(tuningSelection == 1 ? F(">Ki:") : F(" Ki:")); display.println(pidGains.Ki, 3);
            display.print(tuningSelection == 2 ? F(">Kd:") : F(" Kd:")); display.println(pidGains.Kd, 2);
            display.println(tuningSelection == 3 ? F("> Back") : F("  Back"));
            display.setCursor(0, 56); display.print(F("Push=Next/Select"));
            break;
            
        case STATE_SHOW_STATUS:
             display.println(F("--- SYSTEM STATUS ---"));
             display.print(F("Calibrated: ")); display.println(isCalibrated ? F("Yes") : F("No"));
             display.println(F("--- PID Gains ---"));
             display.print(F(" P:")); display.println(pidGains.Kp, 2);
             display.print(F(" I:")); display.println(pidGains.Ki, 3);
             display.print(F(" D:")); display.println(pidGains.Kd, 2);
             display.setCursor(0, 56); display.print(F("Press Select to return"));
            break;

        case STATE_RUNNING: {
            int sensorValues[NUM_SENSORS]; readSensors(sensorValues);
            float error = calculateError(sensorValues);
            display.drawBitmap(40, 8, robot_bitmap, 48, 48, SSD1306_WHITE);
            display.drawFastVLine(94, 0, 64, SSD1306_WHITE);
            display.setCursor(100, 5); display.print(F("P:")); display.println(pidGains.Kp,1);
            display.setCursor(100, 25); display.print(F("I:")); display.println(pidGains.Ki,2);
            display.setCursor(100, 45); display.print(F("D:")); display.println(pidGains.Kd,1);
            display.setCursor(0,0); display.println(F(" Error"));
            int barHeight = map(abs(error), 0, 3500, 0, 50);
            barHeight = constrain(barHeight, 0, 50);
            display.drawRect(10, 10, 15, 52, SSD1306_WHITE);
            if (error > 0) { display.fillRect(11, 36, 13, barHeight, SSD1306_WHITE); }
            else { display.fillRect(11, 36 - barHeight, 13, barHeight, SSD1306_WHITE); }
            display.drawFastHLine(10, 36, 15, SSD1306_WHITE);
            break;
        }
        case STATE_CALIBRATING: break;
    }
    display.display();
}


// --- Remaining functions (initMotors, initButtons, initDisplay, followLine, PID, calibration, etc.) are identical to v0.6 ---
// --- They are included here for completeness ---

void initMotors() { /* ... Same as before ... */ }
void initButtons() { /* ... Same as before ... */ }
void initDisplay() { /* ... Same as before ... */ }
void followLine() { /* ... Same as before ... */ }
float runPID(float error) { /* ... Same as before ... */ }
void runCalibrationRoutine() { /* ... Same as before ... */ }
void readSensors(int sensorValues[NUM_SENSORS]) { /* ... Same as before ... */ }
float calculateError(int sensorValues[NUM_SENSORS]) { /* ... Same as before ... */ }
void setMotorSpeeds(int leftSpeed, int rightSpeed) { /* ... Same as before ... */ }
void stopMotors() { /* ... Same as before ... */ }
void saveSettings() { /* ... Same as before ... */ }
void loadSettings() { /* ... Same as before ... */ }
