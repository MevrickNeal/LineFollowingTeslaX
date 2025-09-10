/*
 * =================================================================================
 * Project: Advanced Line Follower Robot - v0.5 with Buttons & Encoder
 * Description: Controller sketch using an SSD1306 OLED, three push buttons, AND
 * a KY-040 Rotary Encoder for a flexible user interface. (Memory Optimized)
 * Platform: Arduino Nano (ATmega328P)
 * Author: Lian Mollick
 * Project: NEAL
 * Hardware:
 * - 8x TCRT5000 (or similar) analog IR sensors
 * - L298N Motor Driver
 * - SSD1306 I2C OLED Display
 * - 3x Push Buttons (Up, Down, Select)
 * - KY-040 Rotary Encoder (CLK, DT, SW)
 * =================================================================================
 */

// --- LIBRARIES ---
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- DEBUG FLAG ---
const bool DEBUG = true;

// --- PIN DEFINITIONS ---
// Motor A (Left)
#define ENA 5
#define IN1 7
#define IN2 8
// Motor B (Right)
#define ENB 6
#define IN3 9
#define IN4 10

// Sensor Pins (Connect S1-S8 to A0-A7)
const int SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
#define NUM_SENSORS 8

// Original Push Buttons
#define BTN_UP 2
#define BTN_DOWN 3
#define BTN_SELECT 4

// **NEW** Rotary Encoder Pins (using non-interrupt pins)
#define ENCODER_CLK A6
#define ENCODER_DT A7
#define ENCODER_SW A5

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- ENUMS & STRUCTS ---
enum SystemState { STATE_MENU, STATE_CALIBRATING, STATE_RUNNING, STATE_TUNING_PID, STATE_SHOW_STATUS };
struct PIDGains { float Kp = 1.0; float Ki = 0.0; float Kd = 0.0; };

// --- GLOBAL STATE VARIABLES ---
SystemState currentSystemState = STATE_MENU;
PIDGains pidGains;
bool isCalibrated = false;
int baseSpeed = 150;
int sensorMinValues[NUM_SENSORS];
int sensorMaxValues[NUM_SENSORS];

// UI Navigation
int menuSelection = 0;
const int MAIN_MENU_ITEMS = 4;
int tuningSelection = 0;
const int TUNE_MENU_ITEMS = 4; // Kp, Ki, Kd, Back

// Input Debouncing & Encoder State
unsigned long lastInputTime = 0;
const int DEBOUNCE_DELAY = 200;
int encoderCLK_lastState = 0; // For polling the encoder

// --- DIAGNOSTIC FUNCTION ---
int getFreeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// --- CORE FUNCTIONS ---

void setup() {
    if (DEBUG) { Serial.begin(115200); }
    initButtonsAndEncoder();
    initMotors();
    initDisplay();
    loadSettings();
    updateDisplay();
}

void loop() {
    handleButtonInput();
    handleEncoderInput(); // Now we handle both inputs

    switch (currentSystemState) {
        case STATE_RUNNING:
            followLine();
            updateDisplay();
            break;
        case STATE_CALIBRATING:
            runCalibrationRoutine();
            break;
        default:
            break;
    }
}

// --- INITIALIZATION ---

void initMotors() {
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    stopMotors();
}

void initButtonsAndEncoder() {
    // Original Buttons
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);
    // New Encoder
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(ENCODER_SW, INPUT_PULLUP);
    encoderCLK_lastState = digitalRead(ENCODER_CLK);
}

void initDisplay() {
    if(DEBUG){ Serial.print(F("Free RAM: ")); Serial.println(getFreeRam()); }
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        if (DEBUG) { Serial.println(F("SSD1306 allocation failed")); }
        while(true);
    }
    if(DEBUG){ Serial.println(F("Display Initialized!")); }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Line Follower v0.5"));
    display.println(F("Buttons + Encoder"));
    display.display();
    delay(1500);
}

// --- UI & INPUT HANDLING ---

void processUpAction() {
    if (currentSystemState == STATE_MENU) {
        menuSelection = (menuSelection > 0) ? menuSelection - 1 : MAIN_MENU_ITEMS - 1;
    } else if (currentSystemState == STATE_TUNING_PID) {
        // Only adjust values if not on the "Back" option
        if (tuningSelection < TUNE_MENU_ITEMS - 1) {
            if (tuningSelection == 0) pidGains.Kp += 0.1;
            if (tuningSelection == 1) pidGains.Ki += 0.01;
            if (tuningSelection == 2) pidGains.Kd += 0.2;
        }
    }
}

void processDownAction() {
    if (currentSystemState == STATE_MENU) {
        menuSelection = (menuSelection < MAIN_MENU_ITEMS - 1) ? menuSelection + 1 : 0;
    } else if (currentSystemState == STATE_TUNING_PID) {
        // Only adjust values if not on the "Back" option
        if (tuningSelection < TUNE_MENU_ITEMS - 1) {
            if (tuningSelection == 0) pidGains.Kp -= 0.1;
            if (tuningSelection == 1) pidGains.Ki -= 0.01;
            if (tuningSelection == 2) pidGains.Kd -= 0.2;
            if(pidGains.Kp < 0) pidGains.Kp = 0;
            if(pidGains.Ki < 0) pidGains.Ki = 0;
            if(pidGains.Kd < 0) pidGains.Kd = 0;
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
        // If we are on "Back", save and exit. Otherwise, cycle to the next item.
        if (tuningSelection == TUNE_MENU_ITEMS - 1) {
            saveSettings();
            currentSystemState = STATE_MENU;
            tuningSelection = 0; // Reset for next time
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
    if (millis() - lastInputTime < DEBOUNCE_DELAY) return;

    bool updateNeeded = false;
    if (digitalRead(BTN_UP) == LOW) {
        processUpAction();
        updateNeeded = true;
    } else if (digitalRead(BTN_DOWN) == LOW) {
        processDownAction();
        updateNeeded = true;
    } else if (digitalRead(BTN_SELECT) == LOW) {
        processSelectAction();
        updateNeeded = true;
    }

    if (updateNeeded) {
        lastInputTime = millis();
        updateDisplay();
    }
}

void handleEncoderInput() {
    bool updateNeeded = false;

    // Handle encoder switch press (acts as SELECT)
    if (digitalRead(ENCODER_SW) == LOW && (millis() - lastInputTime > DEBOUNCE_DELAY)) {
        processSelectAction();
        updateNeeded = true;
    }

    // Handle encoder rotation (polling method)
    int clkState = digitalRead(ENCODER_CLK);
    if (clkState != encoderCLK_lastState && clkState == LOW) {
        if (digitalRead(ENCODER_DT) == LOW) {
            processDownAction(); // Turned Clockwise
        } else {
            processUpAction(); // Turned Counter-Clockwise
        }
        updateNeeded = true;
    }
    encoderCLK_lastState = clkState;
    
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
            display.println(F("--- MAIN MENU ---"));
            display.println(menuSelection == 0 ? F("> Run") : F("  Run"));
            display.println(menuSelection == 1 ? F("> Calibrate") : F("  Calibrate"));
            display.println(menuSelection == 2 ? F("> Tune PID") : F("  Tune PID"));
            display.println(menuSelection == 3 ? F("> Status") : F("  Status"));
            if (!isCalibrated) {
                display.setCursor(0, 56);
                display.print(F("NEEDS CALIBRATION!"));
            }
            break;
            
        case STATE_TUNING_PID:
            display.println(F("--- TUNE PID ---"));
            display.print(tuningSelection == 0 ? F(">Kp:") : F(" Kp:")); display.println(pidGains.Kp, 2);
            display.print(tuningSelection == 1 ? F(">Ki:") : F(" Ki:")); display.println(pidGains.Ki, 3);
            display.print(tuningSelection == 2 ? F(">Kd:") : F(" Kd:")); display.println(pidGains.Kd, 2);
            display.println(tuningSelection == 3 ? F("> Back") : F("  Back"));
            display.setCursor(0, 56);
            display.print(F("Push=Next/Select"));
            break;
            
        case STATE_SHOW_STATUS:
             display.println(F("--- SYSTEM STATUS ---"));
             display.print(F("Calibrated: ")); display.println(isCalibrated ? F("Yes") : F("No"));
             display.println(F("--- PID Gains ---"));
             display.print(F(" P:")); display.println(pidGains.Kp, 2);
             display.print(F(" I:")); display.println(pidGains.Ki, 3);
             display.print(F(" D:")); display.println(pidGains.Kd, 2);
             display.setCursor(0, 56);
             display.print(F("Press Select to return"));
            break;

        case STATE_RUNNING: {
            int sensorValues[NUM_SENSORS];
            readSensors(sensorValues);
            float error = calculateError(sensorValues);
            display.println(F("--- RUNNING ---"));
            display.print(F("Error: ")); display.println(error);
            display.println(F("----------------"));
            display.print(F("P:")); display.print(pidGains.Kp);
            display.print(F(" I:")); display.print(pidGains.Ki);
            display.print(F(" D:")); display.println(pidGains.Kd);
            display.setCursor(0, 56);
            display.print(F("Press Select to STOP"));
            break;
        }
        case STATE_CALIBRATING:
            // This is handled by the runCalibrationRoutine itself
            break;
    }
    display.display();
}

// --- CONTROL & LOGIC ---

void followLine() {
    int sensorValues[NUM_SENSORS];
    readSensors(sensorValues);
    float error = calculateError(sensorValues);
    float adjustment = runPID(error);
    setMotorSpeeds(baseSpeed - adjustment, baseSpeed + adjustment);
}

float runPID(float error) {
    static float lastError = 0;
    static float integral = 0;
    
    integral += error;
    integral = constrain(integral, -200, 200);

    float derivative = error - lastError;
    float output = (pidGains.Kp * error) + (pidGains.Ki * integral) + (pidGains.Kd * derivative);
    lastError = error;
    return output;
}

// --- SENSORS & CALIBRATION ---

void runCalibrationRoutine() {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("--- CALIBRATION ---"));
    display.println(F("Place on WHITE"));
    display.println(F("and press Select."));
    display.display();

    while(digitalRead(BTN_SELECT) == HIGH && digitalRead(ENCODER_SW) == HIGH) { /* Wait for press */ }
    lastInputTime = millis();

    display.println(F("Calibrating..."));
    display.display();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMinValues[i] = 1023;
        sensorMaxValues[i] = 0;
    }
    
    setMotorSpeeds(-80, 80);
    for(int i = 0; i < 250; i++) {
        int readings[NUM_SENSORS];
        readSensors(readings);
        for(int j=0; j<NUM_SENSORS; j++){
            if(readings[j] < sensorMinValues[j]) sensorMinValues[j] = readings[j];
            if(readings[j] > sensorMaxValues[j]) sensorMaxValues[j] = readings[j];
        }
        delay(20);
    }
    stopMotors();

    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("--- CALIBRATION ---"));
    display.println(F("Place on BLACK"));
    display.println(F("and press Select."));
    display.display();

    while(digitalRead(BTN_SELECT) == HIGH && digitalRead(ENCODER_SW) == HIGH) { /* Wait */ }
    lastInputTime = millis();
    
    display.println(F("Calibrating..."));
    display.display();

    setMotorSpeeds(-80, 80);
    for(int i = 0; i < 250; i++) {
        int readings[NUM_SENSORS];
        readSensors(readings);
        for(int j=0; j<NUM_SENSORS; j++){
            if(readings[j] < sensorMinValues[j]) sensorMinValues[j] = readings[j];
            if(readings[j] > sensorMaxValues[j]) sensorMaxValues[j] = readings[j];
        }
        delay(20);
    }
    stopMotors();

    isCalibrated = true;
    currentSystemState = STATE_MENU;
    saveSettings();
    updateDisplay();
}

void readSensors(int sensorValues[NUM_SENSORS]) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(SENSOR_PINS[i]);
    }
}

float calculateError(int sensorValues[NUM_SENSORS]) {
    float avg = 0;
    float sum = 0;
    bool onLine = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        float value = map(sensorValues[i], sensorMinValues[i], sensorMaxValues[i], 0, 1000);
        value = constrain(value, 0, 1000);
        
        if (value > 200) { onLine = true; }
        
        avg += value * (i * 1000);
        sum += value;
    }

    if (!onLine || sum == 0) { return 0; }
    
    float center = (NUM_SENSORS - 1) * 500.0;
    return (avg / sum) - center;
}

// --- HARDWARE ABSTRACTION ---

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    digitalWrite(IN1, leftSpeed >= 0 ? HIGH : LOW);
    digitalWrite(IN2, leftSpeed >= 0 ? LOW : HIGH);
    analogWrite(ENA, abs(leftSpeed));

    digitalWrite(IN3, rightSpeed >= 0 ? HIGH : LOW);
    digitalWrite(IN4, rightSpeed >= 0 ? LOW : HIGH);
    analogWrite(ENB, abs(rightSpeed));
}

void stopMotors() {
    setMotorSpeeds(0, 0);
}

// --- EEPROM PERSISTENCE ---

void saveSettings() {
    if (DEBUG) { Serial.print(F("Saving settings to EEPROM...")); }
    EEPROM.put(0, pidGains);
    EEPROM.put(sizeof(pidGains), sensorMinValues);
    EEPROM.put(sizeof(pidGains) + sizeof(sensorMinValues), sensorMaxValues);
    if (DEBUG) { Serial.println(F(" Done.")); }
}

void loadSettings() {
    if (DEBUG) { Serial.print(F("Loading settings from EEPROM...")); }
    EEPROM.get(0, pidGains);
    EEPROM.get(sizeof(pidGains), sensorMinValues);
    EEPROM.get(sizeof(pidGains) + sizeof(sensorMinValues), sensorMaxValues);
    
    if(isnan(pidGains.Kp) || pidGains.Kp > 1000 || pidGains.Kp < 0) {
        if (DEBUG) { Serial.println(F(" Invalid settings, using defaults.")); }
        pidGains.Kp = 1.0; pidGains.Ki = 0.0; pidGains.Kd = 0.0;
        isCalibrated = false;
    } else {
        if (DEBUG) { Serial.println(F(" Done.")); }
        isCalibrated = true;
    }
}
