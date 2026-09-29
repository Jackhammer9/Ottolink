#define IR_RECEIVE_PIN 2

#include <Arduino.h>
#include <SPI.h>
#include <IRremote.hpp>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include "kinematics.h"

#define PITCH_PIN 3
#define YAW_PIN   5
#define ROLL_PIN  6
#define ELBOW_PIN 9

#define TFT_CS  10
#define TFT_DC  7
#define TFT_RST -1

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

const uint16_t FG = ST77XX_WHITE;
const uint16_t BG = 0x780F;

Servo pitchServo;
Servo yawServo;
Servo rollServo;
Servo elbowServo;

Kinematics kin;

Matrix<1, 4> q = {0.0f, 0.0f, 0.0f, 0.0f};
Matrix<1, 3> target;
Matrix<1, 3> current;

uint8_t lastCommand = 0;
uint16_t receivedCount = 0;

int pitchAngle = 90;
int yawAngle   = 90;
int rollAngle  = 90;
int elbowAngle = 90;

const float POSITION_STEP = 0.8f;
const float Q4_STEP_RAD = 12 * PI / 180.0f;
const float Q4_MAX_RAD = PI / 2.0f;

void drawUI()
{
    tft.fillScreen(BG);
    tft.setTextColor(FG, BG);
    tft.setTextSize(1);

    tft.setTextSize(2);
    tft.setCursor(15, 3);
    tft.print(F("OttoLink"));
    tft.setTextSize(1);

    tft.drawFastHLine(4, 23, 120, FG);

    tft.setCursor(4, 28);
    tft.print(F("JOINT ANGLES"));

    tft.setCursor(4, 41);  tft.print(F("P:"));
    tft.setCursor(4, 53);  tft.print(F("Y:"));
    tft.setCursor(4, 65);  tft.print(F("R:"));
    tft.setCursor(4, 77);  tft.print(F("E:"));

    tft.drawFastHLine(4, 89, 120, FG);

    tft.setCursor(4, 95);  tft.print(F("IR:"));
    tft.setCursor(4, 108); tft.print(F("T:"));
    tft.setCursor(4, 120); tft.print(F("C:"));
    tft.setCursor(4, 132); tft.print(F("ERR:"));
    tft.setCursor(4, 144); tft.print(F("EVENT:"));
}

void writeServos()
{
    pitchServo.write(pitchAngle);
    yawServo.write(yawAngle);
    rollServo.write(rollAngle);
    elbowServo.write(elbowAngle);
}

void updateAngleDisplay()
{
    tft.fillRect(18, 41, 70, 48, BG);

    tft.setCursor(18, 41);
    tft.print(pitchAngle);
    tft.print((char)247);

    tft.setCursor(18, 53);
    tft.print(yawAngle);
    tft.print((char)247);

    tft.setCursor(18, 65);
    tft.print(rollAngle);
    tft.print((char)247);

    tft.setCursor(18, 77);
    tft.print(elbowAngle);
    tft.print((char)247);
}

void updateIRDisplay(uint8_t command, const char* name)
{
    tft.fillRect(22, 95, 104, 10, BG);
    tft.setCursor(22, 95);

    tft.print(F("0x"));
    if (command < 0x10) {
        tft.print('0');
    }
    tft.print(command, HEX);
    tft.print(' ');
    tft.print(name);
}

void updatePositionDisplay()
{
    tft.fillRect(20, 108, 106, 10, BG);
    tft.setCursor(20, 108);
    tft.print(target(0, 0), 2);
    tft.print(',');
    tft.print(target(0, 1), 2);
    tft.print(',');
    tft.print(target(0, 2), 2);

    tft.fillRect(20, 120, 106, 10, BG);
    tft.setCursor(20, 120);
    tft.print(current(0, 0), 2);
    tft.print(',');
    tft.print(current(0, 1), 2);
    tft.print(',');
    tft.print(current(0, 2), 2);

    const float dx = target(0, 0) - current(0, 0);
    const float dy = target(0, 1) - current(0, 1);
    const float dz = target(0, 2) - current(0, 2);
    const float errorDistance = sqrtf(dx * dx + dy * dy + dz * dz);

    tft.fillRect(30, 132, 96, 10, BG);
    tft.setCursor(30, 132);
    tft.print(errorDistance, 3);
}

void updateEventDisplay(const char* eventName, bool repeat)
{
    tft.fillRect(40, 144, 86, 10, BG);
    tft.setCursor(40, 144);
    tft.print(eventName);
    tft.print(F(" #"));
    tft.print(receivedCount);

    if (repeat) {
        tft.print(F(" R"));
    }
}

void updateAnglesFromQ()
{
    pitchAngle = constrain(
        90 + (int)(q(0, 0) * 180.0f / PI), 0, 180);

    yawAngle = constrain(
        90 + (int)(q(0, 1) * 180.0f / PI), 0, 180);

    rollAngle = constrain(
        90 + (int)(q(0, 2) * 180.0f / PI), 0, 180);

    elbowAngle = constrain(
        90 - (int)(q(0, 3) * 180.0f / PI), 0, 90);
}

void solveIK()
{
    q = kin.inverseStep(q, target);

    if (q(0, 3) < 0.0f) {
        q(0, 3) = 0.0f;
    } else if (q(0, 3) > Q4_MAX_RAD) {
        q(0, 3) = Q4_MAX_RAD;
    }

    current = kin.forward(q);

    updateAnglesFromQ();
    writeServos();
}

void processCommand(uint8_t command, bool repeat)
{
    const char* commandName = "UNKNOWN";
    const char* eventName = "UNKNOWN";
    bool isMovementCommand = true;
    bool runIK = true;

    switch (command) {
        case 0x46: commandName = "+X"; eventName = "+X"; break;
        case 0x15: commandName = "-X"; eventName = "-X"; break;
        case 0x43: commandName = "+Y"; eventName = "+Y"; break;
        case 0x44: commandName = "-Y"; eventName = "-Y"; break;
        case 0x09: commandName = "Z+"; eventName = "Z+"; break;
        case 0x07: commandName = "Z-"; eventName = "Z-"; break;
        default:   isMovementCommand = false; break;
    }

    updateIRDisplay(command, commandName);
    updateEventDisplay(eventName, repeat);

    if (!isMovementCommand) {
        return;
    }

    if (runIK) {
        // Discard leftover target distance from the previous jog direction.
        target = current;

        switch (command) {
            case 0x46: target(0, 0) += POSITION_STEP; break;
            case 0x15: target(0, 0) -= POSITION_STEP; break;
            case 0x43: target(0, 1) += POSITION_STEP; break;
            case 0x44: target(0, 1) -= POSITION_STEP; break;
        }

        solveIK();
    } else {
        if (command == 0x09) {
            q(0, 3) += Q4_STEP_RAD;
            if (q(0, 3) > Q4_MAX_RAD) {
                q(0, 3) = Q4_MAX_RAD;
            }
        } else {
            q(0, 3) -= Q4_STEP_RAD;
            if (q(0, 3) < 0.0f) {
                q(0, 3) = 0.0f;
            }
        }

        current = kin.forward(q);
        updateAnglesFromQ();
        writeServos();
    }

    updateAngleDisplay();
    updatePositionDisplay();
}

void setup()
{
    pitchServo.attach(PITCH_PIN);
    yawServo.attach(YAW_PIN);
    rollServo.attach(ROLL_PIN);
    elbowServo.attach(ELBOW_PIN);

    pitchServo.write(90);
    yawServo.write(90);
    rollServo.write(90);
    elbowServo.write(90);

    pinMode(TFT_CS, OUTPUT);
    digitalWrite(TFT_CS, HIGH);

    tft.initR(INITR_BLACKTAB);
    tft.setSPISpeed(1000000);
    tft.setRotation(2);

    target = kin.forward(q);
    current = target;

    drawUI();
    updateAngleDisplay();
    updatePositionDisplay();
    updateIRDisplay(0, "READY");
    updateEventDisplay("WAITING", false);

    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
}

void loop()
{
    if (!IrReceiver.decode()) {
        return;
    }

    const uint8_t receivedCommand = IrReceiver.decodedIRData.command;
    const bool isRepeat =
        (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) != 0;

    ++receivedCount;

    uint8_t commandToProcess = receivedCommand;

    if (isRepeat && lastCommand != 0) {
        commandToProcess = lastCommand;
    } else if (!isRepeat) {
        lastCommand = receivedCommand;
    }

    IrReceiver.resume();

    processCommand(commandToProcess, isRepeat);
}