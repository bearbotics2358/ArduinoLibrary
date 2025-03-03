/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include "Adafruit_TinyUSB.h"

#define DEFAULT_I2C_ADDR 0x3A
#define BOARD_2_ADDR 0x3B
#define BOARD_3_ADDR 0x3C

#define NUM_BOARDS 3

#define SWITCH1 18  // PA01
#define SWITCH2 19  // PA02
#define SWITCH3 20  // PA03
#define SWITCH4 2   // PA06
#define PWM1 12     // PC00
#define PWM2 13     // PC01
#define PWM3 0      // PA04
#define PWM4 1      // PA05

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};

// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 2, false);

// Report payload defined in src/class/hid/hid.h
hid_gamepad_report_t gp;

Adafruit_seesaw ss[NUM_BOARDS];
int currentAutoSeq = 0;

// constant for number of buttons possible, uses number of boards
const int NUMBER_OF_BUTTONS = NUM_BOARDS * 4;

// Array to store button states (1 for on, 0 for off)
bool buttonStates[NUMBER_OF_BUTTONS];

// Array to define which buttons are active for lighting
const bool activeButtons[NUMBER_OF_BUTTONS] = {
  true,  true,  true,  false,  // Board 1
  false, true,  false, false,  // Board 2
  true,  true,  true,  false   // Board 3
};

void setup() {
  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  TinyUSB_Device_Init(0);
  #endif

  while(!usb_hid.begin()){
    delay(5000);
    usb_hid.begin();
  }

  while (!TinyUSBDevice.mounted()) delay(1);

  if (!ss[0].begin(DEFAULT_I2C_ADDR)) {
    while(1) delay(10);
  }

  if (!ss[1].begin(BOARD_2_ADDR)) {
    while(1) delay(10);
  }

  if (!ss[2].begin(BOARD_3_ADDR)) {
    while(1) delay(10);
  }

  delay(2000);

  uint16_t pid;
  uint8_t year, mon, day;

  for(int i = 0; i < NUM_BOARDS; i++){
    ss[i].getProdDatecode(&pid, &year, &mon, &day);
    if (pid != 5296) {
      while (1) delay(10);
    }
    ss[i].pinMode(SWITCH1, INPUT_PULLUP);
    ss[i].pinMode(SWITCH2, INPUT_PULLUP);
    ss[i].pinMode(SWITCH3, INPUT_PULLUP);
    ss[i].pinMode(SWITCH4, INPUT_PULLUP);
    ss[i].analogWrite(PWM1, 0);
    ss[i].analogWrite(PWM2, 0);
    ss[i].analogWrite(PWM3, 0);
    ss[i].analogWrite(PWM4, 0);
    usb_hid.ready();
  }

  // Initialize button states to all off
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    buttonStates[i] = false;
  }
}

const uint8_t BUTTON_BRIGHTNESS = 255;

// Function to update button states, only affecting active buttons
void updateButtonStates(int buttonIndex) {
  if (activeButtons[buttonIndex]) {
    for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
      if (activeButtons[i]) {
        buttonStates[i] = (i == buttonIndex);
      }
    }
  }
}

// Function to update LEDs based on buttonStates array and activeButtons
void updateLEDs() {
  for (int i = 0; i < NUM_BOARDS; i++) {
    int boardBaseIndex = i * 4;
    if (boardBaseIndex + 3 < NUMBER_OF_BUTTONS) {
      ss[i].analogWrite(PWM1, (activeButtons[boardBaseIndex] && buttonStates[boardBaseIndex]) ? BUTTON_BRIGHTNESS : 0);
      ss[i].analogWrite(PWM2, (activeButtons[boardBaseIndex + 1] && buttonStates[boardBaseIndex + 1]) ? BUTTON_BRIGHTNESS : 0);
      ss[i].analogWrite(PWM3, (activeButtons[boardBaseIndex + 2] && buttonStates[boardBaseIndex + 2]) ? BUTTON_BRIGHTNESS : 0);
      ss[i].analogWrite(PWM4, (activeButtons[boardBaseIndex + 3] && buttonStates[boardBaseIndex + 3]) ? BUTTON_BRIGHTNESS : 0);
    }
  }
}

void loop() {
  // Reset buttons
  gp.x = 0;
  gp.y = 0;
  gp.z = 0;
  gp.rz = 0;
  gp.rx = 0;
  gp.ry = 0;
  gp.hat = 0;
  gp.buttons = 0;

  for (int i = 0; i < NUM_BOARDS; i++) {
    int boardBaseIndex = i * 4;

    if (!ss[i].digitalRead(SWITCH1)) {
      updateButtonStates(boardBaseIndex);
      gp.buttons |= (1U << boardBaseIndex);
    }

    if (!ss[i].digitalRead(SWITCH2)) {
      updateButtonStates(boardBaseIndex + 1);
      gp.buttons |= (1U << (boardBaseIndex + 1));
    }

    if (!ss[i].digitalRead(SWITCH3)) {
      updateButtonStates(boardBaseIndex + 2);
      gp.buttons |= (1U << (boardBaseIndex + 2));
    }

    if (!ss[i].digitalRead(SWITCH4)) {
      updateButtonStates(boardBaseIndex + 3);
      gp.buttons |= (1U << (boardBaseIndex + 3));
    }
  }

  updateLEDs(); // Update LEDs based on the buttonStates array and activeButtons
  usb_hid.sendReport(0, &gp, sizeof(gp));
}
