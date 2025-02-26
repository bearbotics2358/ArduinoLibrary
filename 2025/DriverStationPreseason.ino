/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include "Adafruit_TinyUSB.h"

#define  DEFAULT_I2C_ADDR 0x3A
#define BOARD_2_ADDR 0x3B
#define BOARD_3_ADDR 0x3C
#define BOARD_4_ADDR 0x3E
#define BOARD_5_ADDR 0x3F
#define BOARD_6_ADDR 0x40
#define BOARD_7_ADDR 0x41

#define NUM_BOARDS 7

#define  SWITCH1  18  // PA01
#define  SWITCH2  19 // PA02
#define  SWITCH3  20 // PA03
#define  SWITCH4  2 // PA06
#define  PWM1  12  // PC00
#define  PWM2  13 // PC01
#define  PWM3  0 // PA04
#define  PWM4  1 // PA05

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};

// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 2, false);

// Report payload defined in src/class/hid/hid.h
// - For Gamepad Button Bit Mask see  hid_gamepad_button_bm_t
// - For Gamepad Hat    Bit Mask see  hid_gamepad_hat_t
hid_gamepad_report_t gp;

Adafruit_seesaw ss[NUM_BOARDS];
int currentAutoSeq = 0;

// constant for number of buttons or switches
const int NUMBER_OF_BUTTONS = 28;

// Array to store button states (1 for on, 0 for off)
bool buttonStates[NUMBER_OF_BUTTONS]; // Adjust size based on total number of buttons

void setup() {
  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif

  //Serial.begin(115200);
  // Notes: following commented-out functions has no affect on ESP32
  // usb_hid.setPollInterval(2);
  // usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));

  while(!usb_hid.begin()){
    //Serial.println("HID FAILED :(");
    delay(5000);
    usb_hid.begin();
  }

  // wait until device mounted
  while (!TinyUSBDevice.mounted()) delay(1);

  //pinMode(PIN_EXTERNAL_POWER, OUTPUT);
  // digitalWrite(PIN_EXTERNAL_POWER, HIGH);

  //while (!Serial) delay(10);   // wait until Serial port is opened

  //Serial.println(F("Adafruit PID 5296 I2C QT 4x LED Arcade Buttons test!"));

  if (!ss[0].begin(DEFAULT_I2C_ADDR)) {
    //Serial.println(F("seesaw not found!"));
    while(1) delay(10);
  }

  if (!ss[1].begin(BOARD_2_ADDR)) {
    //Serial.println(F("seesaw 2 not found!"));
    while(1) delay(10);
  }

  if (!ss[2].begin(BOARD_3_ADDR)) {
    //Serial.println(F("seesaw 3 not found!"));
    while(1) delay(10);
  }

  if (!ss[3].begin(BOARD_4_ADDR)) {
    //Serial.println(F("seesaw 4 not found!"));
    while(1) delay(10);
  }

  if (!ss[4].begin(BOARD_5_ADDR)) {
    //Serial.println(F("seesaw 5 not found!"));
    while(1) delay(10);
  }

  if (!ss[5].begin(BOARD_6_ADDR)) {
    //Serial.println(F("seesaw 6 not found!"));
    while(1) delay(10);
  }

  if (!ss[6].begin(BOARD_7_ADDR)) {
    //Serial.println(F("seesaw 7 not found!"));
    while(1) delay(10);
  }

  //Serial.println("Adafruit TinyUSB HID Gamepad example");
  delay(2000);

  uint16_t pid;
  uint8_t year, mon, day;

  for(int i = 0; i<NUM_BOARDS; i++){
    ss[i].getProdDatecode(&pid, &year, &mon, &day);
    //Serial.print("seesaw found PID: ");
    //Serial.print(pid);
    //Serial.print(" datecode: ");
    //Serial.print(2000+year); // Serial.print("/");
    //Serial.print(mon); // Serial.print("/");
    //Serial.println(day);

    if (pid != 5296) {
      //Serial.println(F("Wrong seesaw PID"));
      while (1) delay(10);
    }

    //Serial.println(F("seesaw started OK!"));
    ss[i].pinMode(SWITCH1, INPUT_PULLUP);
    ss[i].pinMode(SWITCH2, INPUT_PULLUP);
    ss[i].pinMode(SWITCH3, INPUT_PULLUP);
    ss[i].pinMode(SWITCH4, INPUT_PULLUP);
    ss[i].analogWrite(PWM1, 0); // Initialize LEDs off
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

// Function to update button states, ensuring only one is lit at a time
void updateButtonStates(int buttonIndex) {
  // Turn all buttons off
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    buttonStates[i] = false;
  }
  // Turn the selected button on
  buttonStates[buttonIndex] = true;
}

// Function to update LEDs based on buttonStates array
void updateLEDs() {
  for (int i = 0; i < NUM_BOARDS; i++) {
    // Calculate the base index for the current board
    int boardBaseIndex = i * 4;

    if (buttonStates[boardBaseIndex + 0]) {
      ss[i].analogWrite(PWM1, BUTTON_BRIGHTNESS);
    } else {
      ss[i].analogWrite(PWM1, 0);
    }

    if (buttonStates[boardBaseIndex + 1]) {
      ss[i].analogWrite(PWM2, BUTTON_BRIGHTNESS);
    } else {
      ss[i].analogWrite(PWM2, 0);
    }

    if (buttonStates[boardBaseIndex + 2]) {
      ss[i].analogWrite(PWM3, BUTTON_BRIGHTNESS);
    } else {
      ss[i].analogWrite(PWM3, 0);
    }

    if (buttonStates[boardBaseIndex + 3]) {
      ss[i].analogWrite(PWM4, BUTTON_BRIGHTNESS);
    } else {
      ss[i].analogWrite(PWM4, 0);
    }
  }
}

void loop() {
  //while (!usb_hid.ready()) {
  //    Serial.println("HID not ready");
  //    delay(2000);
  //  }
  //  if (!usb_hid.ready()) return;

  // Reset buttons
  //Serial.println("No pressing buttons");
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
      updateButtonStates(boardBaseIndex + 0);
      gp.buttons |= (1U << (boardBaseIndex + 0));
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

  updateLEDs(); // Update LEDs based on the buttonStates array
  usb_hid.sendReport(0, &gp, sizeof(gp)); //delay(10);
  // delay(10);
}
