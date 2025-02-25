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

// Array to store button states (1 for on, 0 for off)
bool buttonStates[28]; // Adjust size based on total number of buttons

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
  for (int i = 0; i < 28; i++) {
    buttonStates[i] = false;
  }
}

uint8_t incr = 255;

// Function to update button states, ensuring only one is lit at a time
void updateButtonStates(int buttonIndex) {
  // Turn all buttons off
  for (int i = 0; i < 28; i++) {
    buttonStates[i] = false;
  }
  // Turn the selected button on
  buttonStates[buttonIndex] = true;
}

// Function to update LEDs based on buttonStates array
void updateLEDs() {
  int buttonIndex = 0;
  for(int i = 0; i < NUM_BOARDS; i++) {
    if (buttonStates[buttonIndex++]) {
      ss[i].analogWrite(PWM1, incr);
    } else {
      ss[i].analogWrite(PWM1, 0);
    }

    if (buttonStates[buttonIndex++]) {
      ss[i].analogWrite(PWM2, incr);
    } else {
      ss[i].analogWrite(PWM2, 0);
    }

    if (buttonStates[buttonIndex++]) {
      ss[i].analogWrite(PWM3, incr);
    } else {
      ss[i].analogWrite(PWM3, 0);
    }

    if (buttonStates[buttonIndex++]) {
      ss[i].analogWrite(PWM4, incr);
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

  int buttonIndex = 0;

  if (! ss[0].digitalRead(SWITCH1)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 0);
  }
  buttonIndex++;

  if (! ss[0].digitalRead(SWITCH2)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 1);
  }
  buttonIndex++;

  if (! ss[0].digitalRead(SWITCH3)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 2);
  }
  buttonIndex++;

  if (! ss[0].digitalRead(SWITCH4)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 3);
  }
  buttonIndex++;

  if (! ss[1].digitalRead(SWITCH1)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 4);
  }
  buttonIndex++;

  if (! ss[1].digitalRead(SWITCH2)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 5);
  }
  buttonIndex++;

  if (! ss[1].digitalRead(SWITCH3)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 6);
  }
  buttonIndex++;

  if (! ss[1].digitalRead(SWITCH4)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 7);
  }
  buttonIndex++;

  if (! ss[2].digitalRead(SWITCH1)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 8);
  }
  buttonIndex++;

  if (! ss[2].digitalRead(SWITCH2)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 9);
  }
  buttonIndex++;

  if (! ss[2].digitalRead(SWITCH3)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 10);
  }
  buttonIndex++;

  if (! ss[2].digitalRead(SWITCH4)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 11);
  }
  buttonIndex++;

  if (! ss[3].digitalRead(SWITCH1)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 12);
  }
  buttonIndex++;

  if (! ss[3].digitalRead(SWITCH2)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 13);
  }
  buttonIndex++;

  if (! ss[3].digitalRead(SWITCH3)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 14);
  }
  buttonIndex++;

  if (! ss[3].digitalRead(SWITCH4)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 15);
  }
  buttonIndex++;

  if (! ss[4].digitalRead(SWITCH1)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 16);
  }
  buttonIndex++;

  if (! ss[4].digitalRead(SWITCH2)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 17);
  }
  buttonIndex++;

  if (! ss[4].digitalRead(SWITCH3)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 18);
  }
  buttonIndex++;

  if (! ss[4].digitalRead(SWITCH4)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 19);
  }
  buttonIndex++;

  if (! ss[5].digitalRead(SWITCH1)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 20);
  }
  buttonIndex++;

  if (! ss[5].digitalRead(SWITCH2)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 21);
  }
  buttonIndex++;

  if (! ss[5].digitalRead(SWITCH3)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 22);
  }
  buttonIndex++;

  if (! ss[5].digitalRead(SWITCH4)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 23);
  }
  buttonIndex++;

  if (! ss[6].digitalRead(SWITCH1)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 24);
  }
  buttonIndex++;

  if (! ss[6].digitalRead(SWITCH2)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 25);
  }
  buttonIndex++;

  if (! ss[6].digitalRead(SWITCH3)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 26);
  }
  buttonIndex++;

  if (! ss[6].digitalRead(SWITCH4)) {
    updateButtonStates(buttonIndex);
    gp.buttons |= (1U << 27);
  }
  buttonIndex++;

  updateLEDs(); // Update LEDs based on the buttonStates array
  usb_hid.sendReport(0, &gp, sizeof(gp)); //delay(10);
  // delay(10);
}
