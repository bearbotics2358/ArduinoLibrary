/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include "Adafruit_TinyUSB.h"

#define  DEFAULT_I2C_ADDR 0x3A




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


Adafruit_seesaw ss;

void setup() {
  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif

  Serial.begin(115200);
  
  //while (!Serial) delay(10);   // wait until serial port is opened

  Serial.println(F("Adafruit PID 5296 I2C QT 4x LED Arcade Buttons test!"));
  
  if (!ss.begin(DEFAULT_I2C_ADDR)) {
    Serial.println(F("seesaw not found!"));
    while(1) delay(10);
  }

  


  // Notes: following commented-out functions has no affect on ESP32
  usb_hid.setPollInterval(2);
   usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));

  usb_hid.begin();

  // wait until device mounted
  while (!TinyUSBDevice.mounted()) delay(1);

  Serial.println("Adafruit TinyUSB HID Gamepad example");
  delay(2000);

  uint16_t pid;
  uint8_t year, mon, day;
  
  ss.getProdDatecode(&pid, &year, &mon, &day);
  Serial.print("seesaw found PID: ");
  Serial.print(pid);
  Serial.print(" datecode: ");
  Serial.print(2000+year); Serial.print("/"); 
  Serial.print(mon); Serial.print("/"); 
  Serial.println(day);

  if (pid != 5296) {
    Serial.println(F("Wrong seesaw PID"));
    while (1) delay(10);
  }

  Serial.println(F("seesaw started OK!"));
  ss.pinMode(SWITCH1, INPUT_PULLUP);
  ss.pinMode(SWITCH2, INPUT_PULLUP);
  ss.pinMode(SWITCH3, INPUT_PULLUP);
  ss.pinMode(SWITCH4, INPUT_PULLUP);
  ss.analogWrite(PWM1, 127);
  ss.analogWrite(PWM2, 127);
  ss.analogWrite(PWM3, 127);
  ss.analogWrite(PWM4, 127);
}

uint8_t incr = 0;

void loop() {

   /* if (usb_hid.ready()) {
      Serial.println("HID ready");
      return;
    }*/
    if (!usb_hid.ready()) return;

  // Reset buttons
 // Serial.println("No pressing buttons");
  gp.x = 0;
  gp.y = 0;
  gp.z = 0;
  gp.rz = 0;
  gp.rx = 0;
  gp.ry = 0;
  gp.hat = 0;
  gp.buttons = 0;



 if (! ss.digitalRead(SWITCH1)) {
    Serial.println("Switch 1 Flipped");
    ss.analogWrite(SWITCH1, 1);
    incr += 5;
     gp.buttons |= (1U << 0);
  } else {
    ss.analogWrite(PWM1, 0);
  }
  
  if (! ss.digitalRead(SWITCH2)) {
    Serial.println("Switch 2 pressed");
    ss.analogWrite(PWM2, incr);
    incr += 5;
     gp.buttons |= (1U << 1);
  } else {
    ss.analogWrite(PWM2, 0);
  }
  
  if (! ss.digitalRead(SWITCH3)) {
    Serial.println("Switch 3 pressed");
    ss.analogWrite(PWM3, incr);
    incr += 5;
     gp.buttons |= (1U << 2);
  } else {
    ss.analogWrite(PWM3, 0);
  }
  
  if (! ss.digitalRead(SWITCH4)) {
    Serial.println("Switch 4 pressed");
    ss.analogWrite(PWM4, incr);
    incr += 5;
    gp.buttons |= (1U << 3);
  } else {
    ss.analogWrite(PWM4, 0);
  }

  usb_hid.sendReport(0, &gp, sizeof(gp)); delay(10);
  delay(10);
}

