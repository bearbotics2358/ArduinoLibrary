/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
//#include "Adafruit_TinyUSB.h"

#define DEFAULT_I2C_ADDR 0x3A
#define BOARD_2_ADDR 0x3B
#define BOARD_3_ADDR 0x3C
#define NUM_BOARDS 3



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
/*uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};
*/
// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
//Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 2, false);

// Report payload defined in src/class/hid/hid.h
// - For Gamepad Button Bit Mask see  hid_gamepad_button_bm_t
// - For Gamepad Hat    Bit Mask see  hid_gamepad_hat_t
//hid_gamepad_report_t gp;


Adafruit_seesaw ss[NUM_BOARDS];

void setup() {
 /* #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  //TinyUSB_Device_Init(0);
#endif

  // Notes: following commented-out functions has no affect on ESP32
  usb_hid.setPollInterval(2);
   usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));

  usb_hid.begin();

*/
Serial.begin(115200);

  // wait until device mounted
 // while (!TinyUSBDevice.mounted()) delay(1);

  
  
  //while (!Serial) delay(10);   // wait until serial port is opened

  delay(5000);

  Serial.println(F("Adafruit PID 5296 I2C QT 4x LED Arcade Buttons test!"));

  

 // delay(2000);
   Serial.println("Adafruit TinyUSB HID Gamepad example");

  uint16_t pid;
  uint8_t year, mon, day;
    Serial.println("Adafruit TinyUSB HID Gamepad example");

  if (!ss[0].begin(DEFAULT_I2C_ADDR)) {
    Serial.println(F("seesaw not found!"));
    while(1) {
      delay(10);
    }
  }
  Serial.println(F("seesaw 1!"));

  if (!ss[1].begin(BOARD_2_ADDR)) {
    Serial.println(F("seesaw 2 not found!"));
    while(1) delay(10);
  }
  Serial.println(F("seesaw 2!"));

 
if (!ss[2].begin(BOARD_3_ADDR)) {
    Serial.println(F("seesaw 3 not found!"));
    while(1) delay(10);
  }
  Serial.println(F("seesaw 3!"));
  


  
 

for(int i = 0; i<NUM_BOARDS-1; i++){

 ss[i].getProdDatecode(&pid, &year, &mon, &day);
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
  ss[i].pinMode(SWITCH1, INPUT_PULLUP);
  ss[i].pinMode(SWITCH2, INPUT_PULLUP);
  ss[i].pinMode(SWITCH3, INPUT_PULLUP);
  ss[i].pinMode(SWITCH4, INPUT_PULLUP);
  ss[i].analogWrite(PWM1, 127);
  ss[i].analogWrite(PWM2, 127);
  ss[i].analogWrite(PWM3, 127);
  ss[i].analogWrite(PWM4, 127);
}
}

uint8_t incr = 0;

void loop() {

   /* if (usb_hid.ready()) {
      Serial.println("HID ready");
      return;
    }*/
  //  if (!usb_hid.ready()) return;

  // Reset buttons
 // Serial.println("No pressing buttons");
 /* gp.x = 0;
  gp.y = 0;
  gp.z = 0;
  gp.rz = 0;
  gp.rx = 0;
  gp.ry = 0;
  gp.hat = 0;
  gp.buttons = 0;
*/


 if (! ss[0].digitalRead(SWITCH1)) {
    Serial.println("Switch 1 Flipped");
    ss[0].analogWrite(SWITCH1, 1);
    incr += 5;
     //gp.buttons |= (1U << 0);
  } else {
    ss[0].analogWrite(PWM1, 0);
  }
  
  if (! ss[0].digitalRead(SWITCH2)) {
    Serial.println("Switch 2 pressed");
    ss[0].analogWrite(PWM2, incr);
    incr += 5;
    // gp.buttons |= (1U << 1);
  } else {
    ss[0].analogWrite(PWM2, 0);
  }
  
  if (! ss[0].digitalRead(SWITCH3)) {
    Serial.println("Switch 3 pressed");
    ss[0].analogWrite(PWM3, incr);
    incr += 5;
     //gp.buttons |= (1U << 2);
  } else {
    ss[0].analogWrite(PWM3, 0);
  }
  
  if (! ss[0].digitalRead(SWITCH4)) {
    Serial.println("Switch 4 pressed");
    ss[0].analogWrite(PWM4, incr);
    incr += 5;
  //  gp.buttons |= (1U << 3);
  } else {
    ss[0].analogWrite(PWM4, 0);
  }








if (! ss[1].digitalRead(SWITCH1)) {
    Serial.println("Switch 1 Flipped");
    ss[1].analogWrite(SWITCH1, 1);
    incr += 5;
    // gp.buttons |= (1U << 4);
  } else {
    ss[1].analogWrite(PWM1, 0);
  }
  
  if (! ss[1].digitalRead(SWITCH2)) {
    Serial.println("Switch 2 pressed");
    ss[1].analogWrite(PWM2, incr);
    incr += 5;
   //  gp.buttons |= (1U << 5);
  } else {
    ss[1].analogWrite(PWM2, 0);
  }
  
  if (! ss[1].digitalRead(SWITCH3)) {
    Serial.println("Switch 3 pressed");
    ss[1].analogWrite(PWM3, incr);
    incr += 5;
    // gp.buttons |= (1U << 6);
  } else {
    ss[1].analogWrite(PWM3, 0);
  }
  
  if (! ss[1].digitalRead(SWITCH4)) {
    Serial.println("Switch 4 pressed");
    ss[1].analogWrite(PWM4, incr);
    incr += 5;
   // gp.buttons |= (1U << 7);
  } else {
    ss[1].analogWrite(PWM4, 0);
  }


/*

  if (! ss[2].digitalRead(SWITCH1)) {
    Serial.println("Switch 1 Flipped");
    ss[2].analogWrite(SWITCH1, 1);
    incr += 5;
     gp.buttons |= (1U << 8);
  } else {
    ss[2].analogWrite(PWM1, 0);
  }

  if (! ss[2].digitalRead(SWITCH2)) {
    Serial.println("Switch 2 pressed");
    ss[2].analogWrite(PWM2, incr);
    incr += 5;
     gp.buttons |= (1U << 9);
  } else {
    ss[2].analogWrite(PWM2, 0);
  }

  if (! ss[2].digitalRead(SWITCH3)) {
    Serial.println("Switch 3 pressed");
    ss[2].analogWrite(PWM3, incr);
    incr += 5;
     gp.buttons |= (1U << 10);
  } else {
    ss[2].analogWrite(PWM3, 0);
  }

  if (! ss[2].digitalRead(SWITCH4)) {
    Serial.println("Switch 4 pressed");
    ss[2].analogWrite(PWM4, incr);
    incr += 5;
    gp.buttons |= (1U << 11);
  } else {
    ss[2].analogWrite(PWM4, 0);
  }
*/

//  usb_hid.sendReport(0, &gp, sizeof(gp)); delay(10);
  delay(10);
}