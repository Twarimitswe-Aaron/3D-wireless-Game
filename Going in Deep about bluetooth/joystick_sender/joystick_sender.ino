#include <SoftwareSerial.h>

#define BT_RX 10  // Arduino receives from HC-05 TXD
#define BT_TX 11  // Arduino sends to HC-05 RXD

SoftwareSerial bt(BT_RX, BT_TX); // RX, TX


#define VRX A0
#define VRY A1

void setup() {

  Serial.begin(38400);  
 
  bt.begin(38400);     
  Serial.println("Bluetooth joystick bridge ready");
}

void loop() {
  
  int xValue = analogRead(VRX); // 0-1023
  int yValue = analogRead(VRY); // 0-1023

 
  // Serial.print("X: ");
  // Serial.print(xValue);
  // Serial.print(" | Y: ");
  // Serial.println(yValue);

 
  bt.print("X:");
  bt.print(xValue);
  bt.print(",Y:");
  bt.println(yValue);


  delay(100); 
}
