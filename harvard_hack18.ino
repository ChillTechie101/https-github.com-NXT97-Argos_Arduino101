/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 * Sketch: LedControl.ino
 *
 * Description:
 *   This is a Central sketch that looks for a particular Sevice with a
 *   certain Characteristic from a Peripheral.  Upon succesful discovery,
 *   it reads the state of a button and write that value to the
 *   Peripheral Characteristic.
 *
 * Notes:
 *
 *  - Expected Peripheral Service: 19b10000-e8f2-537e-4f6c-d104768a1214
 *  - Expected Peripheral Characteristic: 19b10001-e8f2-537e-4f6c-d104768a1214
 *  - Expected Peripheral sketch:
 *
 */

#include <CurieBLE.h>

// variables for button
const int buttonPin = 2;
int a=50;
int b;
int dis,pwm;
int rss;
int k=0;
int oldButtonState = LOW;
//void nav();

void setup() {
  Serial.begin(9600);
pinMode(9,OUTPUT);
pinMode(6,OUTPUT);
digitalWrite(9,LOW);
digitalWrite(6,LOW);

  // configure the button pin as input
  pinMode(buttonPin, INPUT);

  // initialize the BLE hardware
  BLE.begin();

  //Serial.println("BLE Central - LED control");

  // start scanning for peripherals
  //BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  //delay(100);
}

void loop() {
  delay(50);
//  analogWrite(9,150);
//analogWrite(6,150);
  if(Serial.available())
{
  a=Serial.read();
  //Serial.println(a);
}
if(a==51)
{
//  if(k==0)
//  {
  BLE.scan();
  delay(50);
  dis=50;
  //k=1;
  //}//Serial.println("Navigation");
  while(dis>45)
  {
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral
     if (peripheral.address()=="98:4F:EE:0F:34:51") {
   // Serial.println("Discovered a peripheral");
   // Serial.println("-----------------------");

    // print address
//    Serial.print("Address: ");
  //  Serial.println(peripheral.address());

    // print the local name, if present
  //  if (peripheral.hasLocalName()) {
    //  Serial.print("Local Name: ");
     // Serial.println(peripheral.localName());
   // }

    // print the advertised service UUID's, if present
    /*if (peripheral.hasAdvertisedServiceUuid()) {
      Serial.print("Service UUID's: ");
      for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
        Serial.print(peripheral.advertisedServiceUuid(i));
        Serial.print(" ");
      }
      Serial.println();
    }*/

    // print the RSSI
   // Serial.print("RSSI: ");
   // Serial.println(peripheral.rssi());
   // 
    dis=abs(peripheral.rssi());
    pwm=map(dis,44,104,205,44);
   // Serial.println(pwm);
    if(Serial.available())
{
  b=Serial.read();
  //Serial.println(a);
}
if(b==49)
{
  analogWrite(9,pwm);
  analogWrite(6,0);
}
else if(b==50)
{
  analogWrite(9,0);
  analogWrite(6,pwm);
}
else
{
  analogWrite(9,pwm);
     analogWrite(6,pwm);
}
     }
//    if(dis<45)
//    {
    }
  }
      Serial.println("r");
      analogWrite(9,0);
     analogWrite(6,0);
      BLE.stopScan();
      a=0;
    }
else if(a==49||a==50)//49,50 = light
{
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  delay(100);
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
   //if (peripheral.address()=="98:4F:EE:0F:34:51") {
    // discovered a peripheral, print out address, local name, and advertised service
  /*  Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();*/
    BLE.stopScan();
  controlLed(peripheral);
//}
}
}
}

void controlLed(BLEDevice peripheral) {
  // connect to the peripheral
//  Serial.println("Connecting ...");

  if (peripheral.connect()) {
  //  Serial.println("Connected");
  } else {
   // Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
 // Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
  //  Serial.println("Attributes discovered");
  } else {
   // Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!ledCharacteristic) {
   // Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
   // Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
   /* if(Serial.available()>0)
  {
    a=Serial.read();
  }*/
    // while the peripheral is connection

    // read the button pin
  /*  int buttonState = digitalRead(buttonPin);

    if (oldButtonState != buttonState) {
      // button changed
      oldButtonState = buttonState;
*/

      if (a==49)//1
      {
        //Serial.println("button pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeByte(0x01);
        a=0;
        peripheral.disconnect();
      } 
      else //if(a==50)//2
      {
       // Serial.println("button released");

        // button is released, write 0x00 to turn the LED of
        ledCharacteristic.writeByte(0x00);
        a=0;
        peripheral.disconnect();
      }
      
//    else if(a==51)//3
//    {
//      peripheral.disconnect();
//    }
  }
 // Serial.println("Peripheral disconnected");
}
