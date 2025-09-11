# Sensors

This Folder contains all the sensors that are utilized with the Seed Grove Kit

# Sensor Setup

Use the resources under each sensor type to learn more about how the sensor works, what libraries are needed, and how to connect it to your arduino.
For the test codes provided for each sensor, open a blank Arduino IDE tab to copy and paste to code into.

***
### **Button/Touch Sensor**
* [Button Part Link](https://wiki.seeedstudio.com/Grove-Button/)
* [Touch Sensor Part Link](https://wiki.seeedstudio.com/Grove-Touch_Sensor/)
* #### Libraries
   * No additional libraries needed for these sensors.
* #### Documents
   * [Button Manual](https://robu.in/wp-content/uploads/2019/09/Grove-Button-User-Manual.pdf)
   * [Touch Sensor Manual](https://robu.in/wp-content/uploads/2019/09/Grove-Touch-Sensor.pdf)
* #### Hookup
   * Using one of the digital ports (D7) on the Base Shield, simply use the provided 4-prong connection wires to directly connect the sensor to the Base Shield, ensuring proper connections are made (GND to GND, VCC to VCC, etc.)
* #### Code
   * [Button/Touch Test Code](Arduino_Code/Part_Testing/ButtonAndTouchSensor_test/ButtonAndTouchSensor_test.ino?ref_type=heads)


***
### **Buzzer**
* [Part Link](https://wiki.seeedstudio.com/Grove-Buzzer/)
* #### Libraries
   * [Servo](https://docs.arduino.cc/libraries/servo/) - Also available in Ardiuno IDE Library Manger
* #### Documents
   * [Manual](https://www.mouser.com/datasheet/2/744/Seeed_107020000-1217511.pdf)
* #### Hookup
   * Using one of the digital ports (D8) on the Base Shield, simply use the provided 4-prong connection wires to directly connect the sensor to the Base Shield, ensuring proper connections are made (GND to GND, VCC to VCC, etc.)
* #### Code
   * [Buzzer Test Code](Arduino_Code/Part_Testing/Buzzer_test/Buzzer_test.ino?ref_type=heads)


***
### **Color Sensor**
* [Part Link](https://www.digikey.ca/en/products/detail/dfrobot/SEN0101/6588457?gclsrc=aw.ds&&utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Product_Low%20ROAS%20Categories&utm_term=&productid=6588457&utm_content=&utm_id=go_cmp-20291741422_adg-_ad-__dev-c_ext-_prd-6588457_sig-Cj0KCQiAs5i8BhDmARIsAGE4xHwkMFMRrdYxqIl3Mv8EGdNE3Zt0SoJy4V8yZpN3muggOEyRdVIUhYoaApFWEALw_wcB&gad_source=1&gclid=Cj0KCQiAs5i8BhDmARIsAGE4xHwkMFMRrdYxqIl3Mv8EGdNE3Zt0SoJy4V8yZpN3muggOEyRdVIUhYoaApFWEALw_wcB&gclsrc=aw.ds)
* #### Libraries
   * [DFRobot_TCS3200](https://github.com/Panjkrc/TCS3200_library) -Also available in Arduino IDE Library Manger
* #### Documents
   * [Hookup Guide](https://github.com/Panjkrc/TCS3200_library/blob/master/wiring_schematics.png)
   * [Datasheet](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2107/SEN0101_Web.pdf)
* #### Hookup 
   * This sensor is very large, so you will likely need to use multiple bread boards to connect it. Feel free to remove one of the positive/negative strips on the side to stick two boards together. You will need to use jumper wires to connect to this sensor.
   * Connect the pin labeled VDD on the chip to the 5v port on the Arduino, and connect the GND on the chip to the GND on the Arduino.
   * The pins labeled s0 and s1 control the output scaling (changing the scaling variable in the code controls these outputs.) Connect s0 to the D0 port on the Arduino, and s1 to D1.
   * The pins labeled s2 and s3 control which colour the chip is measuring (either red, green, blue, or white.) Connect s2 to the D2 port on the Arduino, and S3 to the D3 port.
   * The pin labeled OUT on the chip is what the Arduino uses to get the output from the chip. Connect OUT to port D4 on the Arduino.
   * If you would like to turn the LEDs on to brighten the area you are trying to measure, connect the LED pin on the chip to the 5v port on the Arduino.
* #### Code
   * [Color Sensor Test Code](Arduino_Code/Part_Testing/ColorSensor_test/ColorSensor_test.ino?ref_type=heads)


***
### **LCD Display**
* [Part Link](https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/)
* #### Libraries
   * [Grove LCD RGB Backlight](https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight) - Also available in Arduino IDE Library Manger
* #### Documents
   * [Datasheet](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/1081/104030001_Web.pdf)
* #### Hookup
   * The Grove LCD Display is not compatible with the Arduino R4 and will require pull-up resistors to function correctly. Use a breadboard to set up 4kΩ to 6kΩ  resistors in the configuration shown below.<br>
   ![Pullup Resistors](Images/Pullup_Resitors.png) <br>
   * The LCD Display uses I2C ports, so connect the VCC, GND, SDA, and SCL connection to the corresponding ports on the Base Shield.
* #### Code
   * [LCD Display Test Code](Arduino_Code/Part_Testing/LCD_test/LCD_test.ino?ref_type=heads)


***
### **LEDs** 
* [Part Link](https://www.digikey.ca/en/products/detail/sparkfun-electronics/COM-15206/10064425?s=N4IgTCBcDaIIwFYBsAOAtAYQPIFk2LAAYk0A5AERAF0BfIA)
* #### Libraries
   * [Adafruit_NeoPixel](https://reference.arduino.cc/reference/en/libraries/adafruit-neopixel/) - Also available in Ardiuno IDE Library Manger
* #### Documents
   * [Hookup Guide](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/1179/WS2812_Breakout_Hookup_Guide.pdf)
* #### Hookup
   * Use one of the 4-pin to male wires to connect any of the D ports on the Arduino. The D port you use should be the value of the LED_PIN constant defined in the second line of the LED test code.
   * Connect the black wire from the 4-pin to the black/blue wire on the LED strip (Ground)
   * Connect the red wire from the 4-pin to the red wire on the LED strip (5v)
   * Connect the yellow wire from the 4-pin to the yellow wire on the LED strip (data)
   * The white wire on the 4-pin should not connect to anything.
* #### Code
   * Running the code below with the correct pin setup should turn the LEDs in circuit with the arduino a light purple color. Change the “red”, “green”, and “blue” values and upload code to get different color outputs! 
   * [LED Test Code](Arduino_Code/Part_Testing/LED_test/LED_test.ino?ref_type=heads) <br><br>


***
### **Pulse Oximeter**
   * There are two different Pulse Oximeters: one is green, and the other is purple. They have different part links and datasheets, but everything else, including the hookup and code, is the same.

* [Part Link: Green Sensor](https://www.digikey.ca/en/products/detail/analog-devices-inc-maxim-integrated/maxrefdes117/6165562)

* [Part Link: Purple Sensor](https://www.digikey.ca/en/products/detail/sunfounder/ST0244/22116824?s=N4IgTCBcDaILYEMAeBmADARjWABACwFMEAnAFx2IVIJzgHsATAVwBsCQBdAXyA)
* #### Libraries
   * [DFRobot_MAX30102 Arduino Library ](https://github.com/DFRobot/DFRobot_MAX30102) - Must download from Git. Not available through the IDE.
* #### Documents 
   * [Datasheet: Green Sensor](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/1222/MAXREFDES117_Web.pdf)
   * [Datasheet: Purple Sensor](https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf)
* #### Hookup: SAME FOR BOTH SENSORS!!
   * On the pulse-oximeter sensor, two sides of the sensor there is a single connection point (GND or VIN), and on the other two sides there are three connection points (each with INT, SCL, and SDA).
   * This sensor uses an I2C connection, so we need to use the GND and VIN (VCC), as well as one set of the SDA and SCL pins (from the same side). INT pins are NOT used.
   * Use Breadboard to attach wires from the respected sensor pin to the Base Shield. 
* #### Code: SAME FOR BOTH SENSORS!!
   * [Pulse-Oximeter Test Code](Arduino_Code/Part_Testing/PulseOximeter_test/PulseOximeter_test.ino?ref_type=heads)
   * This code uses the serial plotter instead of the serial monitor. The numbers will still be shown on the monitor but the data will be very hard to understand without being in a graph.

***
### **Servo**
* [Part Link](https://wiki.seeedstudio.com/Grove-Servo/)
* #### Libraries
   * [Servo](https://docs.arduino.cc/libraries/servo/) - Also available in Ardiuno IDE Library Manger
* #### Documents
   * [Manual](https://www.manualslib.com/manual/1836691/Seeed-Grove-Servo-Series.html)
* #### Hookup
   * Using one of the digital ports (D5) on the Base Shield, simply use the provided 4-prong connection wires to directly connect the sensor to the Base Shield, ensuring proper connections are made (GND to GND, VCC to VCC, etc.)
* #### Code
   * [Servo Test Code](Arduino_Code/Part_Testing/Servo_test/Servo_test.ino?ref_type=heads)


***
### **Sound Sensor**
* [Part Link](https://wiki.seeedstudio.com/Grove-Sound_Sensor/)
* #### Libraries
   * No additional libraries needed for this sensor.
* #### Documents
   * [Datasheet](https://www.mouser.com/datasheet/2/744/Seeed_101020015-1217523.pdf?srsltid=AfmBOorwHlJCEQqf9S8z5mSSTPdVx3PrN5UK1Yeg_4D4VB85bLY0FuRT)
* #### Hookup
   * Using one of the analog ports (A0) on the Base Shield, simply use the provided 4-prong connection wires to directly connect the sensor to the Base Shield, ensuring proper connections are made (GND to GND, VCC to VCC, etc.)
* #### Code
   * [Sound Sensor Test Code](Arduino_Code/Part_Testing/SoundSensor_test/SoundSensor_test.ino?ref_type=heads)<br><br>


***
### **Temperature Sensor**
* [Part Link](https://www.digikey.ca/en/products/detail/seeed-technology-co-ltd/101020015/5482612?gclsrc=aw.ds&&utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Product_Low%20ROAS%20Categories&utm_term=&productid=5482612&utm_content=&utm_id=go_cmp-20291741422_adg-_ad-__dev-c_ext-_prd-5482612_sig-Cj0KCQiAs5i8BhDmARIsAGE4xHzTSHPixUNWPe_Sz5zNK9TpqalWp9gAVqcVxikBVv6sWPFyKczcFioaAkwVEALw_wcB&gad_source=1&gclid=Cj0KCQiAs5i8BhDmARIsAGE4xHzTSHPixUNWPe_Sz5zNK9TpqalWp9gAVqcVxikBVv6sWPFyKczcFioaAkwVEALw_wcB&gclsrc=aw.ds)
* #### Libraries
   * [DFRobot_MAX30102 Arduino Library ](https://github.com/DFRobot/DFRobot_MAX30102)
* #### Documents
   * [Datasheet](https://www.mouser.com/datasheet/2/744/Seeed_101020015-1217523.pdf?srsltid=AfmBOorwHlJCEQqf9S8z5mSSTPdVx3PrN5UK1Yeg_4D4VB85bLY0FuRT)
* #### Hookup
   * Using one of the analog ports (A0) on the Base Shield, simply use the provided 4-prong connection wires to directly connect the sensor to the Base Shield, ensuring proper connections are made (GND to GND, VCC to VCC, etc.)
* #### Code
   * [Temp Test Code](Arduino_Code/Part_Testing/TemperatureSensor_test/TemperatureSensor_test.ino?ref_type=heads)
