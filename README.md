
![COM GND Logo](https://raw.githubusercontent.com/COM-GND/com-gnd-espresso/main/docs/com-gnd_logo_left-lock.svg)


# COM-GND Espresso

COM-GND Espresso is an open-source hardware and firmware project aiming to develop a standard set of sensing and control modules for Espresso machines.

With the growing accessibility of advanced microcontrollers and sensors, it has become significantly easier to develop automated control systems. COM-GND Espresso intends to establish a common set of low-cost, plug-and-play electronic components so that attention can be focused on experimentation through software control.

This hardware project is under active development in conjunction with the [GND-CTRL](https://github.com/COM-GND/gnd-ctrl-web-app) software interface.


---

### WARNING:  RISK OF DEATH

This project works with 110V AC Current. Use at your own risk. 

---

### WARNING: ALPHA STAGE PROJECT

This project is unstable and undergoing active development.  

---

## COM-GND Espresso Modules
- Pressure Sensor for pressure profile feedback
- Variable pump power modulator for pressure profile control
- Pressure knob rotary encoder for manual pump control
- BLE (Bluetooth) interface for App control  
- Boiler temperature sensor / PID control
### Forthcoming Modules
- Bluetooth shot scale integration
- Water-tank scale for flow profiling
 
---
## Hardware Stack

- [Esp32 DevkitC](https://www.amazon.com/Espressif-ESP32-DevKitC-32UE-Development-Board/dp/B087T94ZH9): Microcontroller with onboard BLE
- [Robotdyn AC Modulator](https://www.amazon.com/gp/product/B071X19VL1/): Pump power modulation
- [TE Analog Pressure Transducer](https://www.te.com/usa-en/product-10218849-00.html): 10 Bar boiler pressure sensor
   - Note: For new projects, the [Digital I2C version](https://www.te.com/usa-en/product-20003318-00.html) is recomended for easier interfacing with the ESP32
   - Note: adding a pressure sensor requires altering the machine's internal plumbing. 
- [Vishay Temperature Sensor](https://www.mouser.com/ProductDetail/vishay/ntcalug02a103f/?qs=AtFvwFU%2F1FnuJdiGNBrReQ%3D%3D&countrycode=US&currencycode=USD)

## Development

This is a [Platform.io](https://platformio.org/) based project using the Arduino framework.

