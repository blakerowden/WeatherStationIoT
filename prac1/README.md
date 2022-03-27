# **CSSE4011 Prac1: Weather Station**
## Application Host Unit - Blake Rowden 44276340
## Sensor Controller Unit - Jack Mason 44092353
---
## **Application Host Unit: Design Task Functionality**
> Task 1: LED
- 
> Task 2: Timer
-
---
## **Application Host Unit: Design Task Functionality**
> Task1:
-
> Task 2:
-
---
## **Folder Structure**
```
repo/
├── oslib
│   ├── ahu_drivers
│   │   ├── ahu_ble
│   │   │   ├── ble_base.c
│   │   │   └── ble_base.h
│   │   └── ahu_shell
│   │       ├── shell_base.c
│   │       ├── shell_base.h
│   │       ├── shell_led.c
│   │       ├── shell_led.h
│   │       ├── shell_scu.c
│   │       ├── shell_scu.h
│   │       ├── shell_time.c
│   │       └── shell_time.h
│   ├── common
│   │   ├── ble_uuid.h
│   │   ├── hci_driver.c
│   │   ├── hci_driver.h
│   │   ├── led_driver.c
│   │   └── led_driver.h
│   └── scu_drivers
│       ├── scu_ble
│       │   ├── scu_ble.c
│       │   └── scu_ble.h
│       ├── scu_power_management
│       │   ├── scu_power_management.c
│       │   └── scu_power_management.h
│       └── scu_sensors
│           ├── scu_sensors.c
│           └── scu_sensors.h
└── prac1
    ├── ahu
    │   ├── autobuild
    │   ├── bt.conf
    │   ├── CMakeLists.txt
    │   ├── dtc_shell.overlay
    │   ├── mqtt_publish.py
    │   ├── prj.conf
    │   ├── shell.conf
    │   ├── src
    │   │   └── main.c
    │   └── usb.conf
    ├── README.md
    └── scu
        ├── autobuild
        ├── CMakeLists.txt
        ├── prj.conf
        ├── sample.yaml
        └── src
            └── main.c
```
---
## **References Used**
- Sample BLE Connect: https://github.com/uqembeddedsys/zephyr-examples.git
- Sample MQTT: https://github.com/uqembeddedsys/zephyr-examples.git
- Gatt Client Test: https://github.com/zephyrproject-rtos/zephyr.git

---
## **Instructions**
### **Application Host Unit Build Instructions**
All building tasks can be done using the shell script `./autobuild.sh -b -p -r`
- `-b` Will build the AHU
- `-p` Will package and flash the AHU for the USB dongle
- `-r` Will open the AHU CLI
### **Application Host Unit User Instructions**
- Debug settings can be set under the `Debug Settings` header in `main.c` i.e
```
// Debug Settings ==============================================================
#define DEBUG_BLE_LED 1 // Flashing LED on (BLE)disconnect and solid on connect
```
### **Sensor Controller Unit Build Instructions**