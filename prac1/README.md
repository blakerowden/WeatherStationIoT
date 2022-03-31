# **CSSE4011 Prac1: Weather Station**
## Application Host Unit - Blake Rowden 44276340
## Sensor Controller Unit - Jack Mason 44092353
---
## **Application Host Unit: Design Task Functionality**
### *Task 1: System Timer Shell Command*
- System Timer shell command `time` will print the uptime in seconds
- System Timer shell command `time f` will print the up time formatted as hours, minutes, seconds.
### *Task 2: AHU LED Shell Command*
- Shell command `led` is functional with two layer subcommands
    - on(`-o`), off (`-f`) and toggle (`-t`)
    - red (`-r`), green (`-g`) and blue (`-b`)
### *Task 3: Log Message Display*
- debug, **info**, <span style="color:yellow">warning</span> and <span style="color:red">**error**</span> messages have been implimented using the Zepher logging API
- Most *.c* files have a unique logging module
- Logging settings are implimented in oslib/common/log_driver.h
- The logging system is backended onto the shell
### *Task 4: SCU Command Interface*
- The following shell commands have been created to read and display the sensor values from the SCU
```
hts221 r t #temperature
hts221 r h #humidity
lps22 r #pressure
ccs811 r #VOC
lis2dh r x #X acceleration
lis2dh r y #Y acceleration
lis2dh r z #Z acceleration
buzzer w <freq> #buzzer frequency
rgb w <r, g, b> #RGB LED
pb r #pushbutton state (0 or 1)
dc w #configure the duty cycle (percentage on)
sample w #set sampling time (seconds)
```
### *Task 5: Bluetooth Communications*
- Bluetooth is used for communicating between the SCU and AHU using the standard Zephyr Bluetooth libraries to establish a **connection** between the two devices.
- Devices then communicate by writting to each others attributes; AHU (RX) and SCU (TX).
### *Task 6: Host Controller Interface*
- The Host Controller Interface (HCI) is used to allow the SCU to be controlled by the AHU.
- The protocol settings and functions can be found in oslib/common
### *Task 7: Zepher OS Libraries*
- The design uses Zephyr with a multi-threaded design (Only threads in main, no processing) and uses semaphores.
### *Task 8: JSON Interface*
- Continuous sampling can be enabled/disabled via the `all o/f` command and the onboard push button.
- The continuous sampling data is then packaged up into **JSON** format and printed to the terminal
### *Task 9: MQTT Publisher*
- A python script *mqtt_publish.py* located in prac1/ahu has been created to publish the **JSON** data printed through the `all o` command.
- When the script is run the termial is accessed via two threads one for reading and one for writting.
- A third thread then takes the terminal output passed in from the reading thread in a pipe and scans for **JSON** data
- When the data is found it is published to a local broker
---
## **Sensor Controller Unit: Design Task Functionality**
### Task 1:
-
---
## **Folder Structure**
```
repo/
├── oslib
│   ├── ahu_drivers
│   │   ├── ahu_ble
│   │   │   ├── ble_base.c
│   │   │   └── ble_base.h
│   │   ├── ahu_data
│   │   │   ├── ahu_data.c
│   │   │   └── ahu_data.h
│   │   └── ahu_shell
│   │       ├── shell_base.c
│   │       ├── shell_base.h
│   │       ├── shell_led.c
│   │       ├── shell_led.h
│   │       ├── shell_scu.c
│   │       ├── shell_scu.h
│   │       ├── shell_time.c
│   │       └── shell_time.h
│   ├── common
│   │   ├── ble_uuid.h
│   │   ├── hci_driver.c
│   │   ├── hci_driver.h
│   │   ├── led_driver.c
│   │   ├── led_driver.h
│   │   ├── log_driver.h
│   │   ├── pb_driver.c
│   │   └── pb_driver.h
│   └── scu_drivers
│       ├── scu_ble
│       │   ├── scu_ble.c
│       │   └── scu_ble.h
│       ├── scu_power_management
│       │   ├── scu_power_management.c
│       │   └── scu_power_management.h
│       └── scu_sensors
│           ├── scu_sensors.c
│           └── scu_sensors.h
├── prac1
│   ├── ahu
│   │   ├── autobuild
│   │   ├── bt.conf
│   │   ├── CMakeLists.txt
│   │   ├── dtc_shell.overlay
│   │   ├── mqtt_publish.py
│   │   ├── mqtt_subscribe.py
│   │   ├── prj.conf
│   │   ├── shell.conf
│   │   ├── src
│   │   │   └── main.c
│   │   └── usb.conf
│   ├── README.md
│   └── scu
│       ├── autobuild
│       ├── CMakeLists.txt
│       ├── prj.conf
│       ├── sample.yaml
│       └── src
│           └── main.c
└── README.md

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
1. **Simple Run**: Call `./autobuild.sh -r` from prac1/ahu to start the interactive AHU terminal
2. **MQTT Run**: First start a `mosquitto` client on `localhost`. Then run *mqtt_publish.py* and input the port number. Congrats, you now have access to terminal++. Finally you can run  *mqtt_subscribe.py* to test the publishing.
##### Other Notes:
> Debug settings can be set under the `==Debug Settings==` comment in `main.c`

> If the user wishes to publish to a MQTT broker run the python 
> script *mqtt_publish.py* **DO NOT USE** `./autobuild.sh -r`. The terminal will be accessable from the python script.
### **Sensor Controller Unit Build Instructions**