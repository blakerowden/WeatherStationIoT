# Weather Station - Application Host Unit
## Blake Rowden s4427634
---
### Design Task Functionality.
> Task 1: LED
- 
> Task 2: Timer
-
---
### Folder Structure
```
.
│     
├── oslib
│   ├── ahu_drivers
│   │   ├── ahu_ble
│   │   │   ├── ahu_ble.c
│   │   │   └── ahu_ble.h
│   │   └── ahu_shell
│   │       ├── ahu_shell.c
│   │       └── ahu_shell.h
│   └── common
│       ├── led_driver.c
│       └── led_driver.h
└── prac1
    ├── ahu
    │   ├── app.overlay
    │   ├── CMakeLists.txt
    │   ├── dtc_shell.overlay
    │   ├── prj.conf
    │   ├── shell.conf
    │   ├── src
    │   │   └── main.c
    │   └── usb.conf
    ├── autobuild
    └── README.md
```
---
>List any References used

### Instructions
All building tasks can be done using the shell script `./autobuild.sh -b -p -r`
- `-b` Will build the AHU
- `-p` Will package and flash the AHU for the USB dongle
- `-r` Will open the AHU CLI 