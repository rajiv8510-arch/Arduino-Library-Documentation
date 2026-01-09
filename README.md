# Complete Arduino Library API Reference

**Master Documentation for All 21 Rajiv Arduino Libraries**  
**Version:** 2.0 | **Last Updated:** January 9, 2026

---

## 🎯 All 21 Libraries

### 🔴 Core I/O (5 libraries)
1. **RajivSensorDI** - Digital sensors, debouncing, tank level monitoring
2. **RajivRelay** - Latch/momentary/dual-coil relay control
3. **RajivButton** - Advanced button handling (long press, double-click)
4. **RajivDigitalOutput** - Base class for digital outputs
5. **RajivSensorAnalog** - Analog sensors with filtering

### 🔵 Communication (4 libraries)
6. **RajivRS485** - Pfeiffer RPT200 vacuum sensor communication
7. **RajivRFM69** - RFM69 radio wrapper for security systems
8. **rs485** - Industrial Modbus RTU
9. **RajivNextion** - Nextion HMI display

### 🟢 Power & System (3 libraries)
10. **RajivPowerManager** - Battery monitoring & deep sleep
11. **RajivEEPROM** - EEPROM storage with wear leveling
12. **RajivWatchDog** - Hardware watchdog timer

### 🟡 Timing (4 libraries)
13. **RajivTimerOne** - Hardware Timer 1
14. **RajivTimerThree** - Hardware Timer 3
15. **RajivTimerFour** - Hardware Timer 4
16. **RajivTimerFive** - Hardware Timer 5

### ⚫ Motor & Actuation (4 libraries)
17. **RajivDCMotor** - DC motor PWM control
18. **RajivTimeredStepper** - Timer-based stepper motor
19. **RajivValveManager** - Valve control & management
20. **RajivPumpNModeConfig** - Pump configuration

### 🔶 Complete System (1)
21. **Commercial-Security-System** - Full integration example

---

## 🚀 Quick Example - Door Security Module

```cpp
#include <RajivSensorDI.h>
#include <RajivRFM69.h>
#include <RajivPowerManager.h>
#include <RajivWatchDog.h>

RajivSensorDI doorSensor(1, 1, "door");
RajivRFM69 radio(10, 2, true);
RajivPowerManager power;
RajivWatchDog wdt;

void setup() {
    wdt.begin(WDT_8S);
    doorSensor.initialize(4, SENSOR_NO, 13);
    power.begin(A0, 1.1);
    power.setBatteryType(BATTERY_CR2450_2X);
    radio.begin(10, 100, 1);
    radio.setTxPower(20);
}

void loop() {
    wdt.reset();
    if (doorSensor.hasChanged()) {
        radio.wake();
        radio.sendAlarm(EVENT_DOOR_OPEN, power.readBatteryVoltage());
        radio.sleep();
    }
    power.enterDeepSleep(8);
}
```

---

## 📚 Repository Links

All libraries: https://github.com/rajiv8510-arch

Individual repositories:
- [RajivSensorDI](https://github.com/rajiv8510-arch/Arduino-Library-RajivSensorDI)
- [RajivRelay](https://github.com/rajiv8510-arch/Arduino-Library-RajivRelay)
- [RajivButton](https://github.com/rajiv8510-arch/Arduino-Library-RajivButton)
- [RajivRS485](https://github.com/rajiv8510-arch/Arduino-Library-RajivRS485)
- [RajivPowerManager](https://github.com/rajiv8510-arch/Arduino-Library-RajivPowerManager)
- [RajivRFM69](https://github.com/rajiv8510-arch/Arduino-Library-RajivRFM69)
- And 15 more...

---

**⭐ This is the SINGLE SOURCE OF TRUTH for all Rajiv Arduino libraries**

*Maintained by Rajiv Yadav | Last updated: January 9, 2026*
