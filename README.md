# COMPLETE ARDUINO LIBRARY API REFERENCE

**Comprehensive Master Documentation for All 21 Rajiv Arduino Libraries**  
**Version:** 2.0 | **Last Updated:** January 9, 2026  
**Author:** Rajiv Yadav

---

## 📚 Purpose

This document is the **SINGLE SOURCE OF TRUTH** for all 21 Arduino libraries, providing:

✅ **Every function** with complete signatures  
✅ **All parameters** with types and descriptions  
✅ **Return types** and values  
✅ **Usage examples** for each function  
✅ **AI-friendly format** for code generation

---

## 📑 Complete Library Index (21 Libraries)

1. [RajivSensorDI](#1-rajiv-sensordi) - Digital Input Sensors
2. [RajivRelay](#2-rajivrelay) - Relay Control
3. [RajivButton](#3-rajivbutton) - Button Handling
4. [RajivRS485](#4-rajivrs485) - RS485 Communication
5. [RajivPowerManager](#5-rajivpowermanager) - Power Management
6. [RajivRFM69](#6-rajivrfm69) - Radio Communication
7. [RajivNextion](#7-rajivnextion) - Nextion Display
8. [RajivEEPROM](#8-rajiveeprom) - EEPROM Storage
9. [RajivWatchDog](#9-rajivwatchdog) - Watchdog Timer
10. [RajivDigitalOutput](#10-rajivdigitaloutput) - Digital Output Base
11. [RajivTimerOne](#11-rajivtimerone) - Timer 1
12. [RajivTimerThree](#12-rajivtimerthree) - Timer 3
13. [RajivTimerFour](#13-rajivtimerfour) - Timer 4
14. [RajivTimerFive](#14-rajivtimerfive) - Timer 5
15. [RajivDCMotor](#15-rajivdcmotor) - DC Motor
16. [RajivTimeredStepper](#16-rajivtimeredstepper) - Stepper Motor
17. [RajivValveManager](#17-rajivvalvemanager) - Valve Control
18. [RajivSensorAnalog](#18-rajivsensoranalog) - Analog Sensors
19. [RajivPumpNModeConfig](#19-rajivpumpnmodeconfig) - Pump Config
20. [rs485](#20-rs485) - Modbus RTU

---

# 1. RajivSensorDI

**Digital Input Sensor Library with Debouncing, Edge Detection, and Tank Level Monitoring**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivSensorDI

## Constructor

```cpp
RajivSensorDI(uint8_t pid, uint8_t cid, const char *name)
```
**Parameters:**
- `pid` - Parent ID (uint8_t)
- `cid` - Component ID (uint8_t)
- `name` - Sensor name (const char*)

**Returns:** Object instance

---

## Initialization Functions

### initialize()
```cpp
void initialize(uint8_t sensorPin, bool sensorType)
void initialize(uint8_t sensorPin, bool sensorType, uint8_t ledPin)
void initialize(uint8_t sensorPin, bool sensorType, uint8_t ledPin, unsigned long debounceMs)
```
**Parameters:**
- `sensorPin` - Arduino pin number (uint8_t, 0-255)
- `sensorType` - `SENSOR_NO` (normally open) or `SENSOR_NC` (normally closed) (bool)
- `ledPin` - LED indicator pin (uint8_t, optional)
- `debounceMs` - Debounce time in milliseconds (unsigned long, default: 50)

**Returns:** void

**Example:**
```cpp
RajivSensorDI door(1, 1, "door");
door.initialize(4, SENSOR_NO, 13, 50);
```

### initializeAsLevelSwitch()
```cpp
void initializeAsLevelSwitch(uint8_t sensorPin, bool sensorType, bool installPosition)
void initializeAsLevelSwitch(uint8_t sensorPin, bool sensorType, bool installPosition, uint8_t ledPin)
void initializeAsLevelSwitch(uint8_t sensorPin, bool sensorType, bool installPosition, uint8_t ledPin, unsigned long debounceMs)
```
**Parameters:**
- `sensorPin` - Arduino pin number (uint8_t)
- `sensorType` - `SENSOR_NO` or `SENSOR_NC` (bool)
- `installPosition` - `LEVEL_TOP` (tank full) or `LEVEL_BOTTOM` (tank empty) (bool)
- `ledPin` - LED indicator pin (uint8_t, optional)
- `debounceMs` - Debounce time (unsigned long, default: 50)

**Returns:** void

**Example:**
```cpp
RajivSensorDI topSwitch(1, 1, "tankTop");
topSwitch.initializeAsLevelSwitch(3, SENSOR_NO, LEVEL_TOP, 13);
```

---

## Reading Functions

### read()
```cpp
bool read()
```
**Parameters:** None

**Returns:** bool - Current sensor state after debouncing (true = active, false = inactive)

**Example:**
```cpp
if (sensor.read()) {
    Serial.println("Sensor active");
}
```

### readRaw()
```cpp
bool readRaw()
```
**Parameters:** None

**Returns:** bool - Raw pin state without debouncing

### readSensorAlarm()
```cpp
bool readSensorAlarm()
```
**Parameters:** None

**Returns:** bool - Sensor state with 5-second alarm delay (prevents false alarms)

**Example:**
```cpp
if (sensor.readSensorAlarm()) {
    triggerAlarm(); // Only after 5 seconds of continuous activation
}
```

### getState()
```cpp
bool getState()
```
**Parameters:** None

**Returns:** bool - Last read sensor state

### getAlarmState()
```cpp
bool getAlarmState()
```
**Parameters:** None

**Returns:** bool - Current alarm state

### hasChanged()
```cpp
bool hasChanged()
```
**Parameters:** None

**Returns:** bool - true if state changed since last check

**Example:**
```cpp
if (sensor.hasChanged()) {
    Serial.println("State changed!");
}
```

---

## Level Switch Functions

### isLevelDetected()
```cpp
bool isLevelDetected()
```
**Parameters:** None

**Returns:** bool - true if liquid is at switch position

### isTankFull()
```cpp
bool isTankFull()
```
**Parameters:** None

**Returns:** bool - true if tank is full (for LEVEL_TOP switches)

**Example:**
```cpp
if (topSwitch.isTankFull()) {
    pump.deenergize();
}
```

### isTankEmpty()
```cpp
bool isTankEmpty()
```
**Parameters:** None

**Returns:** bool - true if tank is empty (for LEVEL_BOTTOM switches)

### readLevel()
```cpp
bool readLevel()
```
**Parameters:** None

**Returns:** bool - Level detection state

### readLevelAlarm()
```cpp
bool readLevelAlarm()
```
**Parameters:** None

**Returns:** bool - Level state with 5-second alarm delay

---

## Edge Detection Functions

### risingEdge()
```cpp
bool risingEdge()
```
**Parameters:** None

**Returns:** bool - true if rising edge detected (LOW to HIGH transition)

**Example:**
```cpp
if (sensor.risingEdge()) {
    Serial.println("Sensor activated!");
}
```

### fallingEdge()
```cpp
bool fallingEdge()
```
**Parameters:** None

**Returns:** bool - true if falling edge detected (HIGH to LOW transition)

### edgeDetected()
```cpp
bool edgeDetected(EdgeMode mode)
```
**Parameters:**
- `mode` - `EDGE_RISING`, `EDGE_FALLING`, or `EDGE_BOTH` (EdgeMode enum)

**Returns:** bool - true if specified edge detected

---

## Callback Functions

### onStateChange()
```cpp
void onStateChange(SensorCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback(bool state)` (SensorCallback)

**Returns:** void

**Example:**
```cpp
void stateChanged(bool state) {
    Serial.print("New state: ");
    Serial.println(state);
}

sensor.onStateChange(stateChanged);
```

### onRisingEdge()
```cpp
void onRisingEdge(SensorCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback(bool state)` (SensorCallback)

**Returns:** void

### onFallingEdge()
```cpp
void onFallingEdge(SensorCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback(bool state)` (SensorCallback)

**Returns:** void

### onLevelDetected()
```cpp
void onLevelDetected(SensorCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback(bool state)` (SensorCallback)

**Returns:** void

### onLevelCleared()
```cpp
void onLevelCleared(SensorCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback(bool state)` (SensorCallback)

**Returns:** void

---

## Statistics Functions

### getRisingEdgeCount()
```cpp
uint32_t getRisingEdgeCount()
```
**Parameters:** None

**Returns:** uint32_t - Total rising edge count

### getFallingEdgeCount()
```cpp
uint32_t getFallingEdgeCount()
```
**Parameters:** None

**Returns:** uint32_t - Total falling edge count

### getTotalReadCount()
```cpp
uint32_t getTotalReadCount()
```
**Parameters:** None

**Returns:** uint32_t - Total number of read() calls

### resetCounters()
```cpp
void resetCounters()
```
**Parameters:** None

**Returns:** void

### getTimeSinceLastChange()
```cpp
unsigned long getTimeSinceLastChange()
```
**Parameters:** None

**Returns:** unsigned long - Milliseconds since last state change

---

## Configuration Functions

### setDebounceTime()
```cpp
void setDebounceTime(unsigned long milliseconds)
```
**Parameters:**
- `milliseconds` - Debounce time (unsigned long, 0-4294967295)

**Returns:** void

### setSensorType()
```cpp
void setSensorType(bool sensorType)
```
**Parameters:**
- `sensorType` - `SENSOR_NO` or `SENSOR_NC` (bool)

**Returns:** void

### setInstallPosition()
```cpp
void setInstallPosition(bool position)
```
**Parameters:**
- `position` - `LEVEL_TOP` or `LEVEL_BOTTOM` (bool)

**Returns:** void

### setInverted()
```cpp
void setInverted(bool inverted)
```
**Parameters:**
- `inverted` - Invert logic (bool)

**Returns:** void

### enable()
```cpp
void enable()
```
**Parameters:** None

**Returns:** void

### disable()
```cpp
void disable()
```
**Parameters:** None

**Returns:** void

### isEnabled()
```cpp
bool isEnabled()
```
**Parameters:** None

**Returns:** bool - true if sensor is enabled

### setAutoUpdateHMI()
```cpp
void setAutoUpdateHMI(bool autoUpdate)
```
**Parameters:**
- `autoUpdate` - Enable auto HMI updates (bool)

**Returns:** void

---

## LED Control Functions

### setLEDState()
```cpp
void setLEDState(bool state)
```
**Parameters:**
- `state` - LED on/off (bool)

**Returns:** void

### toggleLED()
```cpp
void toggleLED()
```
**Parameters:** None

**Returns:** void

### setLEDFollowSensor()
```cpp
void setLEDFollowSensor(bool follow)
```
**Parameters:**
- `follow` - LED follows sensor state (bool)

**Returns:** void

---

## HMI Functions

### updateHMI()
```cpp
void updateHMI()
void updateHMI(bool state)
```
**Parameters:**
- `state` - State to display (bool, optional)

**Returns:** void

---

# 2. RajivRelay

**Comprehensive Relay Control Library (Latch, Momentary, Dual-Coil)**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivRelay

## Constructor

```cpp
RajivRelay(uint8_t pid, uint8_t cid, const char *name)
```
**Parameters:**
- `pid` - Parent ID (uint8_t)
- `cid` - Component ID (uint8_t)
- `name` - Relay name (const char*)

**Returns:** Object instance

---

## Initialization Functions

### initialize() - Latch Relay
```cpp
void initialize(uint8_t relayPin, bool actingVoltage)
void initialize(uint8_t relayPin, bool actingVoltage, uint8_t ledPin)
```
**Parameters:**
- `relayPin` - Arduino pin number (uint8_t)
- `actingVoltage` - `ACTIVE_HIGH` or `ACTIVE_LOW` (bool)
- `ledPin` - LED indicator pin (uint8_t, optional)

**Returns:** void

**Example:**
```cpp
RajivRelay pump(1, 1, "WaterPump");
pump.initialize(7, ACTIVE_HIGH, 13);
```

### initializeMomentary() - Momentary Relay
```cpp
void initializeMomentary(uint8_t setPin, uint8_t resetPin, bool activeHigh, uint8_t ledPin)
void initializeMomentary(uint8_t setPin, uint8_t resetPin, bool activeHigh, uint8_t ledPin, uint16_t pulseMs)
```
**Parameters:**
- `setPin` - Set coil pin (uint8_t)
- `resetPin` - Reset coil pin (uint8_t)
- `activeHigh` - `ACTIVE_HIGH` or `ACTIVE_LOW` (bool)
- `ledPin` - LED pin (uint8_t)
- `pulseMs` - Pulse width in milliseconds (uint16_t, default: 50)

**Returns:** void

### initializeDualCoil() - Dual-Coil Relay
```cpp
void initializeDualCoil(uint8_t setPin, uint8_t resetPin, bool activeHigh)
void initializeDualCoil(uint8_t setPin, uint8_t resetPin, bool activeHigh, uint8_t ledPin)
```
**Parameters:**
- `setPin` - Set coil pin (uint8_t)
- `resetPin` - Reset coil pin (uint8_t)
- `activeHigh` - `ACTIVE_HIGH` or `ACTIVE_LOW` (bool)
- `ledPin` - LED pin (uint8_t, optional)

**Returns:** void

---

## Control Functions (New API)

### energize()
```cpp
void energize()
```
**Parameters:** None

**Returns:** void

**Purpose:** Turn relay ON

**Example:**
```cpp
pump.energize();
```

### deenergize()
```cpp
void deenergize()
```
**Parameters:** None

**Returns:** void

**Purpose:** Turn relay OFF

### pulse()
```cpp
void pulse(uint16_t milliseconds)
```
**Parameters:**
- `milliseconds` - Pulse duration (uint16_t, 1-65535)

**Returns:** void

**Purpose:** Generate single pulse

**Example:**
```cpp
valve.pulse(200); // 200ms pulse
```

### delayedActivate()
```cpp
void delayedActivate(unsigned long milliseconds)
```
**Parameters:**
- `milliseconds` - Delay before activation (unsigned long)

**Returns:** void

**Purpose:** Turn ON after delay

### delayedDeactivate()
```cpp
void delayedDeactivate(unsigned long milliseconds)
```
**Parameters:**
- `milliseconds` - Delay before deactivation (unsigned long)

**Returns:** void

**Purpose:** Turn OFF after delay

### timedActivate()
```cpp
void timedActivate(unsigned long milliseconds)
```
**Parameters:**
- `milliseconds` - Active duration (unsigned long)

**Returns:** void

**Purpose:** Turn ON for duration, then auto OFF

**Example:**
```cpp
siren.timedActivate(5000); // ON for 5 seconds
```

---

## Control Functions (Backward Compatible)

### latchCommand()
```cpp
void latchCommand(bool activate, bool wled)
void latchCommand(bool activate, bool wled, bool whmi)
void latchCommand(bool activate, uint8_t delayS, bool wled)
```
**Parameters:**
- `activate` - true = ON, false = OFF (bool)
- `wled` - Control LED (bool)
- `whmi` - Update HMI (bool)
- `delayS` - Delay in seconds (uint8_t)

**Returns:** void

### momentoryCommand()
```cpp
void momentoryCommand(bool activate, bool wled)
void momentoryCommand(bool activate, uint8_t delayS, bool wled)
```
**Parameters:**
- `activate` - true = SET, false = RESET (bool)
- `wled` - Control LED (bool)
- `delayS` - Delay in seconds (uint8_t)

**Returns:** void

---

## Configuration Functions

### setPulseWidth()
```cpp
void setPulseWidth(uint16_t milliseconds)
```
**Parameters:**
- `milliseconds` - Pulse width (uint16_t, 1-65535)

**Returns:** void

### getPulseWidth()
```cpp
uint16_t getPulseWidth()
```
**Parameters:** None

**Returns:** uint16_t - Current pulse width in milliseconds

---

## Safety Functions

### isSafeToOperate()
```cpp
bool isSafeToOperate()
```
**Parameters:** None

**Returns:** bool - true if safe to operate

### enableInterlock()
```cpp
void enableInterlock(bool enable)
```
**Parameters:**
- `enable` - Enable/disable interlock (bool)

**Returns:** void

### isInterlocked()
```cpp
bool isInterlocked()
```
**Parameters:** None

**Returns:** bool - true if interlocked

---

## Inherited from RajivDigitalOutput

### enable()
```cpp
void enable()
```
**Parameters:** None

**Returns:** void

### disable()
```cpp
void disable()
```
**Parameters:** None

**Returns:** void

### isEnabled()
```cpp
bool isEnabled()
```
**Parameters:** None

**Returns:** bool

### activate()
```cpp
void activate()
```
**Parameters:** None

**Returns:** void

### deactivate()
```cpp
void deactivate()
```
**Parameters:** None

**Returns:** void

### toggle()
```cpp
void toggle()
```
**Parameters:** None

**Returns:** void

### setState()
```cpp
void setState(bool state)
```
**Parameters:**
- `state` - ON/OFF (bool)

**Returns:** void

### getState()
```cpp
bool getState()
```
**Parameters:** None

**Returns:** bool - Current state

### setLEDState()
```cpp
void setLEDState(bool state)
```
**Parameters:**
- `state` - LED ON/OFF (bool)

**Returns:** void

### toggleLED()
```cpp
void toggleLED()
```
**Parameters:** None

**Returns:** void

### setLEDFollowSensor()
```cpp
void setLEDFollowSensor(bool follow)
```
**Parameters:**
- `follow` - LED follows state (bool)

**Returns:** void

### updateHMI()
```cpp
void updateHMI()
```
**Parameters:** None

**Returns:** void

### getOnTime()
```cpp
unsigned long getOnTime()
```
**Parameters:** None

**Returns:** unsigned long - Current ON time in milliseconds

### getOffTime()
```cpp
unsigned long getOffTime()
```
**Parameters:** None

**Returns:** unsigned long - Current OFF time in milliseconds

### getTotalOnTime()
```cpp
unsigned long getTotalOnTime()
```
**Parameters:** None

**Returns:** unsigned long - Total ON time in milliseconds

### getTotalOffTime()
```cpp
unsigned long getTotalOffTime()
```
**Parameters:** None

**Returns:** unsigned long - Total OFF time in milliseconds

### getCycleCount()
```cpp
uint32_t getCycleCount()
```
**Parameters:** None

**Returns:** uint32_t - Number of ON/OFF cycles

### resetStatistics()
```cpp
void resetStatistics()
```
**Parameters:** None

**Returns:** void

---

# 3. RajivButton

**Advanced Button Handling Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivButton

## Constructor

```cpp
RajivButton(uint8_t pin)
RajivButton(uint8_t pin, unsigned long debounceMs)
RajivButton(uint8_t pin, unsigned long debounceMs, unsigned long longPressMs)
```
**Parameters:**
- `pin` - Arduino pin number (uint8_t)
- `debounceMs` - Debounce time (unsigned long, default: 50)
- `longPressMs` - Long press threshold (unsigned long, default: 1000)

**Returns:** Object instance

---

## Initialization

### begin()
```cpp
void begin()
void begin(bool enablePullup)
```
**Parameters:**
- `enablePullup` - Enable internal pullup resistor (bool, default: true)

**Returns:** void

---

## Configuration Functions

### setDebounceTime()
```cpp
void setDebounceTime(unsigned long milliseconds)
```
**Parameters:**
- `milliseconds` - Debounce time (unsigned long)

**Returns:** void

### setLongPressTime()
```cpp
void setLongPressTime(unsigned long milliseconds)
```
**Parameters:**
- `milliseconds` - Long press threshold (unsigned long)

**Returns:** void

### setDoubleClickTime()
```cpp
void setDoubleClickTime(unsigned long milliseconds)
```
**Parameters:**
- `milliseconds` - Double-click window (unsigned long, default: 400)

**Returns:** void

### setInvertLogic()
```cpp
void setInvertLogic(bool invert)
```
**Parameters:**
- `invert` - Invert button logic (bool)

**Returns:** void

---

## Reading Functions

### read()
```cpp
bool read()
```
**Parameters:** None

**Returns:** bool - true if button state changed (call in loop())

**Example:**
```cpp
void loop() {
    button.read(); // Must call every loop
    if (button.wasPressed()) {
        // Handle press
    }
}
```

### isPressed()
```cpp
bool isPressed()
```
**Parameters:** None

**Returns:** bool - true if currently pressed

### isReleased()
```cpp
bool isReleased()
```
**Parameters:** None

**Returns:** bool - true if currently released

---

## Event Detection Functions

### wasPressed()
```cpp
bool wasPressed()
```
**Parameters:** None

**Returns:** bool - true once on press (triggers once per press)

**Example:**
```cpp
if (button.wasPressed()) {
    Serial.println("Pressed!");
}
```

### wasReleased()
```cpp
bool wasReleased()
```
**Parameters:** None

**Returns:** bool - true once on release

### wasLongPressed()
```cpp
bool wasLongPressed()
bool wasLongPressed(unsigned long duration)
```
**Parameters:**
- `duration` - Custom long press threshold (unsigned long, optional)

**Returns:** bool - true on long press event

**Example:**
```cpp
if (button.wasLongPressed(3000)) {
    Serial.println("3-second hold!");
}
```

### wasDoubleClicked()
```cpp
bool wasDoubleClicked()
```
**Parameters:** None

**Returns:** bool - true on double-click event

---

## Duration Functions

### pressedFor()
```cpp
bool pressedFor(unsigned long duration)
```
**Parameters:**
- `duration` - Time threshold (unsigned long, milliseconds)

**Returns:** bool - true if pressed for duration

**Example:**
```cpp
if (button.pressedFor(5000)) {
    factoryReset();
}
```

### releasedFor()
```cpp
bool releasedFor(unsigned long duration)
```
**Parameters:**
- `duration` - Time threshold (unsigned long)

**Returns:** bool - true if released for duration

### getPressedDuration()
```cpp
unsigned long getPressedDuration()
```
**Parameters:** None

**Returns:** unsigned long - Current press duration in milliseconds

### getLastPressDuration()
```cpp
unsigned long getLastPressDuration()
```
**Parameters:** None

**Returns:** unsigned long - Duration of last press

---

## Advanced Functions

### isLongPress()
```cpp
bool isLongPress()
```
**Parameters:** None

**Returns:** bool - true during long press

### getClickCount()
```cpp
uint8_t getClickCount()
```
**Parameters:** None

**Returns:** uint8_t - Number of clicks

### resetClickCount()
```cpp
void resetClickCount()
```
**Parameters:** None

**Returns:** void

### stateChanged()
```cpp
bool stateChanged()
```
**Parameters:** None

**Returns:** bool - true if state changed

### getCurrentState()
```cpp
bool getCurrentState()
```
**Parameters:** None

**Returns:** bool - Current button state

---

## Callback Functions

### onPress()
```cpp
void onPress(ButtonCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback()` (ButtonCallback)

**Returns:** void

**Example:**
```cpp
void handlePress() {
    Serial.println("Button pressed!");
}

button.onPress(handlePress);
```

### onRelease()
```cpp
void onRelease(ButtonCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback()` (ButtonCallback)

**Returns:** void

### onLongPress()
```cpp
void onLongPress(ButtonCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback()` (ButtonCallback)

**Returns:** void

### onDoubleClick()
```cpp
void onDoubleClick(ButtonCallback callback)
```
**Parameters:**
- `callback` - Function pointer: `void callback()` (ButtonCallback)

**Returns:** void

---

## Statistics Functions

### getPressCount()
```cpp
uint32_t getPressCount()
```
**Parameters:** None

**Returns:** uint32_t - Total press count

### getReleaseCount()
```cpp
uint32_t getReleaseCount()
```
**Parameters:** None

**Returns:** uint32_t - Total release count

### getTotalPressTime()
```cpp
unsigned long getTotalPressTime()
```
**Parameters:** None

**Returns:** unsigned long - Total time pressed (milliseconds)

### resetStatistics()
```cpp
void resetStatistics()
```
**Parameters:** None

**Returns:** void

---

## ButtonManager Class

### Constructor
```cpp
ButtonManager()
```

### addButton()
```cpp
bool addButton(RajivButton* button)
```
**Parameters:**
- `button` - Pointer to RajivButton object (RajivButton*)

**Returns:** bool - true if added successfully

### removeButton()
```cpp
void removeButton(RajivButton* button)
```
**Parameters:**
- `button` - Pointer to RajivButton object (RajivButton*)

**Returns:** void

### clear()
```cpp
void clear()
```
**Parameters:** None

**Returns:** void

### readAll()
```cpp
void readAll()
```
**Parameters:** None

**Returns:** void

**Purpose:** Read all managed buttons

### anyPressed()
```cpp
bool anyPressed()
```
**Parameters:** None

**Returns:** bool - true if any button pressed

### allReleased()
```cpp
bool allReleased()
```
**Parameters:** None

**Returns:** bool - true if all buttons released

### arePressedTogether()
```cpp
bool arePressedTogether(RajivButton* btn1, RajivButton* btn2)
bool arePressedTogether(RajivButton* btn1, RajivButton* btn2, RajivButton* btn3)
```
**Parameters:**
- `btn1`, `btn2`, `btn3` - Button pointers (RajivButton*)

**Returns:** bool - true if specified buttons pressed together

**Example:**
```cpp
if (manager.arePressedTogether(&btn1, &btn2)) {
    Serial.println("Combo activated!");
}
```

### getPressedCount()
```cpp
uint8_t getPressedCount()
```
**Parameters:** None

**Returns:** uint8_t - Number of currently pressed buttons

---

# 4. RajivRS485

**Pfeiffer RPT200 Vacuum Sensor RS-485 Communication Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivRS485

## Constructor

```cpp
PfeifferTR(uint8_t deviceAddress, HardwareSerial& serial, uint8_t dePin, uint8_t rePin, uint8_t txPin)
```
**Parameters:**
- `deviceAddress` - RS-485 device address (uint8_t, 1-32)
- `serial` - Serial port reference (HardwareSerial&, e.g., Serial1)
- `dePin` - Driver Enable pin (uint8_t)
- `rePin` - Receiver Enable pin (uint8_t)
- `txPin` - TX pin number (uint8_t)

**Returns:** Object instance

**Example:**
```cpp
PfeifferTR sensor1(1, Serial1, 2, 2, 18);
```

---

## Initialization Functions

### begin()
```cpp
void begin(uint32_t baudRate = 9600)
```
**Parameters:**
- `baudRate` - Baud rate (uint32_t, default: 9600)

**Returns:** void

**Note:** Only call begin() ONCE for the first sensor on a shared bus!

**Example:**
```cpp
sensor1.begin(9600); // Initialize Serial1
// DON'T call sensor2.begin() if sharing Serial1!
```

### setBusReleaseDelayMs()
```cpp
void setBusReleaseDelayMs(uint16_t milliseconds)
```
**Parameters:**
- `milliseconds` - Delay after transmission (uint16_t, default: 5)

**Returns:** void

### setRecvTimeoutMs()
```cpp
void setRecvTimeoutMs(uint32_t milliseconds)
```
**Parameters:**
- `milliseconds` - Receive timeout (uint32_t, default: 500)

**Returns:** void

**Example:**
```cpp
sensor.setRecvTimeoutMs(800); // 800ms timeout
```

---

## Pressure Reading Functions

### getPressure_uBar()
```cpp
bool getPressure_uBar(uint32_t* pressure)
```
**Parameters:**
- `pressure` - Pointer to store pressure value (uint32_t*, output in microbar)

**Returns:** bool - true if read successful

**Units:** Microbar (µBar)
- 1 mbar = 1000 µBar
- Divide by 1000.0 to get mbar

**Example:**
```cpp
uint32_t pressure_ubar;
if (sensor.getPressure_uBar(&pressure_ubar)) {
    float mbar = pressure_ubar / 1000.0;
    Serial.print(mbar, 3);
    Serial.println(" mbar");
}
```

---

## Device Information Functions

### getDeviceName()
```cpp
bool getDeviceName(String* name)
```
**Parameters:**
- `name` - Pointer to String for device name (String*, output)

**Returns:** bool - true if successful

**Example:**
```cpp
String deviceName;
if (sensor.getDeviceName(&deviceName)) {
    Serial.println(deviceName);
}
```

### getSrNumber()
```cpp
bool getSrNumber(String* serialNumber)
```
**Parameters:**
- `serialNumber` - Pointer to String for serial number (String*, output)

**Returns:** bool - true if successful

### getSwVersion()
```cpp
bool getSwVersion(String* version)
```
**Parameters:**
- `version` - Pointer to String for software version (String*, output)

**Returns:** bool - true if successful

### getHwVersion()
```cpp
bool getHwVersion(String* version)
```
**Parameters:**
- `version` - Pointer to String for hardware version (String*, output)

**Returns:** bool - true if successful

---

## Relay Configuration Functions

### setRelay1Sw_uBar()
```cpp
bool setRelay1Sw_uBar(uint32_t switchpoint)
```
**Parameters:**
- `switchpoint` - Relay 1 setpoint in microbar (uint32_t)

**Returns:** bool - true if successful

**Example:**
```cpp
sensor.setRelay1Sw_uBar(1000000); // 1000 mbar
```

### setRelay2Sw_uBar()
```cpp
bool setRelay2Sw_uBar(uint32_t switchpoint)
```
**Parameters:**
- `switchpoint` - Relay 2 setpoint in microbar (uint32_t)

**Returns:** bool - true if successful

---

## Utility Functions

### microToExponent()
```cpp
uint32_t microToExponent(uint32_t microbar)
```
**Parameters:**
- `microbar` - Pressure in microbar (uint32_t)

**Returns:** uint32_t - Exponent representation

### exponentToMicro()
```cpp
void exponentToMicro(uint32_t* exponent)
```
**Parameters:**
- `exponent` - Pointer to exponent value (uint32_t*, input/output)

**Returns:** void (modifies input value)

---

# 5. RajivPowerManager

**Battery Monitoring and Power Management Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivPowerManager

## Constructor

```cpp
RajivPowerManager()
```
**Returns:** Object instance

---

## Initialization Functions

### begin()
```cpp
void begin(uint8_t adcPin, float vRef = 1.1)
```
**Parameters:**
- `adcPin` - ADC pin for battery voltage (uint8_t, e.g., A0)
- `vRef` - ADC reference voltage (float, default: 1.1V for internal ref)

**Returns:** void

**Example:**
```cpp
power.begin(A0, 1.1); // A0 pin, 1.1V reference
```

### setBatteryType()
```cpp
void setBatteryType(BatteryType type)
```
**Parameters:**
- `type` - Battery type (BatteryType enum)

**Battery Types:**
- `BATTERY_CR2450` - 3.0V, 620mAh coin cell
- `BATTERY_CR2450_2X` - Dual CR2450 parallel (1240mAh)
- `BATTERY_CR2032` - 3.0V, 240mAh coin cell
- `BATTERY_LIPO_1S` - 3.7V Li-Po (3.0-4.2V)
- `BATTERY_LIPO_2S` - 7.4V Li-Po (6.0-8.4V)
- `BATTERY_LIPO_3S` - 11.1V Li-Po (9.0-12.6V)
- `BATTERY_LIION_1S` - 3.7V Li-ion (2.5-4.2V)
- `BATTERY_CUSTOM` - User-defined

**Returns:** void

**Example:**
```cpp
power.setBatteryType(BATTERY_CR2450_2X);
```

### setVoltageDivider()
```cpp
void setVoltageDivider(float r1, float r2)
```
**Parameters:**
- `r1` - Upper resistor value in ohms (float)
- `r2` - Lower resistor value in ohms (float)

**Returns:** void

**Example:**
```cpp
power.setVoltageDivider(10000, 10000); // 10K/10K divider
```

---

## Battery Monitoring Functions

### readBatteryVoltage()
```cpp
uint16_t readBatteryVoltage()
```
**Parameters:** None

**Returns:** uint16_t - Battery voltage in millivolts (mV)

**Example:**
```cpp
uint16_t voltage = power.readBatteryVoltage();
float volts = voltage / 1000.0;
Serial.print(volts);
Serial.println("V");
```

### getBatteryPercent()
```cpp
uint8_t getBatteryPercent()
```
**Parameters:** None

**Returns:** uint8_t - Battery percentage (0-100)

**Example:**
```cpp
uint8_t percent = power.getBatteryPercent();
Serial.print(percent);
Serial.println("%");
```

### isLowBattery()
```cpp
bool isLowBattery()
```
**Parameters:** None

**Returns:** bool - true if battery < 20%

### isCriticalBattery()
```cpp
bool isCriticalBattery()
```
**Parameters:** None

**Returns:** bool - true if battery < 5%

**Example:**
```cpp
if (power.isCriticalBattery()) {
    emergencyShutdown();
}
```

### getBatteryStatus()
```cpp
BatteryStatus getBatteryStatus()
```
**Parameters:** None

**Returns:** BatteryStatus enum
- `POWER_FULL` - > 80%
- `POWER_GOOD` - 20-80%
- `POWER_LOW` - 5-20%
- `POWER_CRITICAL` - 1-5%
- `POWER_EMPTY` - < 1%

---

## Power Mode Functions (AVR)

### enterDeepSleep()
```cpp
void enterDeepSleep(uint32_t seconds)
```
**Parameters:**
- `seconds` - Sleep duration (uint32_t)

**Returns:** void

**Current Draw:** ~1µA during sleep

**Example:**
```cpp
power.enterDeepSleep(60); // Sleep 60 seconds
```

### enterLightSleep()
```cpp
void enterLightSleep(uint32_t milliseconds)
```
**Parameters:**
- `milliseconds` - Sleep duration (uint32_t)

**Returns:** void

**Current Draw:** ~5µA during sleep

### wakeUp()
```cpp
void wakeUp()
```
**Parameters:** None

**Returns:** void

---

## Solar Charging Functions

### enableSolar()
```cpp
void enableSolar(uint8_t solarPin, uint8_t chargePin)
```
**Parameters:**
- `solarPin` - Solar panel voltage ADC pin (uint8_t)
- `chargePin` - Charge current ADC pin (uint8_t)

**Returns:** void

### isCharging()
```cpp
bool isCharging()
```
**Parameters:** None

**Returns:** bool - true if charging

### getSolarVoltage()
```cpp
uint16_t getSolarVoltage()
```
**Parameters:** None

**Returns:** uint16_t - Solar voltage in mV

### getChargeCurrent()
```cpp
uint16_t getChargeCurrent()
```
**Parameters:** None

**Returns:** uint16_t - Charge current in mA

---

## Statistics Functions

### getTotalSleepTime()
```cpp
uint32_t getTotalSleepTime()
```
**Parameters:** None

**Returns:** uint32_t - Total sleep time in seconds

### getTotalActiveTime()
```cpp
uint32_t getTotalActiveTime()
```
**Parameters:** None

**Returns:** uint32_t - Total active time in seconds

### getAverageCurrent()
```cpp
float getAverageCurrent(uint16_t capacity, float hours)
```
**Parameters:**
- `capacity` - Battery capacity in mAh (uint16_t)
- `hours` - Operating hours (float)

**Returns:** float - Average current draw in mA

### getEstimatedLifeHours()
```cpp
uint32_t getEstimatedLifeHours(float currentDraw)
```
**Parameters:**
- `currentDraw` - Average current in mA (float)

**Returns:** uint32_t - Estimated battery life in hours

---

# 6. RajivRFM69

**RFM69 Radio Communication Library for Security Systems**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivRFM69

## Constructor

```cpp
RajivRFM69(uint8_t nssPin, uint8_t dio0Pin, bool isHW)
```
**Parameters:**
- `nssPin` - NSS/CS pin (uint8_t)
- `dio0Pin` - DIO0/IRQ pin (uint8_t)
- `isHW` - true for RFM69HW/HCW, false for RFM69W/CW (bool)

**Returns:** Object instance

**Example:**
```cpp
RajivRFM69 radio(10, 2, true); // Pin 10=NSS, Pin 2=DIO0, HW variant
```

---

## Initialization Functions

### begin()
```cpp
bool begin(uint8_t nodeID, uint8_t networkID, uint8_t gatewayID = 1, uint8_t frequency = RF69_868MHZ)
```
**Parameters:**
- `nodeID` - This node's address (uint8_t, 1-254)
- `networkID` - Network ID (uint8_t, 0-255)
- `gatewayID` - Gateway/hub address (uint8_t, default: 1)
- `frequency` - `RF69_315MHZ`, `RF69_433MHZ`, `RF69_868MHZ`, or `RF69_915MHZ` (uint8_t)

**Returns:** bool - true if initialization successful

**Example:**
```cpp
if (radio.begin(10, 100, 1, RF69_868MHZ)) {
    Serial.println("Radio OK");
}
```

### setTxPower()
```cpp
void setTxPower(int8_t powerLevel)
```
**Parameters:**
- `powerLevel` - TX power in dBm (int8_t)
  - RFM69W: -18 to +13 dBm
  - RFM69HW: -2 to +20 dBm

**Returns:** void

**Example:**
```cpp
radio.setTxPower(20); // +20dBm for RFM69HW
```

### enableEncryption()
```cpp
void enableEncryption(const char* key)
```
**Parameters:**
- `key` - 16-character encryption key (const char*, exactly 16 chars)

**Returns:** void

**Example:**
```cpp
radio.enableEncryption("MySecretKey12345"); // Exactly 16 chars
```

---

## Packet Transmission Functions

### sendAlarm()
```cpp
bool sendAlarm(uint8_t eventType, uint16_t sensorValue = 0, uint32_t uptime = 0, uint8_t fwVersion = 0)
```
**Parameters:**
- `eventType` - Event type code (uint8_t)
- `sensorValue` - Sensor data (uint16_t, optional)
- `uptime` - Device uptime in seconds (uint32_t, optional)
- `fwVersion` - Firmware version (uint8_t, optional)

**Event Types:**
- `EVENT_DOOR_OPEN` (0x10)
- `EVENT_DOOR_CLOSE` (0x11)
- `EVENT_MOTION` (0x20)
- `EVENT_TAMPER` (0x30)
- `EVENT_LOW_BATTERY` (0x40)
- `EVENT_TEMP_ALERT` (0x50)

**Returns:** bool - true if sent successfully

**Example:**
```cpp
if (radio.sendAlarm(EVENT_DOOR_OPEN, batteryVoltage)) {
    Serial.println("Alarm sent");
}
```

### sendHeartbeat()
```cpp
bool sendHeartbeat(uint16_t sensorValue = 0, uint32_t uptime = 0, uint8_t fwVersion = 0)
```
**Parameters:**
- `sensorValue` - Sensor data (uint16_t, optional)
- `uptime` - Uptime in seconds (uint32_t, optional)
- `fwVersion` - Firmware version (uint8_t, optional)

**Returns:** bool - true if sent successfully

**Example:**
```cpp
radio.sendHeartbeat(batteryVoltage, millis()/1000);
```

### sendStatus()
```cpp
bool sendStatus(uint8_t eventType, uint16_t sensorValue = 0, uint32_t uptime = 0, uint8_t fwVersion = 0)
```
**Parameters:**
- `eventType` - Status code (uint8_t)
- `sensorValue` - Sensor data (uint16_t, optional)
- `uptime` - Uptime (uint32_t, optional)
- `fwVersion` - FW version (uint8_t, optional)

**Returns:** bool - true if sent successfully

---

## Packet Reception Functions

### receivePacket()
```cpp
bool receivePacket(SecurityPacket& packet, uint16_t timeout = 0)
```
**Parameters:**
- `packet` - Reference to SecurityPacket structure (SecurityPacket&, output)
- `timeout` - Timeout in milliseconds (uint16_t, 0 = non-blocking)

**Returns:** bool - true if packet received

**Example:**
```cpp
SecurityPacket packet;
if (radio.receivePacket(packet)) {
    Serial.print("From node: ");
    Serial.println(packet.nodeID);
    Serial.print("Event: ");
    Serial.println(packet.eventType, HEX);
    Serial.print("RSSI: ");
    Serial.println(packet.rssi);
}
```

### available()
```cpp
bool available()
```
**Parameters:** None

**Returns:** bool - true if packet available

---

## SecurityPacket Structure

```cpp
struct SecurityPacket {
    uint8_t nodeID;         // Sender node ID
    uint8_t packetType;     // 0x01=ALARM, 0x02=HEARTBEAT, 0x03=STATUS
    uint16_t sequence;      // Packet sequence number
    uint8_t eventType;      // Event type code
    uint16_t sensorValue;   // Sensor data
    int8_t rssi;           // Received signal strength
    uint32_t uptime;        // Sender uptime
    uint8_t fwVersion;      // Sender firmware version
    uint8_t checksum;       // Packet checksum
};
```

---

## RSSI & Diagnostics Functions

### getLastRSSI()
```cpp
int16_t getLastRSSI()
```
**Parameters:** None

**Returns:** int16_t - Last received signal strength in dBm (negative value)

**Example:**
```cpp
int16_t rssi = radio.getLastRSSI();
Serial.print("RSSI: ");
Serial.print(rssi);
Serial.println(" dBm");
```

### getAverageRSSI()
```cpp
int16_t getAverageRSSI()
```
**Parameters:** None

**Returns:** int16_t - Average RSSI in dBm

---

## Power Management Functions

### sleep()
```cpp
void sleep()
```
**Parameters:** None

**Returns:** void

**Current Draw:** ~0.3µA in sleep mode

**Example:**
```cpp
radio.sleep(); // Enter low power mode
```

### wake()
```cpp
void wake()
```
**Parameters:** None

**Returns:** void

### wakeUp()
```cpp
void wakeUp()
```
**Parameters:** None

**Returns:** void

---

## Statistics Functions

### getSentPackets()
```cpp
uint32_t getSentPackets()
```
**Parameters:** None

**Returns:** uint32_t - Total packets sent

### getReceivedPackets()
```cpp
uint32_t getReceivedPackets()
```
**Parameters:** None

**Returns:** uint32_t - Total packets received

### getFailedTransmissions()
```cpp
uint32_t getFailedTransmissions()
```
**Parameters:** None

**Returns:** uint32_t - Failed transmission count

### resetStatistics()
```cpp
void resetStatistics()
```
**Parameters:** None

**Returns:** void

---

# 7. RajivNextion

**Nextion HMI Display Communication Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivNextion

## Constructor

```cpp
RajivNextion(HardwareSerial& serial)
```
**Parameters:**
- `serial` - Serial port reference (HardwareSerial&, e.g., Serial1)

**Returns:** Object instance

**Example:**
```cpp
RajivNextion hmi(Serial1);
```

---

## Initialization

### begin()
```cpp
void begin(uint32_t baudRate = 9600)
```
**Parameters:**
- `baudRate` - Baud rate (uint32_t, default: 9600)

**Returns:** void

**Example:**
```cpp
hmi.begin(9600);
```

---

## Component Update Functions

### updateValue()
```cpp
void updateValue(uint8_t pageID, uint8_t componentID, uint16_t value)
```
**Parameters:**
- `pageID` - Page number (uint8_t)
- `componentID` - Component ID (uint8_t)
- `value` - Numeric value to display (uint16_t)

**Returns:** void

**Example:**
```cpp
hmi.updateValue(0, 1, 2500); // Page 0, Component 1, Value 2500
```

### updateText()
```cpp
void updateText(uint8_t pageID, uint8_t componentID, const char* text)
void updateText(uint8_t pageID, uint8_t componentID, String text)
```
**Parameters:**
- `pageID` - Page number (uint8_t)
- `componentID` - Component ID (uint8_t)
- `text` - Text to display (const char* or String)

**Returns:** void

**Example:**
```cpp
hmi.updateText(0, 2, "Temperature: 25.5C");
hmi.updateText(0, 2, String(temp, 1) + "C");
```

### updateProgressBar()
```cpp
void updateProgressBar(uint8_t pageID, uint8_t componentID, uint8_t percent)
```
**Parameters:**
- `pageID` - Page number (uint8_t)
- `componentID` - Progress bar component ID (uint8_t)
- `percent` - Progress percentage (uint8_t, 0-100)

**Returns:** void

**Example:**
```cpp
uint8_t battery = 75;
hmi.updateProgressBar(0, 3, battery);
```

---

## Navigation Functions

### changePage()
```cpp
void changePage(uint8_t pageID)
```
**Parameters:**
- `pageID` - Target page number (uint8_t)

**Returns:** void

**Example:**
```cpp
hmi.changePage(1); // Go to page 1
```

---

# 8. RajivEEPROM

**EEPROM Storage Library with Wear Leveling**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivEEPROM

## Constructor

```cpp
RajivEEPROM()
```
**Returns:** Object instance

---

## Initialization

### begin()
```cpp
void begin()
```
**Parameters:** None

**Returns:** void

**Example:**
```cpp
RajivEEPROM config;
config.begin();
```

---

## Single Byte Functions

### write()
```cpp
void write(uint16_t address, uint8_t value)
```
**Parameters:**
- `address` - EEPROM address (uint16_t, 0-1023 for ATmega328)
- `value` - Byte to write (uint8_t)

**Returns:** void

**Example:**
```cpp
config.write(0, 0xA7); // Write magic byte
config.write(1, nodeID);
```

### read()
```cpp
uint8_t read(uint16_t address)
```
**Parameters:**
- `address` - EEPROM address (uint16_t)

**Returns:** uint8_t - Byte value

**Example:**
```cpp
uint8_t nodeID = config.read(1);
```

---

## Block Functions

### writeBlock()
```cpp
void writeBlock(uint16_t address, uint8_t* data, uint16_t length)
```
**Parameters:**
- `address` - Starting address (uint16_t)
- `data` - Data buffer pointer (uint8_t*)
- `length` - Number of bytes (uint16_t)

**Returns:** void

**Example:**
```cpp
struct Settings {
    uint8_t nodeID;
    uint8_t networkID;
    char name[16];
};

Settings settings = {10, 100, "Door-01"};
config.writeBlock(0, (uint8_t*)&settings, sizeof(settings));
```

### readBlock()
```cpp
void readBlock(uint16_t address, uint8_t* buffer, uint16_t length)
```
**Parameters:**
- `address` - Starting address (uint16_t)
- `buffer` - Buffer to store data (uint8_t*)
- `length` - Number of bytes (uint16_t)

**Returns:** void

**Example:**
```cpp
Settings settings;
config.readBlock(0, (uint8_t*)&settings, sizeof(settings));
Serial.println(settings.nodeID);
```

---

## Validation Functions

### isValid()
```cpp
bool isValid(uint16_t address, uint8_t magicByte)
```
**Parameters:**
- `address` - Address of magic byte (uint16_t)
- `magicByte` - Expected magic byte value (uint8_t)

**Returns:** bool - true if magic byte matches

**Example:**
```cpp
if (!config.isValid(0, 0xA7)) {
    // Write default configuration
    writeDefaults();
}
```

---

## Maintenance Functions

### clear()
```cpp
void clear(uint16_t start, uint16_t length)
```
**Parameters:**
- `start` - Starting address (uint16_t)
- `length` - Number of bytes to clear (uint16_t)

**Returns:** void

**Example:**
```cpp
config.clear(0, 100); // Clear first 100 bytes
```

---

# 9. RajivWatchDog

**Hardware Watchdog Timer Management Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivWatchDog

## Constructor

```cpp
RajivWatchDog()
```
**Returns:** Object instance

---

## Initialization

### begin()
```cpp
void begin(WatchdogTimeout timeout)
```
**Parameters:**
- `timeout` - Timeout period (WatchdogTimeout enum)

**Timeout Options:**
- `WDT_15MS` - 15 milliseconds
- `WDT_30MS` - 30 milliseconds
- `WDT_60MS` - 60 milliseconds
- `WDT_120MS` - 120 milliseconds
- `WDT_250MS` - 250 milliseconds
- `WDT_500MS` - 500 milliseconds
- `WDT_1S` - 1 second
- `WDT_2S` - 2 seconds
- `WDT_4S` - 4 seconds
- `WDT_8S` - 8 seconds

**Returns:** void

**Example:**
```cpp
RajivWatchDog wdt;
wdt.begin(WDT_8S); // 8-second watchdog
```

---

## Control Functions

### reset()
```cpp
void reset()
```
**Parameters:** None

**Returns:** void

**Purpose:** Reset watchdog timer ("I'm alive" signal)

**Example:**
```cpp
void loop() {
    wdt.reset(); // Must call regularly!
    // Do work...
}
```

### disable()
```cpp
void disable()
```
**Parameters:** None

**Returns:** void

**Purpose:** Disable watchdog timer

---

## Status Functions

### wasReset()
```cpp
bool wasReset()
```
**Parameters:** None

**Returns:** bool - true if last reset was caused by watchdog

**Example:**
```cpp
void setup() {
    if (wdt.wasReset()) {
        Serial.println("System recovered from hang!");
    }
}
```

---

## Manual Reset Function

### forceReset()
```cpp
void forceReset()
```
**Parameters:** None

**Returns:** void (never returns - triggers system reset)

**Purpose:** Trigger immediate system reset

**Example:**
```cpp
if (fatalError) {
    wdt.forceReset(); // Reboot system
}
```

---

# 10. RajivDigitalOutput

**Base Class for Digital Output Control**

Used by: RajivRelay, RajivValveManager

---

## Constructor

```cpp
RajivDigitalOutput(uint8_t pid, uint8_t cid, const char* name)
```
**Parameters:**
- `pid` - Parent ID (uint8_t)
- `cid` - Component ID (uint8_t)
- `name` - Output name (const char*)

**Returns:** Object instance

---

## Control Functions

### enable()
```cpp
void enable()
```
**Parameters:** None

**Returns:** void

### disable()
```cpp
void disable()
```
**Parameters:** None

**Returns:** void

### isEnabled()
```cpp
bool isEnabled()
```
**Parameters:** None

**Returns:** bool

### activate()
```cpp
void activate()
```
**Parameters:** None

**Returns:** void

### deactivate()
```cpp
void deactivate()
```
**Parameters:** None

**Returns:** void

### toggle()
```cpp
void toggle()
```
**Parameters:** None

**Returns:** void

### setState()
```cpp
void setState(bool state)
```
**Parameters:**
- `state` - ON/OFF (bool)

**Returns:** void

### getState()
```cpp
bool getState()
```
**Parameters:** None

**Returns:** bool

---

## LED Functions

### setLEDState()
```cpp
void setLEDState(bool state)
```
**Parameters:**
- `state` - LED ON/OFF (bool)

**Returns:** void

### toggleLED()
```cpp
void toggleLED()
```
**Parameters:** None

**Returns:** void

### setLEDFollowSensor()
```cpp
void setLEDFollowSensor(bool follow)
```
**Parameters:**
- `follow` - LED follows state (bool)

**Returns:** void

---

## HMI Functions

### updateHMI()
```cpp
void updateHMI()
```
**Parameters:** None

**Returns:** void

---

## Statistics Functions

### getOnTime()
```cpp
unsigned long getOnTime()
```
**Parameters:** None

**Returns:** unsigned long - Current ON time (ms)

### getOffTime()
```cpp
unsigned long getOffTime()
```
**Parameters:** None

**Returns:** unsigned long - Current OFF time (ms)

### getTotalOnTime()
```cpp
unsigned long getTotalOnTime()
```
**Parameters:** None

**Returns:** unsigned long - Total ON time (ms)

### getTotalOffTime()
```cpp
unsigned long getTotalOffTime()
```
**Parameters:** None

**Returns:** unsigned long - Total OFF time (ms)

### getCycleCount()
```cpp
uint32_t getCycleCount()
```
**Parameters:** None

**Returns:** uint32_t - Number of cycles

### resetStatistics()
```cpp
void resetStatistics()
```
**Parameters:** None

**Returns:** void

---

# 11-14. RajivTimerOne/Three/Four/Five

**Hardware Timer Libraries for Arduino**

Repositories:
- https://github.com/rajiv8510-arch/Arduino-Library-RajivTimerOne
- https://github.com/rajiv8510-arch/Arduino-Library-RajivTimerThree
- https://github.com/rajiv8510-arch/Arduino-Library-RajivTimerFour
- https://github.com/rajiv8510-arch/Arduino-Library-RajivTimerFive

**Note:** All four timer libraries have identical APIs. Use Timer1 on Uno/Mega, Timer3/4/5 only on Mega.

---

## Global Object

```cpp
Timer1  // Pre-instantiated global object
Timer3  // Arduino Mega only
Timer4  // Arduino Mega only
Timer5  // Arduino Mega only
```

---

## Initialization

### initialize()
```cpp
void initialize(unsigned long microseconds)
```
**Parameters:**
- `microseconds` - Timer period in microseconds (unsigned long)

**Returns:** void

**Example:**
```cpp
Timer1.initialize(500000); // 500ms = 0.5 second
```

---

## Control Functions

### start()
```cpp
void start()
```
**Parameters:** None

**Returns:** void

### stop()
```cpp
void stop()
```
**Parameters:** None

**Returns:** void

### restart()
```cpp
void restart()
```
**Parameters:** None

**Returns:** void

### resume()
```cpp
void resume()
```
**Parameters:** None

**Returns:** void

---

## Interrupt Functions

### attachInterrupt()
```cpp
void attachInterrupt(void (*isr)())
```
**Parameters:**
- `isr` - Interrupt service routine function pointer (void (*)())

**Returns:** void

**Example:**
```cpp
void timerCallback() {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

Timer1.initialize(1000000); // 1 second
Timer1.attachInterrupt(timerCallback);
```

### detachInterrupt()
```cpp
void detachInterrupt()
```
**Parameters:** None

**Returns:** void

---

## PWM Functions

### pwm()
```cpp
void pwm(uint8_t pin, uint16_t duty)
```
**Parameters:**
- `pin` - Arduino pin number (uint8_t)
- `duty` - Duty cycle (uint16_t, 0-1023)

**Returns:** void

**Example:**
```cpp
Timer1.pwm(9, 512); // 50% duty cycle on pin 9
```

### disablePwm()
```cpp
void disablePwm(uint8_t pin)
```
**Parameters:**
- `pin` - Arduino pin number (uint8_t)

**Returns:** void

---

# 15. RajivDCMotor

**DC Motor Control Library with PWM Speed Control**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivDCMotor

## Constructor

```cpp
RajivDCMotor(uint8_t pwmPin, uint8_t dirPin)
```
**Parameters:**
- `pwmPin` - PWM control pin (uint8_t)
- `dirPin` - Direction control pin (uint8_t)

**Returns:** Object instance

---

## Control Functions

### setSpeed()
```cpp
void setSpeed(uint8_t speed)
```
**Parameters:**
- `speed` - Motor speed (uint8_t, 0-255)

**Returns:** void

**Example:**
```cpp
motor.setSpeed(200); // ~78% speed
```

### forward()
```cpp
void forward()
```
**Parameters:** None

**Returns:** void

### reverse()
```cpp
void reverse()
```
**Parameters:** None

**Returns:** void

### stop()
```cpp
void stop()
```
**Parameters:** None

**Returns:** void

### brake()
```cpp
void brake()
```
**Parameters:** None

**Returns:** void

---

# 16. RajivTimeredStepper

**Timer-Based Stepper Motor Control Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivTimeredStepper

## Constructor

```cpp
RajivTimeredStepper(uint16_t stepsPerRevolution, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
```
**Parameters:**
- `stepsPerRevolution` - Steps per revolution (uint16_t, e.g., 200 for 1.8°)
- `pin1`, `pin2`, `pin3`, `pin4` - Coil pins (uint8_t)

**Returns:** Object instance

---

## Control Functions

### setSpeed()
```cpp
void setSpeed(uint16_t rpm)
```
**Parameters:**
- `rpm` - Speed in RPM (uint16_t)

**Returns:** void

### step()
```cpp
void step(int steps)
```
**Parameters:**
- `steps` - Number of steps (int, negative for reverse)

**Returns:** void

### moveTo()
```cpp
void moveTo(long position)
```
**Parameters:**
- `position` - Target position (long)

**Returns:** void

### run()
```cpp
void run()
```
**Parameters:** None

**Returns:** void

### isRunning()
```cpp
bool isRunning()
```
**Parameters:** None

**Returns:** bool

---

# 17. RajivValveManager

**Valve Control and Management Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivValveManager

## Constructor

```cpp
RajivValveManager(uint8_t numValves)
```
**Parameters:**
- `numValves` - Number of valves to manage (uint8_t)

**Returns:** Object instance

---

## Control Functions

### openValve()
```cpp
void openValve(uint8_t valveID)
```
**Parameters:**
- `valveID` - Valve identifier (uint8_t)

**Returns:** void

### closeValve()
```cpp
void closeValve(uint8_t valveID)
```
**Parameters:**
- `valveID` - Valve identifier (uint8_t)

**Returns:** void

### setValvePosition()
```cpp
void setValvePosition(uint8_t valveID, uint8_t position)
```
**Parameters:**
- `valveID` - Valve identifier (uint8_t)
- `position` - Position percentage (uint8_t, 0-100)

**Returns:** void

### isValveOpen()
```cpp
bool isValveOpen(uint8_t valveID)
```
**Parameters:**
- `valveID` - Valve identifier (uint8_t)

**Returns:** bool

---

# 18. RajivSensorAnalog

**Analog Sensor Library with Filtering and Calibration**

## Constructor

```cpp
RajivSensorAnalog()
```
**Returns:** Object instance

---

## Initialization

### begin()
```cpp
void begin(uint8_t pin, float vRef)
```
**Parameters:**
- `pin` - ADC pin (uint8_t)
- `vRef` - Reference voltage (float, e.g., 5.0 or 1.1)

**Returns:** void

### setSampleCount()
```cpp
void setSampleCount(uint8_t samples)
```
**Parameters:**
- `samples` - Number of samples to average (uint8_t)

**Returns:** void

### setFilter()
```cpp
void setFilter(FilterType type)
```
**Parameters:**
- `type` - Filter type (FilterType enum)
  - `FILTER_NONE`
  - `FILTER_AVERAGE`
  - `FILTER_MEDIAN`
  - `FILTER_EMA`

**Returns:** void

---

## Temperature Sensor Functions

### initTemperature()
```cpp
void initTemperature(TempSensorType type)
```
**Parameters:**
- `type` - Temperature sensor type (TempSensorType enum)
  - `TEMP_LM35` - LM35 (10mV/°C)
  - `TEMP_TMP36` - TMP36
  - `TEMP_THERMISTOR` - NTC thermistor
  - `TEMP_CUSTOM`

**Returns:** void

### readTemperature()
```cpp
float readTemperature()
```
**Parameters:** None

**Returns:** float - Temperature in °C

### readTemperatureAlarm()
```cpp
bool readTemperatureAlarm(float minTemp, float maxTemp)
```
**Parameters:**
- `minTemp` - Minimum temperature (float)
- `maxTemp` - Maximum temperature (float)

**Returns:** bool - true if out of range

---

## Light Sensor Functions

### initLDR()
```cpp
void initLDR(uint32_t darkResistance, uint32_t pulldownR)
```
**Parameters:**
- `darkResistance` - LDR resistance in dark (uint32_t, ohms)
- `pulldownR` - Pulldown resistor value (uint32_t, ohms)

**Returns:** void

### readLightLevel()
```cpp
uint16_t readLightLevel()
```
**Parameters:** None

**Returns:** uint16_t - Light level (0-1023)

### isDark()
```cpp
bool isDark()
```
**Parameters:** None

**Returns:** bool

### isBright()
```cpp
bool isBright()
```
**Parameters:** None

**Returns:** bool

### setDarkThreshold()
```cpp
void setDarkThreshold(uint16_t value)
```
**Parameters:**
- `value` - Dark threshold (uint16_t)

**Returns:** void

### setBrightThreshold()
```cpp
void setBrightThreshold(uint16_t value)
```
**Parameters:**
- `value` - Bright threshold (uint16_t)

**Returns:** void

---

## Raw Reading Functions

### read()
```cpp
uint16_t read()
```
**Parameters:** None

**Returns:** uint16_t - Raw ADC value (0-1023)

### readVoltage()
```cpp
float readVoltage()
```
**Parameters:** None

**Returns:** float - Voltage in volts

### readFiltered()
```cpp
float readFiltered()
```
**Parameters:** None

**Returns:** float - Filtered reading

---

## Calibration Functions

### calibrate()
```cpp
void calibrate(float knownVoltage)
```
**Parameters:**
- `knownVoltage` - Known reference voltage (float)

**Returns:** void

### setOffset()
```cpp
void setOffset(int16_t offset)
```
**Parameters:**
- `offset` - Offset value (int16_t)

**Returns:** void

### setScale()
```cpp
void setScale(float scale)
```
**Parameters:**
- `scale` - Scale factor (float)

**Returns:** void

---

# 19. RajivPumpNModeConfig

**Pump Configuration and Mode Management Library**

Repository: https://github.com/rajiv8510-arch/Arduino-Library-RajivPumpNModeConfig

## Constructor

```cpp
RajivPumpNModeConfig()
```
**Returns:** Object instance

---

## Mode Functions

### setMode()
```cpp
void setMode(PumpMode mode)
```
**Parameters:**
- `mode` - Operating mode (PumpMode enum)
  - `MODE_MANUAL`
  - `MODE_AUTO`
  - `MODE_SCHEDULED`
  - `MODE_PRESSURE_CONTROLLED`

**Returns:** void

### getMode()
```cpp
PumpMode getMode()
```
**Parameters:** None

**Returns:** PumpMode - Current mode

---

# 20. rs485

**Industrial RS-485 / Modbus RTU Communication Library**

## Constructor

```cpp
rs485(HardwareSerial& serial, uint8_t txEnablePin)
```
**Parameters:**
- `serial` - Serial port reference (HardwareSerial&)
- `txEnablePin` - TX enable pin (uint8_t)

**Returns:** Object instance

---

## Initialization

### begin()
```cpp
void begin(uint32_t baudRate)
```
**Parameters:**
- `baudRate` - Baud rate (uint32_t)

**Returns:** void

---

## Communication Functions

### write()
```cpp
void write(uint8_t* data, uint16_t length)
```
**Parameters:**
- `data` - Data buffer (uint8_t*)
- `length` - Data length (uint16_t)

**Returns:** void

### read()
```cpp
uint16_t read(uint8_t* buffer, uint16_t maxLength)
```
**Parameters:**
- `buffer` - Receive buffer (uint8_t*)
- `maxLength` - Max bytes to read (uint16_t)

**Returns:** uint16_t - Bytes read

### available()
```cpp
bool available()
```
**Parameters:** None

**Returns:** bool

### flush()
```cpp
void flush()
```
**Parameters:** None

**Returns:** void

---

## Modbus Functions

### calculateCRC()
```cpp
uint16_t calculateCRC(uint8_t* data, uint16_t length)
```
**Parameters:**
- `data` - Data buffer (uint8_t*)
- `length` - Data length (uint16_t)

**Returns:** uint16_t - CRC16 value

---

## 📊 Multi-Library Integration Example

```cpp
#include <RajivSensorDI.h>
#include <RajivRFM69.h>
#include <RajivPowerManager.h>
#include <RajivWatchDog.h>
#include <RajivEEPROM.h>

RajivSensorDI doorSensor(1, 1, "door");
RajivRFM69 radio(10, 2, true);
RajivPowerManager power;
RajivWatchDog wdt;
RajivEEPROM config;

void setup() {
    wdt.begin(WDT_8S);
    
    config.begin();
    uint8_t nodeID = config.read(1);
    
    doorSensor.initialize(4, SENSOR_NO, 13);
    
    power.begin(A0, 1.1);
    power.setBatteryType(BATTERY_CR2450_2X);
    
    radio.begin(nodeID, 100, 1);
    radio.setTxPower(20);
    radio.enableEncryption("MySecretKey12345");
}

void loop() {
    wdt.reset();
    
    if (doorSensor.hasChanged()) {
        radio.wake();
        radio.sendAlarm(
            doorSensor.read() ? EVENT_DOOR_OPEN : EVENT_DOOR_CLOSE,
            power.readBatteryVoltage()
        );
        radio.sleep();
    }
    
    power.enterDeepSleep(8);
}
```

---

## 📦 Installation

**Arduino Library Manager:**
1. Arduino IDE → Sketch → Include Library → Manage Libraries
2. Search for library name
3. Click Install

**Manual:**
```bash
cd ~/Documents/Arduino/libraries/
git clone https://github.com/rajiv8510-arch/Arduino-Library-[Name].git
```

---

## 🔗 Repository Links

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

## 📝 Version History

**v2.0 (January 9, 2026)**
- Complete function-level documentation
- All parameters and return types documented
- Production-ready examples
- Multi-library integration guides

---

## 📧 Support

- **GitHub Issues**: Create in individual repositories
- **Author**: Rajiv Yadav ([@rajiv8510-arch](https://github.com/rajiv8510-arch))

---

## 📄 License

MIT License - Free to use, modify, and distribute

---

**⭐ This is the COMPLETE master documentation for all 21 Rajiv Arduino libraries**

*Every function documented with parameters, return types, and examples*  
*Last updated: January 9, 2026*
