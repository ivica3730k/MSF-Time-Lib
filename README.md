# MSF-Time-Lib

An Arduino library for receiving and decoding the **MSF Radio Time Signal** (broadcast from Anthorn, UK).

This library is designed to be robust against noise. Instead of looking for simple pulse edges, it uses continuous listening approach to statistically identify the unique "Minute Marker" signature (Carrier On -> Carrier Off transition) before attempting to decode the time data.

## Summary

* **Robust Syncing:** Continuously samples the signal and uses a confidence scoring system to identify the exact start of each minute and creates acquiring windows for data bit capture.
* **Parity Checking:** Validates year, date, day-of-week, and time segments individually against the MSF protocol.
* **Hardware Agnostic:** Works with any receiver module (Active High or Active Low) via a user-defined reader callback.
* **Dependency Free:** Returns a simple primitive struct (`MSFData`), allowing you to use the data however you like (e.g., with `TimeLib`, `RTC` libraries, or raw).

## MSF Specification

The url with PDF representing the specification can be found [here](https://www.pvelectronics.co.uk/rftime/msf/MSF_Time_Date_Code.pdf).

## How it works

### 1 Signal Synchronization

Unlike simple receivers that wait for a "long pulse," this library samples the signal continuously at a defined rate. It pushes these samples into a memory structure that represents the last 1.5 seconds of carrier history. It then calculates a "Confidence Score" by comparing the current carrier/no-carrier values against the ideal Minute Marker pattern:

* **Carrier Window:** 700ms of signal.
* **Silence Window:** 500ms of silence.

On every new sample we update the score, and once we have scanned through 65 seconds (to cover the full minute and potential drift), we identify the time of the peak score and we know that the minute marker is at that point.

### 2 Data Acquisition

Once synchronized, the library waits for the next full minute and then when that minute comes it calculates and waits for specific time windows within every next upcoming second (the **Bit A window** and **Bit B window**). It takes multiple samples during these windows to determine if the bit is Logic 1 or Logic 0.

### 3 Decoding & Validation

After collecting 60 seconds of data, it decodes the BCD (Binary Coded Decimal) values and verifies the checksum (parity bits) provided by the MSF signal.

## Usage

### 1. Define the Reader Function

You must provide a function that returns `true` when the **Carrier is Present** and `false` when there is **Silence**.

**Example for a standard Active Low module, where presence of carrier is indicated by the pin being LOW:**

```cpp
const int MSF_PIN = 2;

void setup() {
  pinMode(MSF_PIN, INPUT_PULLUP);
}

// Returns TRUE when Carrier is detected (Pin is LOW)
bool readMSFSignal() {
  // We invert the read because LOW means "Signal Present"
  return !digitalRead(MSF_PIN);
}
```

### 2. Initialization

Include the header and instantiate the class. You must define the `SAMPLE_RATE_MS` (how often the code reads the pin during the sync phase) as a template parameter. Lower values e.g `1ms` will give better second precision but will consume more memory.

```cpp
#include <MSF-Time-Lib.h>

// Initialize with a 2ms sample rate, passing your reader function
MSFReceiver<2> msf(readMSFSignal);
```

### 3. Reading Time

Use `get_time()` for a single attempt, or `get_time_with_retry()` to block until a valid signal is received.

```cpp

boolean readMSFSignal() {
  return !digitalRead(MSF_PIN); // Active Low: LOW means signal present
}

MSFReceiver<2> msf(readMSFSignal);


void setup() {
  Serial.begin(9600);
  pinMode(MSF_PIN, INPUT_PULLUP);

  // Blocks until a valid time is received (Parity checks passed)
  MSFData data = msf.get_time_with_retry();
  Serial.print("Time: ");
  Serial.print(data.hour);
  Serial.print(":");
  Serial.print(data.minute);
  Serial.print(" Date: ");
  Serial.print(data.day);
  Serial.print("/");
  Serial.println(data.month);
}
```

The `MSFData` struct contains:

* `year`, `month`, `day`, `hour`, `minute`, `dayOfTheWeek`
* `checksumPassed` (boolean indicating if the data is valid)

## Debugging

To see what the library is doing internally (Sync scores, signal strength, bit decoding), enable the debug flag before importing the library:

```cpp
#define MSF_TIME_LIB_DEBUG 1
#include <MSF-Time-Lib.h>
```

This will output detailed logs to the Serial Monitor:

```text
[MSF] Syncing... (Scanning 65s for Minute Marker)
[MSF] T+64.9s | Curr: 450 | Best: 1129
...
[MSF] Final Peak Score: 58
[MSF] Sync Complete. Aligning to next minute (Will wait for: 28122ms)...
[MSF] Sec 00 | A:1 [100%] | B:1 [100%]
[MSF] Sec 01 | A:0 [0%]   | B:1 [100%]
```

**Note:** Ensure you disable this flag (`0`) for production, as the serial printing overhead can affect timing sensitivity and make sure you use high baud rate so MSF read logic is not slowed down by serial communication.

## Examples

Please check the `examples/` folder in this repository for complete, ready-to-run sketches:

* **simple:** Simple sketch to fetch time and print it to Serial.
* **time_lib_integration:** Example of how to set the Arduino TimeLib library with the decoded MSF time.

## Currently out of scope for this library

### Handling of Leap Minute/second events

Currently on leap events the checksum will just fail and library will discard the data as invalid. If you use `get_time_with_retry()` it will just try again and you will get the correct time on the next capture event. Im not sure this will ever be implemented as leap events are very rare and the current behavior is to just ignore them which is probably good enough for most use cases.

### Platforms requiring yield()

There is some delay() blocks in the code which is not very friendly with Espressif platforms. Later these sleeps will be handled with while block, timers and yield() calls to make sure the code works reliability on those platforms.

### Non-blocking mode

This code currently occupies the main loop while syncing and acquiring data. While this could be done with interrupt pins or some scheduler library, this is not in scope of this library and will probably be a whole another library with revisited logic flow.

## How to install this library

### Navigate to the "Releses" section on github and download the latest release as a zip file

![alt text](image.png)

### In the Arduino IDE, go to "Sketch" -> "Include Library" -> "Add .ZIP Library..." and select the downloaded zip file

### Navigate to the "examples" folder in this repository and open the "simple.ino" sketch and compile it
