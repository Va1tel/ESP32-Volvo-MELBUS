# ESP32-Volvo-MELBUS
A repository for ESP 32 that emulates CD-CHG with music streaming via Bluetooth based on [Volvo-melbus](https://github.com/visualapproach/Volvo-melbus.git)

# Goals
  The following goals are included for this project: 
1) Creation of the ESP 32 interaction logic with the HU-850 (or other HU-XXX) via the MODBUS protocol 
2) The introduction of Bluetooth into the program, which is in the ESP 32, for wireless connection with the phone and music transmission
3) (for the future) Adding MD-CH support for displaying custom text

# Connection diagram
<img src="https://github.com/user-attachments/assets/e152a46b-3d08-407c-a8ba-4afd200b4505" width="550" height="450">


  The power is taken from the CD-CHG connector and lowered to 5V using a DC-DC converter B1205S-2W with galvanic isolation. The left and right channels are connected to the analog outputs of ESP 32 (GPIO 25 and 26). The audio ground is connected to the -5V module B1205S-2W. The pins responsible for data transmission are connected to the following ESP 32 pins:
  * DATA - GPIO 21
  * CLK - GPIO 19
  * BUSY - GPIO 18

# Quality of sound
Since the ESP 32 has mediocre sound quality, the PCM5102A module operating under the I2S protocol will be connected in the future to improve the sound quality.

# What's ready
* The logic of interaction with HU via MELBUS is ready;
* CD and track number display works; 
* Track switching works from the steering wheel buttons;
* It is possible to enable SAT (commands have been added for it, but the initialization procedure has been commented out, since I do not need this mode yet).

*Note: this all works in the absence of Bluetooth

# Problems
The main problem that blocks further development is the introduction of Bluetooth into the program. The [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP.git) library is used for Bluetooth.

With simple bluetooth initialization using:
```cpp
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

AnalogAudioStream out;
BluetoothA2DPSink a2dp_sink(out);

void setup() {
    a2dp_sink.start("MyMusic");
}
```
With such initialization, if it is done before the main program and a delay after it, HU sees the emulator, but as soon as the phone is connected to ESP 32, the main logic stops being executed.

Initialization on different cores:
```cpp
// Task of launching emulator CD-CHG
void mainTask(void *pvParameters) {
    loop(); //the basic logic of the emulator
}

// Main program
extern "C" void app_main(void)
{   
    // Create task of launching emulator CD-CHG on core 1
    xTaskCreatePinnedToCore(
        mainTask,  // Task function
        "MainTask", // Name function
        8192, // Stack size for the main program
        NULL,  // Parametrs of task
        10,    // Priority of the main task
        NULL, // Task handling
        1     // Running on core 0
    );

    a2dp_sink.start("SwedenMetal");
    while(true){
        vTaskDelay(100/ portTICK_PERIOD_MS);
    }

}
```
This initialization was done based on the assumption that bluetooth event handlers and internal processes use core 0, therefore, core 1 is used to separate the main program.
With this approach, HU manages to see the emulator, connect the phone to Bluetooth, and even switch tracks, but all this works until the music is playing. As soon as the music starts playing, the main logic of the emulator stops working - perhaps due to the fact that Bluetooth does not allow you to switch to the logic of the emulator.

# Warning
All actions that you do with your car, you do at your own risk. The author is not responsible for the breakage of your parts.
