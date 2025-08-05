##  Required Materials
* A CH340G for USB ↔ UART communication
* An LM7809 regulator
* A 16 MHz crystal
* Capacitors, resistors, LEDs, etc.
* A reset button and an ICSP connector

* Your homemade Arduino board (based on ATmega328P)
* A micro-USB or USB-A cable, depending on your CH340G
* A computer with the **Arduino IDE** installed
* The **CH340G drivers** installed
* Optional: an ISP programmer if the bootloader is not yet installed

##  Step 1 — Checking the Essential Components

Before uploading, make sure these components are present and properly connected:

| Component | Function |
| ----------------------------------------- | ----------------------------------------- |
| **ATmega328P** | Main Microcontroller |
| **CH340G** | USB-to-Serial Converter |
| **16 MHz Quartz + 2 x 22pF Capacitors** | Microcontroller Clock |
| **LM7809 Regulator + Filter Capacitors** | 9V to 5V Power Supply via Regulator |
| **LED on Pin 13 (Optional)** | Blink Code Verification |
| **Reset Button + 10k Pull-up Resistor** | Manually Rebooting the Microcontroller |

##  Step 2 — Connecting to the PC

1. Connect your homemade board via the USB cable.
2. If the **CH340G** is properly soldered and powered, an LED should light up (often a Power LED). 3. Check that the serial port appears on your PC:

* **Windows**: Device Manager → COM Ports
* **Linux**: `ls /dev/ttyUSB*`
* **macOS**: `ls /dev/cu.wchusb*`

##  Step 3 — Installing the CH340G driver

If your card is not recognized:

* Download and install the **CH340G drivers**:

* [WCH Official Website](https://sparks.gogo.co.nz/ch340.html)

##  Step 4 — Burn the bootloader (if you haven't already)

>  This should only be done once, unless the microcontroller is blank or incorrectly programmed.

1. Connect an **official Arduino** as an ISP programmer or a USBasp programmer to your board. 2. Branch:

* MOSI → MOSI
* MISO → MISO
* SCK → SCK
* RESET → RESET
* VCC → VCC
* GND → GND
3. Open the Arduino IDE:

* Tools > Board Type: **Arduino Uno**
* Tools > Programmer: **Arduino as ISP** or **USBasp**
* Tools > **Burn Initialization Sequence**

##  Step 5 — Upload Code (like Blink)

1. Open the **Arduino IDE**
2. Select:

* **Board**: Arduino Uno
* **Port**: The one detected for your board (e.g., COM3 or /dev/ttyUSB0)
3. Load a simple sketch, for example, `Blink`:

```cpp
void setup()
{
    pinMode(13, OUTPUT);
}
void loop()
{
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
}
```
4. Click **Upload**

##  Error Tips

| Error Message | Probable Cause | Solution |
| ------------------------------ | ------------------------------------------------- | -------------------------------------------- |
| `stk500_recv()` | Bootloader missing or wrong port selected | Burn the bootloader / choose the correct port |
| `avrdude: not in sync` | CH340G not properly connected or faulty RESET | Check connections, DTR/RESET signal |
| `programmer is not responding` | Problem with CH340G or power supply | Check 5V and GND, crystal present |

##  Example pinout for CH340G

| CH340G Pin | Connected to |
| ---------- | ----------------------------- |
| TXD | RX (ATmega328P pin 2) |
| RXD | TX (ATmega328P pin 3) |
| DTR | 100nF capacitor to RESET |
| VCC | 5V |
| GND | GND |

##  Verify proper operation

* The LED connected to pin 13 should blink if the `Blink` code has been uploaded.
* You can also use the serial console to display messages.
