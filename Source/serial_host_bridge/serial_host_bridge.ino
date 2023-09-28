/* This example demonstrates use of both device and host, where
   - Device run on native usb controller (roothub port0)
   - Host depending on MCUs run on either:
     - rp2040: bit-banging 2 GPIOs with the help of Pico-PIO-USB library (roothub port1)

   Requirements:
   - For rp2040:
     - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
     - 2 consecutive GPIOs: D+ is defined by PIN_USB_HOST_DP, D- = D+ +1
     - Provide VBus (5v) and GND for peripheral
     - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
*/

/* This example demonstrates use of Host Serial (CDC). SerialHost (declared below) is
   an object to manage an CDC peripheral connected to our USB Host connector. This example
   will forward all characters from Serial to SerialHost and vice versa.
*/

// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"

#include <Wire.h>

#define UARTRX 1
#define UARTTX 0
#define I2CSDA 2
#define I2CSCL 3
#define resetPin 4

void updateUARTBaud();
void updateUSBBaud();
void restartMCU();
void sendCurrentUARTBaud();
void sendCurrentUSBBaud();
void checkUSBConnection();
void I2C_TxHandler();
void I2C_RxHandler(int RXnum);

void forward_serial(void);

int uartSerial = 9;
int usbSerial = 9;

unsigned long baudRates[] = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200, 230400, 250000, 500000, 1000000, 2000000};

volatile uint8_t commandBuffer[16];
volatile uint8_t commandRequest = 0;

bool USBStatus = false;

void I2C_RxHandler(int RXnum) {
  uint8_t i = 0;

  while (0 < Wire.available()) { // Read Any Received Data
    commandBuffer[i] = Wire.read();

    i++;
  }

  if (commandBuffer[0] == 101) {
    updateUARTBaud();
  } else if (commandBuffer[0] == 102) {
    updateUSBBaud();
  } else if (commandBuffer[0] == 103) {
    restartMCU();
  } else if (commandBuffer[0] == 104) {
    sendCurrentUARTBaud();
  } else if (commandBuffer[0] == 105) {
    sendCurrentUSBBaud();
  } else if (commandBuffer[0] == 106) {
    checkUSBConnection();
  }

}

void I2C_TxHandler() {
  commandRequest = 1;
}

// CDC Host object
Adafruit_USBH_CDC SerialHost;

// forward Seral <-> SerialHost
void forward_serial(void) {
  uint8_t buf[64];

  // Serial -> SerialHost
  if (Serial1.available()) {
    size_t count = Serial1.readBytes(buf, sizeof(buf));
    if (SerialHost && SerialHost.connected()) {
      SerialHost.write(buf, count);
      SerialHost.flush();
    }
  }

  // SerialHost -> Serial
  if (SerialHost.connected() && SerialHost.available()) {
    size_t count = SerialHost.read(buf, sizeof(buf));
    Serial1.write(buf, count);
    Serial1.flush();
  }
}

//--------------------------------------------------------------------+
// For RP2040 use both core0 for device stack, core1 for host stack
//--------------------------------------------------------------------+

//------------- Core0 -------------//
void setup() {
  // Set up RX/TX for UART
  Serial1.setTX(UARTTX);
  Serial1.setRX(UARTRX);

  // Set up SDA/SCL for I2C Slave
  Wire.setSDA(I2CSDA);
  Wire.setSCL(I2CSCL);

  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);

  // Set up I2C Slave
  Wire.begin(0x55); // Initialize I2C (Slave Mode: address=0x55)
  Wire.onReceive(I2C_RxHandler);
  Wire.onRequest(I2C_TxHandler);

  Serial.begin(115200);
  Serial1.begin(baudRates[uartSerial]);
  
  Serial.println("TinyUSB Host Serial Echo Example");
}

void loop() {
  forward_serial();
}

//------------- Core1 -------------//
void setup1() {
  // configure pio-usb: defined in usbh_helper.h
  rp2040_configure_pio_usb();

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);

  // Initialize SerialHost
  SerialHost.begin(baudRates[usbSerial]);
}

void loop1() {
  USBHost.task();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
extern "C" {

  // Invoked when a device with CDC interface is mounted
  // idx is index of cdc interface in the internal pool.
  void tuh_cdc_mount_cb(uint8_t idx) {
    // bind SerialHost object to this interface index
    SerialHost.mount(idx);
    Serial.println("SerialHost is connected to a new CDC device");
    USBStatus = true;
  }

  // Invoked when a device with CDC interface is unmounted
  void tuh_cdc_umount_cb(uint8_t idx) {
    SerialHost.umount(idx);
    Serial.println("SerialHost is disconnected");
    USBStatus = false;
  }

}

void updateUARTBaud() {
  Serial1.flush();
  Serial1.end();

  delay(20);

  if (commandBuffer[1] < sizeof(baudRates)) {
    uartSerial = commandBuffer[1];
    Serial1.begin(baudRates[uartSerial]);

    Wire.write('t');
  } else {
    Wire.write('f');
  }
}

void updateUSBBaud() {
  SerialHost.flush();
  SerialHost.end();

  delay(20);

  if (commandBuffer[1] < sizeof(baudRates)) {
    usbSerial = commandBuffer[1];
    SerialHost.begin(baudRates[usbSerial]);

    Wire.write('t');
  } else {
    Wire.write('f');
  }
}

void restartMCU() {
  SerialHost.flush();
  SerialHost.end();
  Serial1.flush();
  Serial1.end();

  Wire.write('y');

  delay(500);

  digitalWrite(resetPin, LOW);
}

void sendCurrentUARTBaud() {
  Wire.write(uartSerial);
}

void sendCurrentUSBBaud() {
  Wire.write(usbSerial);
}

void checkUSBConnection() {
  if (USBStatus) {
    Wire.write('c');
  } else if (!USBStatus) {
    Wire.write('d');
  }
}
