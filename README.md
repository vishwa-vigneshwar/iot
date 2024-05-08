# iot


### 1. Addition and Multiplication using Embedded C
```c
#include <stdio.h>

int main() {
    int a = 5, b = 3;
    int sum = a + b;
    int product = a * b;
    printf("Sum: %d\n", sum);
    printf("Product: %d\n", product);
    return 0;
}
```

### 2. Program to Transfer Data between Two Registers (Assuming 8051 Microcontroller)
```c
#include <reg51.h> // Including header for 8051

void main() {
    char data1 = 0xAA;  // Example data
    char data2;
    
    data2 = data1; // Transfer data from data1 to data2
    while(1); // Infinite loop to stop execution
}
```

### 3. Program to Transfer Data between Memory and Register (Assuming 8051 Microcontroller)
```c
#include <reg51.h>

void main() {
    char data = 0x55;
    char xdata *ptr = 0x1000; // External memory location

    *ptr = data; // Store data in memory
    data = *ptr; // Load data from memory

    while(1);
}
```

### 4. Smart City Application Based on IoT
For a broad topic like this, one could think of implementing various IoT-enabled services like smart lighting, smart waste management, etc. Below is a very simple conceptual outline using pseudocode:

```python
# Pseudocode for a smart lighting system
Initialize system
while True:
    if night_time() and motion_detected():
        turn_on_lights()
    if day_time() or no_motion_for_10_minutes():
        turn_off_lights()
Report system status
```

### 6. Subtraction and Division Operation using Embedded C
```c
#include <stdio.h>

int main() {
    int a = 10, b = 2;
    int difference = a - b;
    int quotient = a / b;
    printf("Difference: %d\n", difference);
    printf("Quotient: %d\n", quotient);
    return 0;
}
```

### 10. Arithmetic Operations in 8051 using Simulator
```c
#include <reg51.h>

void main() {
    unsigned char a = 12;
    unsigned char b = 4;
    unsigned char result;

    result = a + b; // Add
    result = a - b; // Subtract
    result = a * b; // Multiply (Use software multiplication for 8051)
    result = a / b; // Divide (Use software division for 8051)
    while(1);
}
```

### 11. Logical Operations using Embedded C
```c
#include <stdio.h>

int main() {
    unsigned char a = 0x0F; // Binary: 00001111
    unsigned char b = 0xF0; // Binary: 11110000
    unsigned char result;

    result = a & b; // AND
    printf("AND Result: %02x\n", result);

    result = a | b; // OR
    printf("OR Result: %02x\n", result);

    result = a ^ b; // XOR
    printf("XOR Result: %02x\n", result);

    result = ~a; // NOT
    printf("NOT Result: %02x\n", result);

    return 0;
}
```

### 16. Interfacing a Sensor with Raspberry Pi
Here’s an example of interfacing a DHT11 temperature and humidity sensor with Raspberry Pi using Python:

```python
import Adafruit_DHT

sensor = Adafruit_DHT.DHT11
pin = 4  # GPIO pin connected to the sensor

humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
if humidity is not None and temperature is not None:
    print(f"Temp: {temperature} C, Humidity: {humidity} %")
else:
    print("Failed to retrieve data from humidity sensor")
```

### 18. Arduino Platform and Programming
Arduino is an open-source electronics platform based on easy-to-use hardware and software. Arduinos boards are able to read inputs (like light on a sensor, a finger on a button, or a Twitter message) and turn it into an output (activating a motor, turning on an LED, publishing something online). Here’s an example program:

```cpp
void setup() {
  pinMode(13, OUTPUT); // Initialize digital pin 13 as an output.
}

void loop() {
  digitalWrite(13, HIGH);   // Turn the LED on
  delay(1000);              // Wait for a second
 

 digitalWrite(13, LOW);    // Turn the LED off
  delay(1000);              // Wait for a second
}
```

### 19. Python Programming for Data Transfer Operation in Arduino
For data transfer between a computer and an Arduino, you typically use the serial interface. Here’s how you might write a Python script to send data to an Arduino:

```python
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600)  # Open serial port at 9600 baud

def send_data(data):
    ser.write(data.encode())

while True:
    send_data("Hello Arduino!")
    time.sleep(1)  # Wait for a second

ser.close()  # Close serial port
```



### 1. Measure Human Body Temperature Using a Sensor with Raspberry Pi
You might use a DS18B20 temperature sensor which is accurate and easy to interface with a Raspberry Pi:

```python
import os
import glob
import time
import board
import adafruit_ds18b20

# Initialize the DS18B20 sensor
ds18b20 = adafruit_ds18b20.DS18B20(board.D4)

while True:
    print('Temperature: {:.2f} C'.format(ds18b20.temperature))
    time.sleep(1)
```

This script requires the Adafruit CircuitPython DS18B20 library.

### 4. Program to Transfer Data between Two Registers (Generic Example)
In embedded C (Pseudocode as no specific hardware context was provided):
```c
void main() {
    uint8_t regA = 10;
    uint8_t regB;

    regB = regA;  // Data transfer from regA to regB
}
```

### 5. Set the Platform to Store the Data
This could mean setting up a database. Here’s how you might initiate a simple file-based storage system in Python:
```python
# Open a file in append mode
with open('data_store.txt', 'a') as file:
    file.write("Data storage initialized\n")
```

### 7. Design an Automatic Irrigation System Using IoT Devices
Conceptual overview using pseudocode:
```plaintext
Initialize sensors (soil moisture, weather forecast)
Initialize actuator (water valve)
while true:
    if soil_moisture < threshold and forecast != 'rain':
        open_valve()
    else:
        close_valve()
Send status to central server
```

### 8. Swap Numbers in Register and Accumulator
In assembly language for a generic microcontroller:
```assembly
; Assume A and B are registers
MOV A, R1  ; Move content of R1 to A
XCH A, R2  ; Exchange content of A with R2
MOV R1, A  ; Move updated A back to R1
```

### 9. Divide Two Numbers Using Embedded C
```c
#include <stdio.h>

int main() {
    int dividend = 15, divisor = 3;
    if (divisor != 0) {
        int quotient = dividend / divisor;
        printf("Quotient is %d\n", quotient);
    } else {
        printf("Division by zero error.\n");
    }
    return 0;
}
```

### 10. Communication Between IoT and Bluetooth
A simple conceptual outline for sending data from an IoT device to a Bluetooth module:
```c
// Pseudocode
initialize_bluetooth();
while(1) {
    data = read_sensor();
    bluetooth_send(data);
}
```

### 11. Find Largest Number in a Group Using Simulator Tool
Using Embedded C for an array of integers:
```c
#include <stdio.h>

int findLargest(int arr[], int n) {
    int i;
    int max = arr[0];
    for (i = 1; i < n; i++)
        if (arr[i] > max)
            max = arr[i];
    return max;
}

int main() {
    int arr[] = {2, 8, 5, 3, 9};
    int n = sizeof(arr) / sizeof(arr[0]);
    printf("Largest number is %d\n", findLargest(arr, n));
    return 0;
}
```

### 12. Deliver Patient Condition to Doctor Using IoT Devices
Conceptual IoT-based system outline using pseudocode:
```plaintext
Initialize sensors (heart rate, temperature)
Initialize communication module (WiFi, cellular)
while true:
    data = read_sensors()
    if critical_conditions(data):
        send_data_to_doctor(data)
    sleep(interval)
```

### 15. Transfer Data between Memory and Accumulator
Assuming use of an assembly language for a microcontroller:
```assembly
; Assuming ACC is accumulator and MEM is memory location
MOV A, MEM  ; Move content of memory to accumulator
MOV MEM, A  ; Store content of accumulator back to memory
```

### 17. Add Two Numbers Using Embedded C
```c
#include <stdio.h>

int main() {
    int num1 = 10, num2 = 20;
    int sum = num1 + num2;
    printf("Sum = %d\n", sum);
    return 0;
}
```

### 18. Design a Smart City System Using IoT Devices
Conceptual IoT-based smart city system might involve:
```plaintext
Initialize systems (traffic management, pollution monitoring, waste management)
while true:
    data = collect_data_from_all_sensors()
    analyze

_data(data)
    adjust_city_services_based_on_data()
    report_status_to_city_dashboard()
```

