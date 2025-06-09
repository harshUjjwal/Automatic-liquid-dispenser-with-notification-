# Hardware Schematic

## Components List
1. Arduino Board (Uno/Nano/Mega)
2. IR Proximity Sensor
3. L293D Motor Driver
4. 16x2 I2C LCD Display
5. DHT11 Temperature & Humidity Sensor
6. DC Motor/Pump (5-12V)
7. Liquid Level Sensor (Optional)
8. Status LED
9. Power Supply (12V for motor, 5V for logic)
10. Connecting Wires

## Pin Connections

### IR Sensor
- VCC → Arduino 5V
- GND → Arduino GND
- OUT → Arduino D2

### L293D Motor Driver
- Pin 1 (Enable 1,2) → Arduino D5
- Pin 2 (Input 1) → Arduino D3
- Pin 3 (Output 1) → Motor +
- Pin 4, 5 (GND) → Arduino GND
- Pin 6 (Output 2) → Motor -
- Pin 7 (Input 2) → Arduino D4
- Pin 8 (VCC2) → 12V Power Supply
- Pin 16 (VCC1) → Arduino 5V

### I2C LCD Display
- VCC → Arduino 5V
- GND → Arduino GND
- SDA → Arduino A4
- SCL → Arduino A5

### DHT11 Sensor
- VCC → Arduino 5V
- GND → Arduino GND
- DATA → Arduino D6

### Liquid Level Sensor
- VCC → Arduino 5V
- GND → Arduino GND
- OUT → Arduino A0

### Status LED
- Anode → Arduino D13 (through 220Ω resistor)
- Cathode → Arduino GND

## Power Supply Connections
1. Connect 12V power supply to L293D VCC2 (Pin 8)
2. Connect power supply GND to Arduino GND
3. Arduino can be powered via USB or external 7-12V supply

## Notes
1. Use appropriate current limiting resistors for LED
2. Ensure common ground between Arduino and motor power supply
3. Add decoupling capacitors near L293D (100µF and 0.1µF)
4. Motor voltage should match power supply (typically 12V)
5. Verify I2C address of LCD display (default 0x27)

## Safety Considerations
1. Double-check all connections before powering up
2. Ensure proper ventilation for L293D
3. Use appropriate gauge wires for motor current
4. Add reverse polarity protection if needed
5. Consider adding a fuse for motor circuit 