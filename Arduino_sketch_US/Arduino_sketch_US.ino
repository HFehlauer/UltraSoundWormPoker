/*
 * Sketch to control the pins of Arduino via serial interface
 *
 * Commands implemented with examples:
 *
 * - RD13 -> Reads the Digital input at pin 13
 * - RA4 - > Reads the Analog input at pin 4
 * - WD13:1 -> Writes 1 (HIGH) to digital output pin 13
 * - WA6:125 -> Writes 125 to analog output pin 6 (PWM)
 */

char operation; // Holds operation (R, W, F, M)
char mode; // Holds the mode (D, A)
int pin_number; // Holds the pin number
int digital_value;
int analog_value;
long timeOn; // On time of the US pulse
long timeOff; // Off time of the US pulse
int value_to_write; // Holds the value that we want to write
int wait_for_transmission = 5; // Delay in ms in order to receive the serial data

void set_pin_mode(int pin_number, char mode){
    /*
     * Performs a pinMode() operation depending on the value of the parameter
     * mode :
     * - I: Sets the mode to INPUT
     * - O: Sets the mode to OUTPUT
     * - P: Sets the mode to INPUT_PULLUP
     */

    switch (mode){
        case 'I':
            pinMode(pin_number, INPUT);
            break;
        case 'O':
            pinMode(pin_number, OUTPUT);
            break;
        case 'P':
            pinMode(pin_number, INPUT_PULLUP);
            break;
    }
}

void digital_write(int pin_number, int digital_value){
    /*
     * Performs a digital write on pin_number with the digital_value
     * The value must be 1 or 0
     */
  digitalWrite(pin_number, digital_value);
}

void analog_write(int pin_number, int analog_value){
    /*
   * Performs an analog write on pin_number with the analog_value
   * The value must be range from 0 to 255
     */
  analogWrite(pin_number, analog_value);
}

void setup() {
    Serial.begin(9600); // Serial Port at 9600 baud
    Serial.setTimeout(100); // Timeout for the Serial.parseInt() 
}

void loop() {
    if (Serial.available() > 0) { // Check if characters available in the buffer
        operation = Serial.read();
        delay(wait_for_transmission); // If not delayed, second character is not correctly read
        mode = Serial.read();
        pin_number = Serial.parseInt(); // Waits for an int to be transmitted
        if (Serial.read()==':'){
            value_to_write = Serial.parseInt(); // Collects the value to be written
            if (Serial.read()=='/'){
              timeOn = Serial.parseInt(); // Collects the pulse length 
              if (Serial.read()=='-'){
                 timeOff = Serial.parseInt(); // Collects the pulse length 
              }            
            }        
        }
        switch (operation){
            case 'W': // Write operation, e.g. WD3:1, WA8:255
                if (mode == 'D'){ // Digital write
                    digital_write(pin_number, (int)(value_to_write));
                } else if (mode == 'A'){ // Analog write
                    analog_write(pin_number, value_to_write);
                } else {
                    break; // Unexpected mode
                }
                break;
            case 'F':// Frequency
                if (mode == 'D'){ // Digital write
                  for (int i=0; i<= (int)(value_to_write-1); i++){
                      digital_write(pin_number, 1);
                      delayMicroseconds(timeOn%1000);
                      delay(timeOn/1000);
                      digital_write(pin_number, 0);
                      delayMicroseconds(timeOff%1000); 
                      delay(timeOff/1000);                                                                    
                  }
                } else if (mode == 'A'){ // Analog write
                    analog_write(pin_number, value_to_write);
                } else {
                    break; // Unexpected mode
                }
                break;
            case 'M': // Pin mode, e.g. MI3, MO3, MP3
                set_pin_mode(pin_number, mode); // Mode contains I, O or P (INPUT, OUTPUT or PULLUP_INPUT)
                break;
            default: // Unexpected char
                break;
        }
    }
}
