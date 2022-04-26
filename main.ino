#include <MIDIUSB.h>
#include <PitchToNote.h>
#define NUM_BUTTONS  7
#include <tone.h>

// const uint8_t button1 = 2;
// const uint8_t button2 = 3;
// const uint8_t button3 = 4;
// const uint8_t button4 = 5;
// const uint8_t button5 = 6;
// const uint8_t button6 = 7;
// const uint8_t button7 = 8;

int LEDPin = 13;
// Pin to connect to your drawing
const uint8_t capSensePin1 = 2;
const uint8_t capSensePin2 = 3;
const uint8_t capSensePin3 = 4;
const uint8_t capSensePin4 = 5;
const uint8_t capSensePin5 = 6;
const uint8_t capSensePin6 = 7;
const uint8_t capSensePin7 = 8;

// Threshold
int touchedCutoff = 60;

// A0 input
const int intensityPot = 0; 

const uint8_t buttons[NUM_BUTTONS] = {capSensePin1, capSensePin2, capSensePin3, capSensePin4, capSensePin5, capSensePin6, capSensePin7};

const byte notePitches[NUM_BUTTONS] = {C3, D3, E3, F3, G3, A3, B3};

uint8_t notesTime[NUM_BUTTONS];

uint8_t pressedButtons = 0x00;

uint8_t previousButtons = 0x00;

uint8_t intensity;

void setup() {
    Serial.begin(9600);
    // Set up the LED
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, LOW);
    
    // for (int i = 0; i < NUM_BUTTONS; i++){
    //     pinMode(buttons[i], INPUT_PULLUP);
    // }
}

void loop() {

    // Every 500 ms, print the value of the capacitive sensor
//   if ( (millis() % 500) == 0){
//     Serial.print("Capacitive Sensor on Pin 2 reads: ");
//     Serial.println(readCapacitivePin(capSensePin));
//   }

  readButtons();

  readIntensity();

  playNotes();
}

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void readButtons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (readCapacitivePin(buttons[i]) > touchedCutoff) {
      bitWrite(pressedButtons, i, 1);
      delay(50);
    }
    else {
      bitWrite(pressedButtons, i, 0);
    }
  }
}

void readIntensity() {
  int val = analogRead(intensityPot);
  intensity = (uint8_t) (map(val, 0, 1023, 0, 127));
}

void playNotes() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (bitRead(pressedButtons, i) != bitRead(previousButtons, i)) {
      if (bitRead(pressedButtons, i)) {
        bitWrite(previousButtons, i , 1);
        noteOn(0, notePitches[i], intensity);
        MidiUSB.flush();
      }
      else {
        bitWrite(previousButtons, i , 0);
        noteOff(0, notePitches[i], 0);
        MidiUSB.flush();
      }
    }
  }
}

// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}


/* 
    This is a test
*/

// readCapacitivePin
//  Input: Arduino pin number
//  Output: A number, from 0 to 17 expressing
//          how much capacitance is on the pin
//  When you touch the pin, or whatever you have
//  attached to it, the number will get higher
//  In order for this to work now,
// The pin should have a 1+Megaohm resistor pulling
//  it up to +5v.
uint8_t readCapacitivePin(int pinToMeasure){
    // This is how you declare a variable which
    //  will hold the PORT, PIN, and DDR registers
    //  on an AVR
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t* pin;
    // Here we translate the input pin number from
    //  Arduino pin number to the AVR PORT, PIN, DDR,
    //  and which bit of those registers we care about.
    byte bitmask;
    if ((pinToMeasure >= 0) && (pinToMeasure <= 7)) {
        port = &PORTD;
        ddr = &DDRD;
        bitmask = 1 << pinToMeasure;
        pin = &PIND;
    }
    if ((pinToMeasure > 7) && (pinToMeasure <= 13)) {
        port = &PORTB;
        ddr = &DDRB;
        bitmask = 1 << (pinToMeasure - 8);
        pin = &PINB;
    }
    if ((pinToMeasure > 13) && (pinToMeasure <= 19)) {
        port = &PORTC;
        ddr = &DDRC;
        bitmask = 1 << (pinToMeasure - 13);
        pin = &PINC;
    }
    // Discharge the pin first by setting it low and output
    *port &= ~(bitmask);
    *ddr  |= bitmask;
    delay(1);
    // Make the pin an input WITHOUT the internal pull-up on
    *ddr &= ~(bitmask);
    // Now see how long the pin to get pulled up
    int cycles = 16000;
    for(int i = 0; i < cycles; i++) {
        if (*pin & bitmask){
            cycles = i;
            break;
        }
    }
    // Discharge the pin again by setting it low and output
    //  It's important to leave the pins low if you want to 
    //  be able to touch more than 1 sensor at a time - if
    //  the sensor is left pulled high, when you touch
    //  two sensors, your body will transfer the charge between
    //  sensors.
    *port &= ~(bitmask);
    *ddr  |= bitmask;

    return cycles;
}
