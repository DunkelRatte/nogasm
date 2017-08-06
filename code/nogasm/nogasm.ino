#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"
#include "RunningAverage.h"


#define DEBUG false     // Display debug data on serial monitor
#define DEBUG_PLOT true // Display debug data on serial plotter

//=======Hardware Setup===============================
//Pressure Sensor Analog In
#define BUTTPIN 15 // Pressure sensor
#define MOTPIN 23 // Motor
//LEDs
#define NUM_LEDS 13
#define LED_PIN 17 //5V buffered pin on Teensy LC, single wire data out to WS8212Bs
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 10 //Subject to change, limits current that the LEDs draw
CRGB leds[13];
//Encoder
#define REDPIN   5 //RGB pins of the encoder LED
#define GREENPIN 4
#define BLUEPIN  3
#define ENC_SW   6 //Pushbutton on the encoder
Encoder encoder(8, 7); //Quadrature inputs on pins 7,8
//DIP Switches
#define SW1PIN 12 //Dip switch pins, for setting software options without reflashing code
#define SW2PIN 11
#define SW3PIN 10
#define SW4PIN 9


//Update/render period
#define period (1000/FREQUENCY)
#define FREQUENCY 60 //Update frequency in Hz

//Running pressure average array length and update frequency
#define RA_HIST_SECONDS 25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverage raPressure(RA_FREQUENCY*RA_HIST_SECONDS);

#define MOT_MAX_SPEED 256
#define MOT_MIN_SPEED 40

#define COOLDOWN_TIME 1000

////=======State Machine Modes=========================
#define State_MANUAL    0
#define State_AUTO      1
#define State_OPT_SPEED 2

#define ButtonMode_NONE  0
#define ButtonMode_SHORT 1
#define ButtonMode_LONG  2

#define ButtonState_UP   0
#define ButtonState_DOWN 1


int avgPressure = 0; //Running 25 second average pressure
float motSpeed = 0; //Motor speed, 0-255 (float to maintain smooth ramping to low speeds)
int sampleTick = 0;
float maximumDelta = 0.0;
int state = State_MANUAL;
int buttonMode = ButtonMode_NONE;

int lastButtonState = ButtonState_UP;
int buttonPressTimestamp = 0;
int cooldownUntil = 0;

int get_button_state() {
    return digitalRead(ENC_SW) ? ButtonState_DOWN : ButtonState_UP;
}

float get_pressure_percentage_of(float max) {

    int delta = (get_sensor() - avgPressure);
    if(delta == 0) { delta = 1;};
    return min(1.0, delta / max);
}

void update_button_mode() {
    if( ButtonState_DOWN == lastButtonState &&
        ButtonState_UP   == get_button_state()) { //  released

        lastButtonState = ButtonState_UP;
        int presstime = millis() - buttonPressTimestamp;

        buttonMode =  (presstime < 500) ? ButtonMode_SHORT : ButtonMode_LONG;
        return;
    }

    if( ButtonState_UP   == lastButtonState &&
        ButtonState_DOWN == get_button_state()) { // pressed down

        buttonPressTimestamp = millis();
        lastButtonState = ButtonState_DOWN;
    }
    
    buttonMode = ButtonMode_NONE; // nothing yet
}

void check_button() {
    if(buttonMode != ButtonMode_NONE) {
        switch (buttonMode) {
            case ButtonMode_SHORT:
                next_state();
                break;
            case ButtonMode_LONG:
                break;
        }

        buttonMode = ButtonMode_NONE;
    }
}

void next_state() {
    state += 1;
    state = state % 2;
}

void run_selected_mode() {
    switch (state) {
        case State_MANUAL:
            mode_manual();
            break;
        case State_AUTO:
            mode_auto();
            break;
    }
}

bool is_cooldown() {
    return millis() < cooldownUntil;
}

void mode_auto() {
    showKnobRGB(CRGB::Blue);

    float sensitivityStep = maximumDelta / 51;

    int delta = get_sensor() - avgPressure;
    int knob = get_encoder(0, 51);
    float stopDelta = max(1.0, maximumDelta - (knob * sensitivityStep));

    if(stopDelta < 1.0) {
        stopDelta = 1;
    }

    draw_bar(get_pressure_percentage_of(stopDelta));
    draw_multilevel_cursor(knob);

    if(delta > stopDelta) {
        cooldownUntil = millis() + COOLDOWN_TIME;
        motSpeed = 0;
    }

    if(!is_cooldown()) {
        float motIncrement = 0.1;
        motSpeed += motIncrement;
    }
}

void mode_manual() {
    showKnobRGB(CRGB::Purple);

    int knob = get_encoder(0, 51);
    motSpeed = (knob == 0) ? 0 : map(knob, 1, 51, MOT_MIN_SPEED, MOT_MAX_SPEED);
    draw_bar(get_pressure_percentage_of(maximumDelta));
    draw_multilevel_cursor(knob);

}
void showKnobRGB(const CRGB& rgb) {
    digitalWrite(REDPIN,   (rgb.r > 0 ? HIGH : LOW) );
    digitalWrite(GREENPIN, (rgb.g > 0 ? HIGH : LOW) );
    digitalWrite(BLUEPIN,  (rgb.b > 0 ? HIGH : LOW) );
}

void beep_motor(int f1, int f2, int f3){
    if(motSpeed > 245) analogWrite(MOTPIN, 245); //make sure the frequency is audible
    else if(motSpeed < 10) analogWrite(MOTPIN, 10);
    analogWriteFrequency(MOTPIN, f1);
    delay(250);
    analogWriteFrequency(MOTPIN, f2);
    delay(250);
    analogWriteFrequency(MOTPIN, f3);
    delay(250);
    analogWriteFrequency(MOTPIN, 440);
    analogWrite(MOTPIN,motSpeed);
}

void draw_bar(float percentage) {
    CRGB colors[] = {
            CRGB::Green,
            CRGB::Green,
            CRGB::Green,
            CRGB::Green,
            CRGB::Green,
            CRGB::Green,
            CRGB::Green,
            CRGB::Green,
            CRGB::Green,
            CRGB::Yellow,
            CRGB::Orange,
            CRGB::Red,
            CRGB::Red
    };

    for(int i = 0; i < floor(percentage * 13); i++) {
        leds[i] = colors[i];
    }
}

//Draw a "cursor", one pixel representing either a pressure or encoder position value
void draw_cursor(int pos,CRGB C1){
    pos = constrain(pos,0,NUM_LEDS-1);
    leds[pos] = C1;
}

void draw_multilevel_cursor(int pos) {
    CRGB colors[] = {CRGB::Blue, CRGB::Green, CRGB::Yellow, CRGB::Red};
    pos = min(pos, 52);
    int level = floor(pos / NUM_LEDS);
    int p = pos % (NUM_LEDS);
    draw_cursor(p, colors[level]);
}

//Draw 3 revolutions of bars around the LEDs. From 0-39, 3 colors
void draw_bars_3(int pos,CRGB C1, CRGB C2, CRGB C3){
    pos = constrain(pos,0,NUM_LEDS*3-1);
    int colorNum = pos/NUM_LEDS; //revolution number
    int barPos = pos % NUM_LEDS; //place on circle, from 0-12
    switch(colorNum){
        case 0:
            fill_gradient_RGB(leds,0,C1,barPos,C1);
            //leds[barPos] = C1;
            break;
        case 1:
            fill_gradient_RGB(leds,0,C1,barPos,C2);
            break;
        case 2:
            fill_gradient_RGB(leds,0,C2,barPos,C3);
            break;
    }
}

//Provide a limited encoder reading corresponting to tacticle clicks on the knob.
//Each click passes through 4 encoder pulses. This reduces it to 1 pulse per click
int get_encoder(int minVal, int maxVal){
    if(encoder.read()>maxVal*4)encoder.write(maxVal*4);
    else if(encoder.read()<minVal*4) encoder.write(minVal*4);
    return constrain(encoder.read()/4,minVal,maxVal);
}

void set_motor_speed() {
    if(motSpeed > 1) {
        motSpeed = max(motSpeed, MOT_MIN_SPEED); // skip the slow values
    }
    motSpeed = min(motSpeed, MOT_MAX_SPEED);
    analogWrite(MOTPIN, (int) motSpeed);
}

int get_sensor() {
    return analogRead(BUTTPIN);
}

void update_sensor_average() {
    sampleTick++; //Add pressure samples to the running average slower than 60Hz
//    if (sampleTick % RA_TICK_PERIOD == 0) {
    if (sampleTick % 50 == 0) {
        raPressure.addValue(get_sensor());
        avgPressure = raPressure.getAverage();
    }
}

void fade_leds() {
    fadeToBlackBy(leds,NUM_LEDS,20);
}

void update_leds() {
    FastLED.show();
}

void check_sensor_maxing_out() {
    //Alert that the Pressure voltage amplifier is railing, and the trim pot needs to be adjusted
    if(analogRead(BUTTPIN) > 4030) {
        //beep_motor(2093,2093,2093); //Three high beeps
    }
}

void debug_print_data() {
    if(DEBUG) {
        Serial.println("----------------------");
        Serial.print("Average pressure: "); Serial.println(avgPressure);
        Serial.print("Motor speed: "); Serial.println(motSpeed);
        Serial.print("Button pressed: "); Serial.println(get_button_state());
        Serial.print("State: "); Serial.println(state);
        Serial.print("Is cooling down: "); Serial.println(is_cooldown());
    }
    
    if(DEBUG_PLOT) {
        Serial.print(avgPressure);
        Serial.print(",");
        Serial.print(get_sensor());
        
        Serial.println("");
    }
}

void setup(){
    pinMode(REDPIN,   OUTPUT); //Connected to RGB LED in the encoder
    pinMode(GREENPIN, OUTPUT);
    pinMode(BLUEPIN,  OUTPUT);
    pinMode(ENC_SW,   INPUT);

    pinMode(SW1PIN,   INPUT); //Set DIP switch pins as inputs
    pinMode(SW2PIN,   INPUT);
    pinMode(SW3PIN,   INPUT);
    pinMode(SW4PIN,   INPUT);

    digitalWrite(SW1PIN, HIGH); //Enable pullup resistors on DIP switch pins.
    digitalWrite(SW2PIN, HIGH); //They are tied to GND when switched on.
    digitalWrite(SW3PIN, HIGH);
    digitalWrite(SW4PIN, HIGH);

    pinMode(MOTPIN,OUTPUT); //Enable "analog" out (PWM)

    pinMode(BUTTPIN,INPUT); //default is 10 bit resolution (1024), 0-3.3
    analogReadRes(12); //changing ADC resolution to 12 bits (4095)
    analogReadAveraging(32); //To reduce noise, average 32 samples each read.

    raPressure.clear(); //Initialize a running pressure average

    digitalWrite(MOTPIN, LOW);//Make sure the motor is off


    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    maximumDelta = 4096.0 - get_sensor();

    delay(1000); // 1 second delay for recovery


    Serial.begin(115200);

    beep_motor(1047,1396,2093);

    mode_manual();
}

void loop() {
    static int sampleTick = 0;
    //Run this section at the update frequency (default 60 Hz)
    if (millis() % period == 0) {
        delay(1);

        update_button_mode();
        check_button();
        
        run_selected_mode();

        update_sensor_average();
        check_sensor_maxing_out();

        fade_leds();
        update_leds();

        set_motor_speed();
        debug_print_data();
    }
}



