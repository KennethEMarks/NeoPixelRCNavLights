// INCLUDES
#include <math.h>       /* pow */
#include <EEPROM.h>
#include <arduino-timer.h>
#include <Adafruit_NeoPixel.h>
#include <OneButton.h>
#include "NeoPatterns.h"

#define DEBUG 1

// TYPES
typedef enum e_operation_state {
    OPERATION_STATE_INIT,
    OPERATION_STATE_CONFIG_MAIN_ON_NAV,     // Order is important as enum
    OPERATION_STATE_CONFIG_MAIN_ON_STROBE,  // value is used in calculation
    OPERATION_STATE_CONFIG_MAIN_ON_BEACON,
    OPERATION_STATE_CONFIG_MAIN_ON_LANDING,
    OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET,
    OPERATION_STATE_CONFIG_IN_NAV,             
    OPERATION_STATE_CONFIG_IN_STROBE,          
    OPERATION_STATE_CONFIG_IN_BEACON,
    OPERATION_STATE_CONFIG_IN_LANDING,
    OPERATION_STATE_CONFIG_IN_FACTORY_RESET,
    OPERATION_STATE_NORMAL,
    OPERATION_STATE_RAINBOW,
    OPERATION_STATE_CHASE
} eOperationState;

typedef enum e_eeprom_address {
    EEPROM_ADDRESS_NAV_LED_SEGMENT_COUNT,
    EEPROM_ADDRESS_STROBE_LED_SEGMENT_COUNT,
    EEPROM_ADDRESS_BEACON_LED_SEGMENT_COUNT,
    EEPROM_ADDRESS_LANDING_LED_SEGMENT_COUNT,
    EEPROM_ADDRESS_NAV_LED_SEGMENT_START_INDEX,
    EEPROM_ADDRESS_STROBE_LED_SEGMENT_START_INDEX,
    EEPROM_ADDRESS_BEACON_LED_SEGMENT_START_INDEX,
    EEPROM_ADDRESS_LANDING_LED_SEGMENT_START_INDEX
} eEepromAddress;

// FUNCTION DECLARATIONS
void initialize_eeprom_if_needed();
bool initialize_eeprom_address_if_needed(eEepromAddress eepromAddress, int eepromValue);
void read_eeprom();
void update_eeprom();

void initialize_nav_lights();
void turn_off_nav_lights();

// Nav Strobe functions called by timer
bool start_nav_strobe_on_1(void *);
bool start_nav_strobe_off_1(void *);
bool start_nav_strobe_on_2(void *);
bool start_nav_strobe_off_2(void *);

bool turn_on_nav_strobe(void *);
bool turn_off_nav_strobe(void *);

// Beacon functions called by timer
bool start_on_beacon(void *);
bool start_off_beacon(void *);

bool turn_on_beacon(void *);
bool turn_off_beacon(void *);

// Landing Lights Functions
void LandingLightsPulseWidthTimer();
void manage_landing_lights();

// Color Mode Receiver Channel Functions
void NavDisplayModePulseWidthTimer();
void manage_nav_display_mode();

// Config functions called by timer
bool turn_on_first_nav_led_no_repeat(void *);
bool turn_off_first_nav_led_no_repeat(void *);

bool toggle_first_nav_led_with_color(uint32_t color);

void blink_nav_for_number_of_segments(int num_of_blinks);

bool turn_on_first_nav_led_with_color_no_repeat(uint32_t color);

void rapid_blink_nav_then_set_config_main(uint32_t color);

void blink_nav_led_with_color(uint32_t color);
void rapid_blink_nav_led_with_color(uint32_t color);

void set_nav_lights_to_rainbow();
void set_nav_lights_to_theater_chase();
void update_color_mode_for_nav_lights();

// Running State Management
void manage_running_states();

// Configuration Button functions
void single_click();
void long_click_start();

// Configuration State Management
void manage_config_states();

// STATIC DEFINES
// Pin #s of Strings
#define PORT_NAV_AND_STROBE_LED_STRING_PIN 4
#define STARBOARD_NAV_AND_STROBE_LED_STRING_PIN 5   
#define BEACON_LED_STRING_PIN 6
#define LANDING_LED_STRING_PIN 7

// Pin #s for Nav Modes
#define LANDING_LED_TOGGLE_PIN 2
#define NAV_DISPLAY_MODE_PIN 3

// Default LEDs in Segments
#define DEFAULT_NAV_LED_SEGMENT_COUNT 1
#define DEFAULT_STROBE_LED_SEGMENT_COUNT 1
#define DEFAULT_BEACON_LED_SEGMENT_COUNT 1
#define DEFAULT_LANDING_LED_SEGMENT_COUNT 2

// Max LEDs in Segments
#define MAX_NAV_LED_SEGMENT_COUNT 2
#define MAX_STROBE_LED_SEGMENT_COUNT 2
#define MAX_BEACON_LED_SEGMENT_COUNT 2
#define MAX_LANDING_LED_SEGMENT_COUNT 3

// Start index of Segments in String
#define DEFAULT_NAV_LED_SEGMENT_START_INDEX 0
#define DEFAULT_STROBE_LED_SEGMENT_START_INDEX (DEFAULT_NAV_LED_SEGMENT_COUNT)
#define DEFAULT_BEACON_LED_SEGMENT_START_INDEX 0
#define DEFAULT_LANDING_LED_SEGMENT_START_INDEX 0

// EEPROM definitions
#define EEPROM_ADDRESS_EMPTY 255

// Neo Pixel Brightness
#define NEO_PIXEL_BRIGHTNESS 12 //255 //12

// Button Pin
#define BUTTON_PIN A0

#define LONG_CLICK_IN_MSECS 2000

#define CONFIG_MENU_ITEM_DURATION_IN_MSECS 3000

// VARS
// Count of LEDs in Segments
int nav_led_segment_count = DEFAULT_NAV_LED_SEGMENT_COUNT;
int strobe_led_segment_count = DEFAULT_STROBE_LED_SEGMENT_COUNT;
int beacon_led_segment_count = DEFAULT_BEACON_LED_SEGMENT_COUNT;
int landing_led_segment_count = DEFAULT_LANDING_LED_SEGMENT_COUNT;

// Start index of Segments in String
int nav_led_segment_start_index = DEFAULT_NAV_LED_SEGMENT_START_INDEX;
int strobe_led_segment_start_index = DEFAULT_STROBE_LED_SEGMENT_START_INDEX;
int beacon_led_segment_start_index = DEFAULT_BEACON_LED_SEGMENT_START_INDEX;
int landing_led_segment_start_index = DEFAULT_LANDING_LED_SEGMENT_START_INDEX;

// Count of LEDs in Strings
int nav_and_strobe_led_string_count = nav_led_segment_count + strobe_led_segment_count;

// Colors
uint32_t black = Adafruit_NeoPixel::Color(0, 0, 0);
uint32_t red = Adafruit_NeoPixel::Color(255, 0, 0);
uint32_t green = Adafruit_NeoPixel::Color(0, 255, 0);
uint32_t white = Adafruit_NeoPixel::Color(255, 255, 255);
uint32_t blue = Adafruit_NeoPixel::Color(0, 0, 255);
uint32_t purple = Adafruit_NeoPixel::Color(255, 0, 255);
uint32_t yellow = Adafruit_NeoPixel::Color(255, 255, 0);
uint32_t orange = Adafruit_NeoPixel::Color(165, 255, 0);

// State
eOperationState operation_state;

bool is_config_setting_modified = false;

bool landing_lights_on = false;

bool toggle_first_nav_led_on = false;

#define MAX_PULSE_WIDTH 2015
#define HIGH_PWM_POSITION 1900
#define MID_PWM_POSITION 1400
#define LOW_PWM_POSITION 900

// Landing Lights PWM vars
volatile long landing_led_pulse_start_time_in_micro_seconds;
volatile long landing_led_pulse_current_time_in_micro_seconds;
volatile long landing_led_pulses;
int landing_led_pulse_width_in_micro_seconds;

// Nav Display Mode PWM vars
volatile long nav_display_mode_pulse_start_time_in_micro_seconds;
volatile long nav_display_mode_pulse_current_time_in_micro_seconds;
volatile long nav_display_mode_pulses;
int nav_display_mode_pulse_width_in_micro_seconds;

// Times
unsigned long long_button_press_start_time_in_milliseconds;
unsigned long config_state_start_time_in_milliseconds;
unsigned long current_time_in_milliseconds;
unsigned long current_config_state_timer_in_milliseconds;
unsigned long config_loop_duration_in_milliseconds
        = (int)(OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET)
        * CONFIG_MENU_ITEM_DURATION_IN_MSECS;

// INSTANCES
Timer<12> timer; // 12 concurrent tasks, using millis as resolution

Timer<2, millis, uint32_t> color_timer;

NeoPatterns port_nav_strip(
        nav_and_strobe_led_string_count, 
        PORT_NAV_AND_STROBE_LED_STRING_PIN, 
        NEO_GRB + NEO_KHZ800, 
        NULL);

NeoPatterns starboard_nav_strip(
        nav_and_strobe_led_string_count, 
        STARBOARD_NAV_AND_STROBE_LED_STRING_PIN, 
        NEO_GRB + NEO_KHZ800, 
        NULL);

NeoPatterns beacon_strip(
        beacon_led_segment_count,
        BEACON_LED_STRING_PIN, 
        NEO_GRB + NEO_KHZ800,
        NULL);

NeoPatterns landing_strip(
        landing_led_segment_count,
        LANDING_LED_STRING_PIN, 
        NEO_GRB + NEO_KHZ800,
        NULL);

OneButton button(BUTTON_PIN); // NOTE:  Default constructor uses pull-up resistor 
                              //        and expects button to be active low
// SETUP AND MAIN LOOP
//////////////////////
void setup()
{
    Serial.begin(9600);

#ifdef DEBUG
    Serial.println("** Starting Nav Lights **");
#endif // DEBUG
    initialize_eeprom_if_needed();

    read_eeprom();

    // Setup Button for Configuration
    button.setClickTicks(LONG_CLICK_IN_MSECS);
    button.attachClick(single_click);
    button.attachLongPressStart(long_click_start);

    initialize_nav_lights();

    pinMode(LANDING_LED_TOGGLE_PIN, INPUT_PULLUP);
    pinMode(NAV_DISPLAY_MODE_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LANDING_LED_TOGGLE_PIN), LandingLightsPulseWidthTimer, CHANGE);
    attachInterrupt(digitalPinToInterrupt(NAV_DISPLAY_MODE_PIN), NavDisplayModePulseWidthTimer, CHANGE);

#ifdef DEBUG
    Serial.println("******");
    Serial.println("Current State: OPERATION_STATE_NORMAL");
#endif // DEBUG
}

void loop()
{
    timer.tick();
    color_timer.tick();
    button.tick();

    manage_running_states();
    manage_config_states();

    switch (operation_state)
    {
        case OPERATION_STATE_NORMAL:
        case OPERATION_STATE_RAINBOW:
        case OPERATION_STATE_CHASE:

            manage_nav_display_mode();

            break;

        default:

            break;
    }

    if (operation_state == OPERATION_STATE_NORMAL)
    {
        manage_landing_lights();
    }
}
//////////////////////

// FUNCTION DEFINITIONS
// Initialization of EEPROM (if needed)
void initialize_eeprom_if_needed()
{
    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_NAV_LED_SEGMENT_COUNT, DEFAULT_NAV_LED_SEGMENT_COUNT);

    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_STROBE_LED_SEGMENT_COUNT,
            DEFAULT_STROBE_LED_SEGMENT_COUNT);

    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_BEACON_LED_SEGMENT_COUNT,
            DEFAULT_BEACON_LED_SEGMENT_COUNT);

    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_LANDING_LED_SEGMENT_COUNT,
            DEFAULT_LANDING_LED_SEGMENT_COUNT);

    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_NAV_LED_SEGMENT_START_INDEX, DEFAULT_NAV_LED_SEGMENT_START_INDEX);

    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_STROBE_LED_SEGMENT_START_INDEX,
            DEFAULT_STROBE_LED_SEGMENT_START_INDEX);

    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_BEACON_LED_SEGMENT_START_INDEX,
            DEFAULT_BEACON_LED_SEGMENT_START_INDEX);

    initialize_eeprom_address_if_needed(e_eeprom_address::EEPROM_ADDRESS_LANDING_LED_SEGMENT_START_INDEX,
            DEFAULT_LANDING_LED_SEGMENT_START_INDEX);
}

bool initialize_eeprom_address_if_needed(eEepromAddress eepromAddress, int eepromValue)
{
    bool retVal = false;

    if (EEPROM.read(eepromAddress) == EEPROM_ADDRESS_EMPTY)
    {
        #ifdef DEBUG
        Serial.print("Initializing EEPROM ADDRESS: ");
        Serial.println(eepromAddress);
        #endif // DEBUG
        EEPROM.write(eepromAddress, eepromValue);
        retVal = true;
    }

    return retVal;
}

void read_eeprom()
{
    nav_led_segment_count = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_NAV_LED_SEGMENT_COUNT);
    strobe_led_segment_count = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_STROBE_LED_SEGMENT_COUNT);
    beacon_led_segment_count = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_BEACON_LED_SEGMENT_COUNT);
    landing_led_segment_count = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_LANDING_LED_SEGMENT_COUNT);
 
    nav_led_segment_start_index = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_NAV_LED_SEGMENT_START_INDEX);
    strobe_led_segment_start_index = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_STROBE_LED_SEGMENT_START_INDEX);
    beacon_led_segment_start_index = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_BEACON_LED_SEGMENT_START_INDEX);
    landing_led_segment_start_index = EEPROM.read(e_eeprom_address::EEPROM_ADDRESS_LANDING_LED_SEGMENT_START_INDEX);
}

void update_eeprom()
{
    #ifdef DEBUG
    Serial.println("Updating EEPROM");
    #endif // DEBUG
    EEPROM.update(e_eeprom_address::EEPROM_ADDRESS_NAV_LED_SEGMENT_COUNT, nav_led_segment_count);
    EEPROM.update(e_eeprom_address::EEPROM_ADDRESS_STROBE_LED_SEGMENT_COUNT, strobe_led_segment_count);
    EEPROM.update((e_eeprom_address::EEPROM_ADDRESS_BEACON_LED_SEGMENT_COUNT), beacon_led_segment_count);
    EEPROM.update(e_eeprom_address::EEPROM_ADDRESS_LANDING_LED_SEGMENT_COUNT, landing_led_segment_count);
            
    EEPROM.update(e_eeprom_address::EEPROM_ADDRESS_NAV_LED_SEGMENT_START_INDEX, nav_led_segment_start_index);
    EEPROM.update(e_eeprom_address::EEPROM_ADDRESS_STROBE_LED_SEGMENT_START_INDEX, strobe_led_segment_start_index);
    EEPROM.update(e_eeprom_address::EEPROM_ADDRESS_BEACON_LED_SEGMENT_START_INDEX, beacon_led_segment_start_index);
    EEPROM.update(e_eeprom_address::EEPROM_ADDRESS_LANDING_LED_SEGMENT_START_INDEX, landing_led_segment_start_index);
}

// Initialization of Nav Lights
void initialize_nav_lights()
{
    // Inititalize State
    operation_state = OPERATION_STATE_INIT;

    // Initialize LED Strips
    port_nav_strip.begin();
    port_nav_strip.updateLength(nav_led_segment_count + strobe_led_segment_count);
    port_nav_strip.setBrightness(NEO_PIXEL_BRIGHTNESS);
    port_nav_strip.show();

    starboard_nav_strip.begin();
    starboard_nav_strip.updateLength(nav_led_segment_count + strobe_led_segment_count);
    starboard_nav_strip.setBrightness(NEO_PIXEL_BRIGHTNESS);
    starboard_nav_strip.show();

    beacon_strip.begin();
    beacon_strip.updateLength(beacon_led_segment_count);
    beacon_strip.setBrightness(NEO_PIXEL_BRIGHTNESS);
    beacon_strip.show();

    landing_strip.begin();
    landing_strip.updateLength(landing_led_segment_count);
    landing_strip.setBrightness(NEO_PIXEL_BRIGHTNESS);
    landing_strip.show();

    // Set Pixel Colors //
    // Port Nav
    port_nav_strip.clear();
    port_nav_strip.show();
    port_nav_strip.fill(red, nav_led_segment_start_index, nav_led_segment_count);
    port_nav_strip.show();

    // Starboard Nav
    starboard_nav_strip.clear();
    starboard_nav_strip.show();
    starboard_nav_strip.fill(green, nav_led_segment_start_index, nav_led_segment_count);
    starboard_nav_strip.show();

    // Setup Timer for Nav Strobes
    timer.in(0, start_nav_strobe_on_1);
    timer.in(50, start_nav_strobe_off_1);
    timer.in(100, start_nav_strobe_on_2);
    timer.in(150, start_nav_strobe_off_2);

    // Setup Timer for Beacon
    timer.in(500, start_on_beacon);
    timer.in(600, start_off_beacon);

    // Landing
    landing_strip.clear();
    landing_strip.fill(white, landing_led_segment_start_index, landing_led_segment_count);
    landing_strip.show();

    landing_lights_on = true;

    operation_state = OPERATION_STATE_NORMAL;
}

// Turn off Nav Lights
void turn_off_nav_lights()
{
    timer.cancel();
    port_nav_strip.clear();
    port_nav_strip.show();
    starboard_nav_strip.clear();
    starboard_nav_strip.show();
    beacon_strip.clear();
    beacon_strip.show();
    landing_lights_on = false;
    landing_strip.clear();
    landing_strip.show();
}

// Nav Strobe functions
bool start_nav_strobe_on_1(void *) {
    timer.every(1000, turn_on_nav_strobe);
    return false;
}

bool start_nav_strobe_off_1(void *) {
    timer.every(1000, turn_off_nav_strobe);
    return false;
}

bool start_nav_strobe_on_2(void *) {
    timer.every(1000, turn_on_nav_strobe);
    return false;
}

bool start_nav_strobe_off_2(void *) {
    timer.every(1000, turn_off_nav_strobe);
    return false;
}

bool turn_on_nav_strobe(void *) {
    port_nav_strip.fill(white, strobe_led_segment_start_index,
            strobe_led_segment_count);
    port_nav_strip.show();

    starboard_nav_strip.fill(white, strobe_led_segment_start_index,
            strobe_led_segment_count);
    starboard_nav_strip.show();

    return true;
}

bool turn_off_nav_strobe(void *) {
    port_nav_strip.fill(black, 
                        strobe_led_segment_start_index,
                        strobe_led_segment_count);
    port_nav_strip.show();

    starboard_nav_strip.fill(black, 
                        strobe_led_segment_start_index,
                        strobe_led_segment_count);
    starboard_nav_strip.show();

   return true;
}

// Beacon functions
bool start_on_beacon(void *) {
    timer.every(1000, turn_on_beacon);
    return false;
}

bool start_off_beacon(void *) {
    timer.every(1000, turn_off_beacon);
    return false;
}

bool turn_on_beacon(void *) {
    beacon_strip.fill(red, beacon_led_segment_start_index, beacon_led_segment_count);
    beacon_strip.show();
    return true; // to repeat the action - false to stop
}

bool turn_off_beacon(void *) {
    beacon_strip.fill(black, beacon_led_segment_start_index, beacon_led_segment_count);
    beacon_strip.show();
    return true; // to repeat the action - false to stop
}

// Landing Lights Functions
void LandingLightsPulseWidthTimer() {
    noInterrupts();
    landing_led_pulse_current_time_in_micro_seconds = micros();
    interrupts();

    if (landing_led_pulse_current_time_in_micro_seconds 
            > landing_led_pulse_start_time_in_micro_seconds)
    {
        landing_led_pulses = landing_led_pulse_current_time_in_micro_seconds
                - landing_led_pulse_start_time_in_micro_seconds;

        landing_led_pulse_start_time_in_micro_seconds
            = landing_led_pulse_current_time_in_micro_seconds;

        if (landing_led_pulses < MAX_PULSE_WIDTH)
        {
            landing_led_pulse_width_in_micro_seconds = landing_led_pulses;

            if (operation_state == OPERATION_STATE_NORMAL)
            {
                if (!landing_lights_on && landing_led_pulse_width_in_micro_seconds > HIGH_PWM_POSITION) {
                    landing_lights_on = true;
                    #ifdef DEBUG
                    Serial.println("Landing Lights: ON");
                    Serial.println(landing_led_pulse_width_in_micro_seconds);
                    #endif // DEBUG
                } else if (landing_lights_on && landing_led_pulse_width_in_micro_seconds <= HIGH_PWM_POSITION) {
                    landing_lights_on = false;
                    #ifdef DEBUG
                    Serial.println("Landing Lights: OFF");
                    Serial.println(landing_led_pulse_width_in_micro_seconds);
                    #endif // DEBUG
                }
            }
        }
    }
}

void manage_landing_lights()
{

    if (landing_lights_on) {
        landing_strip.fill(white, landing_led_segment_start_index, landing_led_segment_count);
        landing_strip.show();
    } else {
        landing_strip.fill(black, landing_led_segment_start_index, landing_led_segment_count);
        landing_strip.show();
    }

    #ifdef DEBUG
    //Serial.println(landing_led_pulse_width_in_micro_seconds);
    #endif // DEBUG
}

void NavDisplayModePulseWidthTimer()
{
    noInterrupts();
    nav_display_mode_pulse_current_time_in_micro_seconds = micros();
    interrupts();

    if (nav_display_mode_pulse_current_time_in_micro_seconds 
            > nav_display_mode_pulse_start_time_in_micro_seconds)
    {
        nav_display_mode_pulses = nav_display_mode_pulse_current_time_in_micro_seconds
                - nav_display_mode_pulse_start_time_in_micro_seconds;

        nav_display_mode_pulse_start_time_in_micro_seconds
            = nav_display_mode_pulse_current_time_in_micro_seconds;

        if (nav_display_mode_pulses < MAX_PULSE_WIDTH)
        {
            nav_display_mode_pulse_width_in_micro_seconds = nav_display_mode_pulses;

            if (operation_state != OPERATION_STATE_CHASE
                && nav_display_mode_pulse_width_in_micro_seconds > 2000 - 100
                /*&& nav_display_mode_pulse_width_in_micro_seconds < 2100*/) {
                operation_state = OPERATION_STATE_CHASE;
                #ifdef DEBUG
                Serial.println("Display Mode: THEATER CHASE");
                Serial.println(nav_display_mode_pulse_width_in_micro_seconds);
                #endif // DEBUG

                turn_off_nav_lights();

                set_nav_lights_to_theater_chase();

            } else if (operation_state != OPERATION_STATE_RAINBOW
                && nav_display_mode_pulse_width_in_micro_seconds > 1500 - 100
                && nav_display_mode_pulse_width_in_micro_seconds < 1500 + 100) {
                operation_state = OPERATION_STATE_RAINBOW;
                #ifdef DEBUG
                Serial.println("Display Mode: RAINBOW");
                Serial.println(nav_display_mode_pulse_width_in_micro_seconds);
                #endif // DEBUG

                turn_off_nav_lights();

                set_nav_lights_to_rainbow();

            } else if (operation_state != OPERATION_STATE_NORMAL
                //&& nav_display_mode_pulse_width_in_micro_seconds > 1000 - 100
                && nav_display_mode_pulse_width_in_micro_seconds < 1000 + 100) {
                operation_state = OPERATION_STATE_NORMAL;
                #ifdef DEBUG
                Serial.println("Display Mode: NAV");
                Serial.println(nav_display_mode_pulse_width_in_micro_seconds);
                #endif // DEBUG

                turn_off_nav_lights();

                initialize_nav_lights();
            }
        }
    }
}

void manage_nav_display_mode()
{
    #ifdef DEBUG
    //Serial.println(nav_display_mode_pulse_width_in_micro_seconds);
    #endif // DEBUG
}

void set_nav_lights_to_rainbow()
{
    port_nav_strip.RainbowCycle(3);
    starboard_nav_strip.RainbowCycle(3);
    beacon_strip.RainbowCycle(3);
    landing_strip.RainbowCycle(3);
}

void set_nav_lights_to_theater_chase()
{
    uint8_t random_color = random(255);
    uint8_t anti_random_color = random_color + 128;

    port_nav_strip.ActivePattern = THEATER_CHASE;
    port_nav_strip.Interval = 100;
    port_nav_strip.Color1 = port_nav_strip.Wheel(random_color);
    port_nav_strip.Color2 = port_nav_strip.Wheel(anti_random_color);

    starboard_nav_strip.ActivePattern = THEATER_CHASE;
    starboard_nav_strip.Interval = 100;
    starboard_nav_strip.Color1 = starboard_nav_strip.Wheel(random_color);
    starboard_nav_strip.Color2 = starboard_nav_strip.Wheel(anti_random_color);

    beacon_strip.ActivePattern = THEATER_CHASE;
    beacon_strip.Interval = 100;
    beacon_strip.Color1 = beacon_strip.Wheel(random_color);
    beacon_strip.Color2 = beacon_strip.Wheel(anti_random_color);

    landing_strip.ActivePattern = THEATER_CHASE;
    landing_strip.Interval = 100;
    landing_strip.Color1 = landing_strip.Wheel(random_color);
    landing_strip.Color2 = landing_strip.Wheel(anti_random_color);
}

void update_color_mode_for_nav_lights()
{
    port_nav_strip.Update();
    starboard_nav_strip.Update();
    beacon_strip.Update();
    landing_strip.Update();
}

void manage_running_states()
{
    switch (operation_state)
    {
        case OPERATION_STATE_NORMAL:
            
            break;
        
        case OPERATION_STATE_RAINBOW:

            update_color_mode_for_nav_lights();

            break;

        case OPERATION_STATE_CHASE:

            update_color_mode_for_nav_lights();

            break;

        default:
            break;
        }
}

// CONFIGURATION FUNCTIONS

bool turn_on_first_nav_led_no_repeat(void *)
{
    port_nav_strip.fill(white, 0, 1);
    port_nav_strip.show();

    return false;
}

bool turn_off_first_nav_led_no_repeat(void *)
{
    port_nav_strip.fill(black, 0, 1);
    port_nav_strip.show();

    return false;
}

void blink_nav_for_number_of_segments(int num_of_blinks)
{
    const int PERIOD_IN_MSECS = 200;
    int time_to_fire_function_in_msecs = 0;

    timer.in(time_to_fire_function_in_msecs, turn_off_first_nav_led_no_repeat);
    time_to_fire_function_in_msecs = PERIOD_IN_MSECS * 2;

    for (int i = 0; i < num_of_blinks; i++)
    {
        timer.in(time_to_fire_function_in_msecs, turn_on_first_nav_led_no_repeat);
        time_to_fire_function_in_msecs += PERIOD_IN_MSECS;
        timer.in(time_to_fire_function_in_msecs, turn_off_first_nav_led_no_repeat);
        time_to_fire_function_in_msecs += PERIOD_IN_MSECS;
    }
}

bool turn_on_first_nav_led_with_color_no_repeat(uint32_t color)
{
    port_nav_strip.fill(color, 0, 1);
    port_nav_strip.show();

    return false;
}

bool turn_off_first_nav_led_with_color_no_repeat(uint32_t color)
{
    port_nav_strip.fill(black, 0, 1);
    port_nav_strip.show();

    return false;
}

bool toggle_first_nav_led_with_color(uint32_t color)
{
    if (toggle_first_nav_led_on) {
        port_nav_strip.fill(color, 0, 1);
        port_nav_strip.show();

        toggle_first_nav_led_on = false;
    } else {
        port_nav_strip.fill(black, 0, 1);
        port_nav_strip.show();

        toggle_first_nav_led_on = true;
    }

    return true;
}

void rapid_blink_nav_then_set_config_main(uint32_t color)
{
    const int PERIOD_IN_MSECS = 75;
    int time_to_fire_function_in_msecs = 0;

    timer.in(time_to_fire_function_in_msecs, turn_off_first_nav_led_no_repeat);
    time_to_fire_function_in_msecs = PERIOD_IN_MSECS * 2;

    for (int i = 0; i < 5; i++)
    {
        timer.in(time_to_fire_function_in_msecs, turn_on_first_nav_led_no_repeat);
        time_to_fire_function_in_msecs += PERIOD_IN_MSECS;
        timer.in(time_to_fire_function_in_msecs, turn_off_first_nav_led_no_repeat);
        time_to_fire_function_in_msecs += PERIOD_IN_MSECS;
    }

    color_timer.in(time_to_fire_function_in_msecs, turn_on_first_nav_led_with_color_no_repeat, color);
}

void blink_nav_led_with_color(uint32_t color)
{
    const int PERIOD_IN_MSECS = 200;
    toggle_first_nav_led_on = true;
    color_timer.every(PERIOD_IN_MSECS, toggle_first_nav_led_with_color, color);
}

void rapid_blink_nav_led_with_color(uint32_t color)
{
    const int PERIOD_IN_MSECS = 75;
    toggle_first_nav_led_on = true;
    color_timer.every(PERIOD_IN_MSECS, toggle_first_nav_led_with_color, color);
}

// Configuration Button functions
void long_click_start()
{
    long_button_press_start_time_in_milliseconds 
            = current_time_in_milliseconds = millis();

    switch (operation_state)
    {
        case OPERATION_STATE_NORMAL:
        case OPERATION_STATE_RAINBOW:
        case OPERATION_STATE_CHASE:

            config_state_start_time_in_milliseconds
                    = current_time_in_milliseconds;

            operation_state = OPERATION_STATE_CONFIG_MAIN_ON_NAV;

            current_config_state_timer_in_milliseconds = 0;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_NAV");
            #endif // DEBUG

            turn_off_nav_lights();

            port_nav_strip.fill(green, 0, 1);
            port_nav_strip.show();

            break;

        case OPERATION_STATE_CONFIG_MAIN_ON_NAV:
        case OPERATION_STATE_CONFIG_MAIN_ON_STROBE:
        case OPERATION_STATE_CONFIG_MAIN_ON_BEACON:
        case OPERATION_STATE_CONFIG_MAIN_ON_LANDING:
        case OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET:

            operation_state = OPERATION_STATE_NORMAL;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_NORMAL");
            #endif // DEBUG

            update_eeprom();

            initialize_nav_lights();

            break;

        case OPERATION_STATE_CONFIG_IN_FACTORY_RESET:

            #ifdef DEBUG
            Serial.println("Doing Factory Reset");
            #endif // DEBUG

            is_config_setting_modified = true;

            current_config_state_timer_in_milliseconds = 0;

            // Reset all segment counts to defaults and update lengths
            nav_led_segment_count = DEFAULT_NAV_LED_SEGMENT_COUNT;
            nav_led_segment_start_index = DEFAULT_NAV_LED_SEGMENT_START_INDEX;
            strobe_led_segment_count = DEFAULT_STROBE_LED_SEGMENT_COUNT;
            strobe_led_segment_start_index = nav_led_segment_count;
            nav_and_strobe_led_string_count = nav_led_segment_count + strobe_led_segment_count;

            beacon_led_segment_count = DEFAULT_BEACON_LED_SEGMENT_COUNT;
            beacon_led_segment_start_index = DEFAULT_BEACON_LED_SEGMENT_START_INDEX;

            landing_led_segment_count = DEFAULT_LANDING_LED_SEGMENT_COUNT;
            landing_led_segment_start_index = DEFAULT_LANDING_LED_SEGMENT_START_INDEX;

            port_nav_strip.updateLength(nav_and_strobe_led_string_count);
            starboard_nav_strip.updateLength(nav_and_strobe_led_string_count);
            beacon_strip.updateLength(beacon_led_segment_count);
            landing_strip.updateLength(landing_led_segment_count);

            color_timer.cancel();
            rapid_blink_nav_led_with_color(purple);

            break;

        default:

            break;
    }
}

void single_click()
{
    switch (operation_state)
    {
        case OPERATION_STATE_NORMAL:

            operation_state = OPERATION_STATE_RAINBOW;
            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_RAINBOW");
            #endif // DEBUG

            turn_off_nav_lights();

            set_nav_lights_to_rainbow();

            break;

        case OPERATION_STATE_RAINBOW:

            operation_state = OPERATION_STATE_CHASE;
            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_CHASE");
            #endif // DEBUG

            turn_off_nav_lights();

            set_nav_lights_to_theater_chase();

            break;

        case OPERATION_STATE_CHASE:

            operation_state = OPERATION_STATE_NORMAL;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_NORMAL");
            # endif // DEBUG

            turn_off_nav_lights();

            initialize_nav_lights();

            break;

        case OPERATION_STATE_CONFIG_MAIN_ON_NAV:

            operation_state = OPERATION_STATE_CONFIG_IN_NAV;

            current_config_state_timer_in_milliseconds = 0;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_CONFIG_IN_NAV");
            #endif // DEBUG

            blink_nav_for_number_of_segments(nav_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_IN_NAV:

            #ifdef DEBUG
            Serial.println("Modifying NAV");
            #endif // DEBUG

            is_config_setting_modified = true;

            current_config_state_timer_in_milliseconds = 0;

            nav_led_segment_count++;

            if (nav_led_segment_count > MAX_NAV_LED_SEGMENT_COUNT)
            {
                nav_led_segment_count = 1;
            }

            strobe_led_segment_start_index = nav_led_segment_count;

            nav_and_strobe_led_string_count = nav_led_segment_count + strobe_led_segment_count;

            port_nav_strip.updateLength(nav_and_strobe_led_string_count);
            starboard_nav_strip.updateLength(nav_and_strobe_led_string_count);

            blink_nav_for_number_of_segments(nav_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_MAIN_ON_STROBE:

            operation_state = OPERATION_STATE_CONFIG_IN_STROBE;

            current_config_state_timer_in_milliseconds = 0;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_CONFIG_IN_STROBE");
            #endif // DEBUG

            blink_nav_for_number_of_segments(strobe_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_IN_STROBE:

            #ifdef DEBUG
            Serial.println("Modifying STROBE");
            #endif // DEBUG

            is_config_setting_modified = true;

            current_config_state_timer_in_milliseconds = 0;

            strobe_led_segment_count++;

            if (strobe_led_segment_count > MAX_STROBE_LED_SEGMENT_COUNT)
            {
                strobe_led_segment_count = 1;
            }

            nav_and_strobe_led_string_count = nav_led_segment_count + strobe_led_segment_count;

            port_nav_strip.updateLength(nav_and_strobe_led_string_count);
            starboard_nav_strip.updateLength(nav_and_strobe_led_string_count);

            blink_nav_for_number_of_segments(strobe_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_MAIN_ON_BEACON:

            operation_state = OPERATION_STATE_CONFIG_IN_BEACON;

            current_config_state_timer_in_milliseconds = 0;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_CONFIG_IN_BEACON");
            #endif // DEBUG

            blink_nav_for_number_of_segments(beacon_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_IN_BEACON:

            #ifdef DEBUG
            Serial.println("Modifying BEACON");
            #endif // DEBUG

            is_config_setting_modified = true;

            current_config_state_timer_in_milliseconds = 0;

            beacon_led_segment_count++;

            if (beacon_led_segment_count > MAX_BEACON_LED_SEGMENT_COUNT)
            {
                beacon_led_segment_count = 1;
            }

            beacon_strip.updateLength(beacon_led_segment_count);

            blink_nav_for_number_of_segments(beacon_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_MAIN_ON_LANDING:

            operation_state = OPERATION_STATE_CONFIG_IN_LANDING;

            current_config_state_timer_in_milliseconds = 0;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_CONFIG_IN_LANDING");
            #endif // DEBUG

            blink_nav_for_number_of_segments(landing_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_IN_LANDING:

            #ifdef DEBUG
            Serial.println("Modifying LANDING");
            #endif // DEBUG

            is_config_setting_modified = true;

            current_config_state_timer_in_milliseconds = 0;

            landing_led_segment_count++;

            if (landing_led_segment_count > MAX_LANDING_LED_SEGMENT_COUNT)
            {
                landing_led_segment_count = 1;
            }

            landing_strip.updateLength(landing_led_segment_count);

            blink_nav_for_number_of_segments(landing_led_segment_count);

            break;

        case OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET:

            operation_state = OPERATION_STATE_CONFIG_IN_FACTORY_RESET;

            current_config_state_timer_in_milliseconds = 0;

            #ifdef DEBUG
            Serial.println("State Transition TO: OPERATION_STATE_CONFIG_IN_FACTORY_RESET");
            #endif // DEBUG

            blink_nav_led_with_color(purple);

            break;

        default:
            break;
    }
}

// Configuration State Management
void manage_config_states()
{
    unsigned long now_in_milliseconds = millis();

    switch (operation_state)
    {
        case OPERATION_STATE_CONFIG_MAIN_ON_NAV:
        case OPERATION_STATE_CONFIG_MAIN_ON_STROBE:
        case OPERATION_STATE_CONFIG_MAIN_ON_BEACON:
        case OPERATION_STATE_CONFIG_MAIN_ON_LANDING:
        case OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET:

        case OPERATION_STATE_CONFIG_IN_NAV:
        case OPERATION_STATE_CONFIG_IN_STROBE:
        case OPERATION_STATE_CONFIG_IN_BEACON:
        case OPERATION_STATE_CONFIG_IN_LANDING:
        case OPERATION_STATE_CONFIG_IN_FACTORY_RESET:

            current_config_state_timer_in_milliseconds += now_in_milliseconds - current_time_in_milliseconds;

            current_time_in_milliseconds = now_in_milliseconds;

            switch (operation_state)
            {
                // Main Menu States
                case OPERATION_STATE_CONFIG_MAIN_ON_NAV:
                    
                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_STROBE;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_STROBE");
                        #endif // DEBUG

                        port_nav_strip.fill(blue, 0, 1);
                        port_nav_strip.show();
                    }

                    break;
                
                case OPERATION_STATE_CONFIG_MAIN_ON_STROBE:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_BEACON;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_BEACON");
                        #endif // DEBU

                        port_nav_strip.fill(red, 0, 1);
                        port_nav_strip.show();
                    }

                    break;

                case OPERATION_STATE_CONFIG_MAIN_ON_BEACON:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_LANDING;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_LANDING");
                        #endif // DEBUG

                        port_nav_strip.fill(yellow, 0, 1);
                        port_nav_strip.show();
                    }

                    break;

                case OPERATION_STATE_CONFIG_MAIN_ON_LANDING:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET");
                        #endif // DEBUG

                        port_nav_strip.fill(purple, 0, 1);
                        port_nav_strip.show();
                    }

                    break;

                case OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_NAV;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_NAV");
                        #endif // DEBUG

                        port_nav_strip.fill(green, 0, 1);
                        port_nav_strip.show();
                    }

                    break;

                case OPERATION_STATE_CONFIG_IN_NAV:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_STROBE;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_STROBE");
                        #endif // DEBUG

                        if (is_config_setting_modified) {
                            is_config_setting_modified = false;
                            rapid_blink_nav_then_set_config_main(blue);
                        } else {
                            port_nav_strip.fill(blue, 0, 1);
                            port_nav_strip.show();
                        }
                    }

                    break;
                
                case OPERATION_STATE_CONFIG_IN_STROBE:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_BEACON;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_BEACON");
                        #endif // DEBUG

                        if (is_config_setting_modified) {
                            is_config_setting_modified = false;
                            rapid_blink_nav_then_set_config_main(red);
                        } else {
                            port_nav_strip.fill(red, 0, 1);
                            port_nav_strip.show();
                        }
                   }

                   break;

                case OPERATION_STATE_CONFIG_IN_BEACON:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_LANDING;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_LANDING");
                        #endif // DEBUG

                        if (is_config_setting_modified) {
                            is_config_setting_modified = false;
                            rapid_blink_nav_then_set_config_main(yellow);
                        } else {
                            port_nav_strip.fill(yellow, 0, 1);
                            port_nav_strip.show();
                        }
                   }

                   break;

                case OPERATION_STATE_CONFIG_IN_LANDING:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_FACTORY_RESET");
                        #endif // DEBUG

                        if (is_config_setting_modified) {
                            is_config_setting_modified = false;
                            rapid_blink_nav_then_set_config_main(purple);
                        } else {
                            port_nav_strip.fill(purple, 0, 1);
                            port_nav_strip.show();
                        }
                   }

                   break;

                case OPERATION_STATE_CONFIG_IN_FACTORY_RESET:

                    if (current_config_state_timer_in_milliseconds > CONFIG_MENU_ITEM_DURATION_IN_MSECS)
                    {
                        color_timer.cancel();
                        current_config_state_timer_in_milliseconds = 0;
                        operation_state = OPERATION_STATE_CONFIG_MAIN_ON_NAV;
                        #ifdef DEBUG
                        Serial.println("State Transition TO: OPERATION_STATE_CONFIG_MAIN_ON_NAV");
                        #endif // DEBUG

                        if (is_config_setting_modified) {
                            is_config_setting_modified = false;
                            port_nav_strip.fill(green, 0, 1);
                            port_nav_strip.show();
                        } else {
                            port_nav_strip.fill(green, 0, 1);
                            port_nav_strip.show();
                        }
                    }

                    break;

                default:
                    break;
            }

            break;
            
        default:

            break;
    }
}