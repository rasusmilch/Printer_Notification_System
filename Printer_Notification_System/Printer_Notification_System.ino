/**
 * @file Printer_Notification_System.ino
 * @brief Main program for controlling a notification system to alert users of
 * new printouts.
 *
 * This program is designed to control and monitor various components connected
 * to an Arduino Nano, including buttons, LEDs, a notification light, and a
 * buzzer. The system uses a watchdog timer (WDT) to ensure reliability and
 * robustness by resetting the system in case of unexpected behavior. Key
 * functionalities include:
 *
 * - **Button Handling**: The program uses the AceButton library to manage
 * button events for the light and buzzer buttons. It handles press and
 * long-press events to change the modes of the light and buzzer.
 *
 * - **Pulsing Control**: The `PinPulser` class controls the pulsing behavior of
 * the notification light and buzzer with configurable high and low times. It
 * supports various modes such as disabled, pulsing, one-shot, and steady state.
 *
 * - **Watchdog Timer (WDT)**: The program employs a WDT to monitor task
 * execution. It initializes, checks, and updates the WDT state before and after
 * each task. If any task takes too long or fails to complete, the WDT will
 * reset the system to prevent malfunction.
 *
 * - **EEPROM Storage**: Current settings for the light and buzzer modes are
 * stored in the EEPROM to retain settings between power cycles. The EEWL
 * library is used to manage EEPROM read/write operations with CRC for data
 * integrity.
 *
 * - **Feedback and Notifications**: The system provides visual and auditory
 * feedback using LEDs and a buzzer. Different states are indicated by changing
 * the color of the buttons and pulsing pattern of the LEDs and the buzzer.
 *
 * - **Task Timing and Logging**: The program logs task execution times and
 * checks if they fall within specified limits. It logs events to the serial
 * monitor for debugging and monitoring purposes.
 *
 * The primary objective of this program is to efficiently alert users when a
 * new printout is available in the printer tray, ensuring timely handling and
 * processing of documents. By leveraging advanced monitoring and notification
 * mechanisms, this system guarantees a robust and responsive solution for
 * managing printout notifications, enhancing productivity and operational
 * efficiency.
 */

// Include necessary headers
#include "PinPulser.h"
#include "eewl.h"
#include <AceButton.h>
#include <avr/wdt.h>

// Declare inline function for halting and waiting for the watchdog timer to
// reset the system
inline void haltAndWaitForWatchdog();

// Define task states with explicit values
enum TaskState {
  TASK1 = 0,
  TASK2 = 1,
  TASK3 = 2,
  TASK4 = 3,
  LOOP = 4,
  COMPLETE = 5,
  SETUP = 6
};

volatile TaskState currentTask = SETUP; // Initialize the current task

// Watchdog timer state variables
volatile uint32_t wdt_state = 0;
volatile uint32_t wdt_redundant_state = 0;

// Task completion flags
volatile uint8_t taskCompletionFlags = 0;
volatile uint8_t redundantTaskCompletionFlags = 0;

// Timing variables for watchdog
volatile uint32_t wdt_start_time, wdt_end_time;

// Arrays to store minimum and maximum times for each task
volatile uint32_t wdt_min_times[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                      0xFFFFFFFF};
volatile uint32_t wdt_max_times[4] = {0, 0, 0, 0};

// Define a safety code (could be a CRC or checksum in a real application)
const uint8_t safetyCode = 0xA5; // Example safety code
volatile uint8_t safetyCodeRegister = 0;

// Define error codes
enum ErrorCode {
  NO_ERROR,
  STATE_ERROR,
  REDUNDANCY_ERROR,
  SAFETY_CODE_ERROR,
  TASK_ERROR,
  TIMING_ERROR
};

ErrorCode errorCode = NO_ERROR; // Initialize the error code

// Define timing windows (in milliseconds)
const unsigned long TASK1_MIN_TIME = 0;
const unsigned long TASK1_MAX_TIME = 500;
const unsigned long TASK2_MIN_TIME = 0;
const unsigned long TASK2_MAX_TIME = 500;
const unsigned long TASK3_MIN_TIME = 0;
const unsigned long TASK3_MAX_TIME = 500;
const unsigned long TASK4_MIN_TIME = 0;
const unsigned long TASK4_MAX_TIME = 500;

// Define pin numbers
#define PIN_BTN_LIGHT 5
#define PIN_BTN_BUZZER 6
#define PIN_BTN_LIGHT_LED_RED 3
#define PIN_BTN_LIGHT_LED_GRN 4
#define PIN_BTN_BUZZER_LED_RED 7
#define PIN_BTN_BUZZER_LED_GRN 8
#define PIN_POWER_LED 12
#define PIN_NOTIFICATION_LIGHT 9
#define PIN_NOTIFICATION_BUZZER 10
#define PIN_PRINTOUT 11

// Define timing constants
#define STROBE_SLOW_OFF_MS 10000UL
#define STROBE_SLOW_ON_MS 100UL
#define STROBE_FAST_OFF_MS 1000UL
#define STROBE_FAST_ON_MS 100UL
#define BUZZER_ONCE_ON_MS 100UL

// Define debounce delay and printout old time constants
#define DEBOUNCE_DELAY 50UL
#define PRINTOUT_OLD_MS                                                        \
  10000UL // When is a printout considered old, so increase urgency.

// Using the ace_button namespace for button handling
using namespace ace_button;

// Define button configuration objects
ButtonConfig btnLightConfig;
ButtonConfig btnBuzzerConfig;

// Define AceButton objects for light and buzzer buttons
AceButton btnLight(&btnLightConfig, PIN_BTN_LIGHT, HIGH);
AceButton btnBuzzer(&btnBuzzerConfig, PIN_BTN_BUZZER, HIGH);

// Declare button event handler functions
void handleBtnEventLight(AceButton *button, uint8_t eventType,
                         uint8_t buttonState);
void handleBtnEventBuzzer(AceButton *button, uint8_t eventType,
                          uint8_t buttonState);

// Define button color enumeration
enum btn_color { BTN_OFF, BTN_RED, BTN_GRN, BTN_YEL };

// Define button enumeration
enum btn { BTN_LIGHT, BTN_BUZZER };

// Define light configuration enumeration
enum light_config {
  LIGHT_OFF,
  LIGHT_STROBE_SLOW,
  LIGHT_STROBE_FAST,
  LIGHT_STEADY
};

// Define buzzer configuration enumeration
enum buzzer_config {
  BUZZER_OFF,
  BUZZER_ONCE,
  BUZZER_STROBE_SLOW,
  BUZZER_STROBE_FAST
};

// Define printout detection variables
bool printout_detected;
uint32_t printout_detected_ms;

// Define printout state enumeration
enum PrintoutState { PRINTOUT_IDLE, PRINTOUT_NEW, PRINTOUT_OLD };

PrintoutState current_printout_state =
    PRINTOUT_IDLE; // Initialize the printout state

// Define notification state enumeration
enum notificationState { NOTIFICATION_OFF, NOTIFICATION_NEW, NOTIFICATION_OLD };

// Define PinPulser objects for notification light and buzzer
PinPulser pulser_light(PIN_NOTIFICATION_LIGHT);
PinPulser pulser_buzzer(PIN_NOTIFICATION_BUZZER);

// Define calibration data structure
struct CalibrationData {
  light_config light_mode = LIGHT_STROBE_SLOW;
  buzzer_config buzzer_mode = BUZZER_ONCE;
};

// Define EEPROM calibration data structure
struct CalibrationDataEEPROM {
  CalibrationData data;
  uint16_t checksum;
};

CalibrationDataEEPROM
    calibration_data_eeprom; // Initialize the EEPROM calibration data

// Define EEPROM constants
#define EEPROM_SIZE 1024UL // Total EEPROM size for Arduino Nano
#define CALIBRATION_DATA_SIZE                                                  \
  sizeof(calibration_data_eeprom) // Size of data plus CRC
#define EEPROM_START_ADDR 1       // Start from address 1 to avoid address 0
#define EEPROM_BLOCK_NUM                                                       \
  (EEPROM_SIZE /                                                               \
   (CALIBRATION_DATA_SIZE + 1)) // Number of blocks that fit in the EEPROM

// Declare the EEWL object for unsigned long values with a CRC
EEWL eewl(calibration_data_eeprom, EEPROM_BLOCK_NUM, EEPROM_START_ADDR);

// Define feedback state enumeration
enum FeedbackState {
  FEEDBACK_IDLE,
  FEEDBACK_BEEP_ONCE,
  FEEDBACK_BEEP_STROBE_SLOW,
  FEEDBACK_BEEP_STROBE_FAST,
  FEEDBACK_LIGHT_STROBE_SLOW,
  FEEDBACK_LIGHT_STROBE_FAST,
  FEEDBACK_LIGHT_STEADY
};

FeedbackState new_feedback_state; // Initialize the new feedback state

/**
 * @brief Perform pre-task watchdog checks.
 *
 * This function checks the current watchdog state before executing a task.
 *
 * @param task The current task being executed.
 * @param expected_state The expected watchdog state before executing the task.
 */
void wdt_pre_check(TaskState task, uint16_t expected_state) {
  currentTask = task;
  if (wdt_state != expected_state || wdt_redundant_state != expected_state) {
    logEvent("State error in task, system will reset");
    haltAndWaitForWatchdog();
  }
  noInterrupts();
  wdt_state += 0x1111; // Modify wdt_state before task
  wdt_redundant_state += 0x1111;
  interrupts();
}

/**
 * @brief Perform post-task watchdog checks.
 *
 * This function checks and updates the watchdog state after executing a task.
 *
 * @param task The current task being executed.
 * @param expected_state The expected watchdog state after executing the task.
 * @return true if the watchdog state is correct after executing the task.
 * @return false if there is a state error after executing the task.
 */
bool wdt_post_check(TaskState task, uint16_t expected_state) {
  noInterrupts();
  if (wdt_state != expected_state || wdt_redundant_state != expected_state) {
    logEvent("State error in task, system will reset");
    haltAndWaitForWatchdog();
  }

  wdt_state += 0x1111; // Modify wdt_state after task
  wdt_redundant_state += 0x1111;

  uint8_t taskFlag = 1 << task;
  taskCompletionFlags |= taskFlag;
  redundantTaskCompletionFlags |= taskFlag;
  interrupts();
  return true;
}

/**
 * @brief Computes the CRC-16 checksum for a given array of data.
 *
 * This function calculates the CRC-16 checksum using a specified
 * polynomial. The CRC-16 checksum is used to detect alterations to raw
 * data, and is widely used in protocols to ensure data integrity. The
 * polynomial used in this implementation is 0x8408, which is the reverse of
 * the standard polynomial 0x1021 commonly used in CRC-16-CCITT.
 *
 * @param data Pointer to the data array over which the CRC is to be
 * calculated.
 * @param size Size of the data array in bytes.
 * @return uint16_t The computed CRC-16 checksum value.
 */
uint16_t crc16(const uint8_t *data, uint16_t size) {
  uint16_t crc = 0xFFFF; // Start with the mask 0xFFFF, which initializes the
                         // CRC register to all 1's

  // Process each byte of data in the array
  for (uint16_t i = 0; i < size; ++i) {
    crc ^= data[i]; // XOR the next data byte into the CRC register
    // Process each bit within the current byte
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        // If the LSB (Least Significant Bit) is set, apply the polynomial
        // division
        crc = (crc >> 1) ^ 0x8408; // 0x8408 is used here as the polynomial
      } else {
        // If the LSB is not set, just shift right
        crc >>= 1;
      }
    }
  }
  // After processing all data and bits, take the one's complement of the
  // calculated CRC
  return ~crc; // Return the final CRC-16 checksum
}

/**
 * @brief Saves the given calibration data structure to EEPROM and verifies the
 * write operation.
 *
 * This function writes a calibration data structure to EEPROM, incorporating
 * wear leveling via the EEWL library. It computes a checksum for the entire
 * data structure, writes the data to EEPROM, and then reads back the data to
 * verify the integrity of the write operation using a memory comparison.
 *
 * @param calData The calibration data structure to be saved.
 * @return bool Returns true if the data is successfully written and verified,
 * otherwise returns false.
 */
bool saveCalibrationValue(CalibrationDataEEPROM &cal) {
  CalibrationDataEEPROM
      read_data; // Structure to hold data read back for verification

  cal.checksum = crc16(reinterpret_cast<byte *>(&cal.data), sizeof(cal.data));

  // Write the prepared data to EEPROM using the EEWL library which handles wear
  // leveling
  eewl.put(cal);

  // Read back the data from EEPROM to verify the write operation was successful
  if (eewl.get(read_data) == 0) { // Check if the read operation was successful
    Serial.println(F("Failed EEPROM read"));
    return false; // Return false if reading from EEPROM failed
  }

  // Compare the written data with the data read back to ensure they are
  // identical
  if (memcmp(&cal, &read_data, sizeof(CalibrationDataEEPROM)) != 0) {
    Serial.println(F("EEPROM MEMCMP FAILED!"));
    return false; // Return false if the data does not match
  }

  return true; // Return true indicating the write and verification were
               // successful
}

/**
 * @brief Loads and verifies the calibration data from EEPROM.
 *
 * This function attempts to load a calibration data structure from EEPROM using
 * the EEWL library. It checks the integrity of the data using a CRC checksum.
 * If the checksum verifies correctly, it updates the passed reference variable
 * with the loaded data.
 *
 * @param calData Reference to a CalibrationData variable where the loaded
 * calibration data will be stored if successful.
 * @return bool Returns true if the calibration data is successfully read and
 * verified, false otherwise.
 */
bool loadCalibrationValue(CalibrationDataEEPROM &cal) {
  CalibrationDataEEPROM
      read_data; // Structure to hold the data read from EEPROM

  // Attempt to read calibration data from EEPROM
  if (eewl.get(read_data) == 0) {                 // Check if data is valid
    Serial.println(F("EEPROM LOAD READ FAILED")); // Log an error message if
                                                  // data reading fails
    return false; // Return false indicating no valid data could be read
  }

  // Compute the CRC checksum of the read value for verification
  uint16_t checksum =
      crc16(reinterpret_cast<byte *>(&read_data.data), sizeof(read_data.data));
  if (read_data.checksum == checksum) {
    cal = read_data; // If checksum matches, update the referenced variable with
                     // the read value
    return true;     // Return true indicating successful data verification and
                     // loading
  } else {
    Serial.println(
        F("EEPROM CRC FAILURE")); // Log an error message if CRC check fails
    return false;                 // Return false indicating a CRC error
  }
}

/**
 * @brief Resets calibration data to default values.
 *
 * This function sets all calibration coefficients and constants in the provided
 * CalibrationDataEEPROM structure to predefined default values. These defaults
 * are usually set as placeholders indicating that calibration is required.
 *
 * @param cal Reference to the CalibrationDataEEPROM structure containing
 * calibration data.
 * @return uint8_t Always returns TEST_SUCCESS, indicating the function executed
 * successfully.
 */
uint8_t load_cal_defaults(CalibrationDataEEPROM &cal) {
  // Set the BANDGAP value to a default, typically indicating uncalibrated
  // state.
  cal.data.buzzer_mode = BUZZER_ONCE;
  cal.data.light_mode = LIGHT_STROBE_SLOW;

  return true; // Indicate successful reset.
}

/**
 * @brief Sets the color of the specified button's LED.
 *
 * This function controls the LED colors for the specified button (either
 * BTN_LIGHT or BTN_BUZZER). It sets the LED to off, red, green, or yellow based
 * on the provided color parameter.
 *
 * @param button The button whose LED color is to be set. Can be BTN_LIGHT or
 * BTN_BUZZER.
 * @param color The color to set the button's LED to. Can be BTN_OFF, BTN_RED,
 * BTN_GRN, or BTN_YEL.
 */
void set_button_color(btn button, btn_color color) {
  switch (button) {
  case BTN_LIGHT:
    switch (color) {
    case BTN_OFF:
      // Turn off both red and green LEDs for BTN_LIGHT
      digitalWrite(PIN_BTN_LIGHT_LED_GRN, HIGH);
      digitalWrite(PIN_BTN_LIGHT_LED_RED, HIGH);
      break;

    case BTN_RED:
      // Turn on red LED and turn off green LED for BTN_LIGHT
      digitalWrite(PIN_BTN_LIGHT_LED_RED, LOW);
      digitalWrite(PIN_BTN_LIGHT_LED_GRN, HIGH);
      break;

    case BTN_GRN:
      // Turn off red LED and turn on green LED for BTN_LIGHT
      digitalWrite(PIN_BTN_LIGHT_LED_RED, HIGH);
      digitalWrite(PIN_BTN_LIGHT_LED_GRN, LOW);
      break;

    case BTN_YEL:
      // Turn off both red and green LEDs for BTN_LIGHT
      digitalWrite(PIN_BTN_LIGHT_LED_RED, HIGH);
      digitalWrite(PIN_BTN_LIGHT_LED_GRN, HIGH);
      break;
    }
    break;

  case BTN_BUZZER:
    switch (color) {
    case BTN_OFF:
      // Turn off both red and green LEDs for BTN_BUZZER
      digitalWrite(PIN_BTN_BUZZER_LED_GRN, HIGH);
      digitalWrite(PIN_BTN_BUZZER_LED_RED, HIGH);
      break;

    case BTN_RED:
      // Turn on red LED and turn off green LED for BTN_BUZZER
      digitalWrite(PIN_BTN_BUZZER_LED_RED, LOW);
      digitalWrite(PIN_BTN_BUZZER_LED_GRN, HIGH);
      break;

    case BTN_GRN:
      // Turn off red LED and turn on green LED for BTN_BUZZER
      digitalWrite(PIN_BTN_BUZZER_LED_RED, HIGH);
      digitalWrite(PIN_BTN_BUZZER_LED_GRN, LOW);
      break;

    case BTN_YEL:
      // Turn off both red and green LEDs for BTN_BUZZER
      digitalWrite(PIN_BTN_BUZZER_LED_RED, HIGH);
      digitalWrite(PIN_BTN_BUZZER_LED_GRN, HIGH);
      break;
    }
    break;
  }
}

/**
 * @brief Sets the new feedback state.
 *
 * This function updates the global variable `new_feedback_state` with the
 * specified feedback state.
 *
 * @param new_feedback The new feedback state to set.
 */
void set_feedback_state(FeedbackState new_feedback) {
  // Update the global feedback state with the new state
  new_feedback_state = new_feedback;
}

/**
 * @brief Handles the feedback mechanisms (buzzer and light) based on the
 * current feedback state.
 *
 * This function manages the feedback state machine, controlling the
 * notification buzzer and light according to the defined feedback patterns. It
 * also ensures watchdog timer state transitions and validates the system state.
 *
 * @return true if the function executes successfully without state errors.
 * @return false if there is a state error and the system needs to reset.
 */
bool handle_feedback() {

  wdt_pre_check(TASK4, 0xBBBB);

  // Static variables to maintain feedback state and timing
  static FeedbackState feedbackState = FEEDBACK_IDLE;
  static unsigned long feedbackStartTime = 0;
  static int feedbackCounter = 0;
  unsigned long currentTime = millis();

  // Check if a new feedback state has been set and initialize the feedback
  // cycle
  if (new_feedback_state != FEEDBACK_IDLE && feedbackState == FEEDBACK_IDLE) {
    feedbackState = new_feedback_state;
    new_feedback_state = FEEDBACK_IDLE;
    feedbackStartTime = millis();
    feedbackCounter = 0;
  }

  // State machine to handle different feedback patterns
  switch (feedbackState) {
  case FEEDBACK_IDLE:
    // No feedback action required
    break;

  case FEEDBACK_BEEP_ONCE:
    if (currentTime - feedbackStartTime < 100) {
      digitalWrite(PIN_NOTIFICATION_BUZZER, HIGH); // Turn buzzer on
    } else if (currentTime - feedbackStartTime < 300) {
      digitalWrite(PIN_NOTIFICATION_BUZZER, LOW); // Turn buzzer off
    } else {
      feedbackState = FEEDBACK_IDLE; // End feedback cycle
    }
    break;

  case FEEDBACK_BEEP_STROBE_SLOW:
    if (currentTime - feedbackStartTime < 400) {
      digitalWrite(PIN_NOTIFICATION_BUZZER, HIGH); // Turn buzzer on
    } else if (currentTime - feedbackStartTime < 800) {
      digitalWrite(PIN_NOTIFICATION_BUZZER, LOW); // Turn buzzer off
    } else {
      feedbackStartTime = currentTime;
      feedbackCounter++;
      if (feedbackCounter >= 3) {
        feedbackState = FEEDBACK_IDLE; // End feedback cycle after 3 pulses
      }
    }
    break;

  case FEEDBACK_BEEP_STROBE_FAST:
    if (currentTime - feedbackStartTime < 200) {
      digitalWrite(PIN_NOTIFICATION_BUZZER, HIGH); // Turn buzzer on
    } else if (currentTime - feedbackStartTime < 400) {
      digitalWrite(PIN_NOTIFICATION_BUZZER, LOW); // Turn buzzer off
    } else {
      feedbackStartTime = currentTime;
      feedbackCounter++;
      if (feedbackCounter >= 3) {
        feedbackState = FEEDBACK_IDLE; // End feedback cycle after 3 pulses
      }
    }
    break;

  case FEEDBACK_LIGHT_STROBE_SLOW:
    if (currentTime - feedbackStartTime < 400) {
      digitalWrite(PIN_NOTIFICATION_LIGHT, HIGH); // Turn light on
    } else if (currentTime - feedbackStartTime < 800) {
      digitalWrite(PIN_NOTIFICATION_LIGHT, LOW); // Turn light off
    } else {
      feedbackStartTime = currentTime;
      feedbackCounter++;
      if (feedbackCounter >= 3) {
        feedbackState = FEEDBACK_IDLE; // End feedback cycle after 3 pulses
      }
    }
    break;

  case FEEDBACK_LIGHT_STROBE_FAST:
    if (currentTime - feedbackStartTime < 200) {
      digitalWrite(PIN_NOTIFICATION_LIGHT, HIGH); // Turn light on
    } else if (currentTime - feedbackStartTime < 400) {
      digitalWrite(PIN_NOTIFICATION_LIGHT, LOW); // Turn light off
    } else {
      feedbackStartTime = currentTime;
      feedbackCounter++;
      if (feedbackCounter >= 3) {
        feedbackState = FEEDBACK_IDLE; // End feedback cycle after 3 pulses
      }
    }
    break;

  case FEEDBACK_LIGHT_STEADY:
    if (currentTime - feedbackStartTime < 2000) {
      digitalWrite(PIN_NOTIFICATION_LIGHT, HIGH); // Turn light on
    } else {
      digitalWrite(PIN_NOTIFICATION_LIGHT, LOW); // Turn light off
      feedbackState = FEEDBACK_IDLE;             // End feedback cycle
    }
    break;
  }

  return wdt_post_check(TASK4, 0xCCCC);
}

/**
 * @brief Increases the buzzer mode to the next state and updates the feedback
 * state accordingly.
 *
 * This function cycles through the buzzer modes in the following order:
 * BUZZER_OFF -> BUZZER_ONCE -> BUZZER_STROBE_SLOW -> BUZZER_STROBE_FAST ->
 * BUZZER_OFF. It also updates the feedback state to reflect the current buzzer
 * mode.
 */
void increase_buzzer_mode() {
  // Switch based on the current buzzer mode stored in EEPROM
  switch (calibration_data_eeprom.data.buzzer_mode) {
  case BUZZER_OFF:
    // Transition from BUZZER_OFF to BUZZER_ONCE
    set_buzzer_mode(BUZZER_ONCE);
    set_feedback_state(FEEDBACK_BEEP_ONCE);
    break;

  case BUZZER_ONCE:
    // Transition from BUZZER_ONCE to BUZZER_STROBE_SLOW
    set_buzzer_mode(BUZZER_STROBE_SLOW);
    set_feedback_state(FEEDBACK_BEEP_STROBE_SLOW);
    break;

  case BUZZER_STROBE_SLOW:
    // Transition from BUZZER_STROBE_SLOW to BUZZER_STROBE_FAST
    set_buzzer_mode(BUZZER_STROBE_FAST);
    set_feedback_state(FEEDBACK_BEEP_STROBE_FAST);
    break;

  default:
    // Reset to BUZZER_OFF and feedback state to IDLE
    set_buzzer_mode(BUZZER_OFF);
    set_feedback_state(FEEDBACK_IDLE);
    break;
  }
}

/**
 * @brief Increases the light mode to the next state and updates the feedback
 * state accordingly.
 *
 * This function cycles through the light modes in the following order:
 * LIGHT_OFF -> LIGHT_STROBE_SLOW -> LIGHT_STROBE_FAST -> LIGHT_STEADY ->
 * LIGHT_OFF. It also updates the feedback state to reflect the current light
 * mode.
 */
void increase_light_mode() {
  // Switch based on the current light mode stored in EEPROM
  switch (calibration_data_eeprom.data.light_mode) {
  case LIGHT_OFF:
    // Transition from LIGHT_OFF to LIGHT_STROBE_SLOW
    set_light_mode(LIGHT_STROBE_SLOW);
    set_feedback_state(FEEDBACK_LIGHT_STROBE_SLOW);
    break;

  case LIGHT_STROBE_SLOW:
    // Transition from LIGHT_STROBE_SLOW to LIGHT_STROBE_FAST
    set_light_mode(LIGHT_STROBE_FAST);
    set_feedback_state(FEEDBACK_LIGHT_STROBE_FAST);
    break;

  case LIGHT_STROBE_FAST:
    // Transition from LIGHT_STROBE_FAST to LIGHT_STEADY
    set_light_mode(LIGHT_STEADY);
    set_feedback_state(FEEDBACK_LIGHT_STEADY);
    break;

  default:
    // Reset to LIGHT_OFF and feedback state to IDLE
    set_light_mode(LIGHT_OFF);
    set_feedback_state(FEEDBACK_IDLE);
    break;
  }
}

/**
 * @brief Configures the event handlers and features for the buzzer and light
 * buttons.
 *
 * This function sets up the event handlers and enables the click and long press
 * features for both the buzzer and light buttons.
 */
void config_buttons() {
  // Configure the buzzer button
  btnBuzzerConfig.setEventHandler(
      handleBtnEventBuzzer); // Set event handler for buzzer button events
  btnBuzzerConfig.setFeature(
      ButtonConfig::kFeatureClick); // Enable click feature for buzzer button
  btnBuzzerConfig.setFeature(
      ButtonConfig::kFeatureLongPress); // Enable long press feature for buzzer
                                        // button

  // Configure the light button
  btnLightConfig.setEventHandler(
      handleBtnEventLight); // Set event handler for light button events
  btnLightConfig.setFeature(
      ButtonConfig::kFeatureClick); // Enable click feature for light button
  btnLightConfig.setFeature(
      ButtonConfig::kFeatureLongPress); // Enable long press feature for light
                                        // button
}

/**
 * @brief Checks the state of the printout button with debouncing and updates
 * the printout state.
 *
 * This function reads the printout button state, applies debouncing, and
 * updates the printout state. It ensures that the watchdog timer state
 * transitions are performed correctly.
 *
 * @return true if the function executes successfully without state errors.
 * @return false if there is a state error and the system needs to reset.
 */
bool check_printout() {
  // Static variables to maintain the state and timing for debouncing
  static int printout_state = LOW;
  static int last_state = LOW;
  static unsigned long lastDebounceTime = 0;
  static unsigned long printout_time = 0;

  wdt_pre_check(TASK2, 0x5555);

  // Read the current state of the printout button
  int reading = digitalRead(PIN_PRINTOUT);

  // Check if the button state has changed
  if (reading != last_state) {
    lastDebounceTime = millis(); // Reset the debouncing timer
  }

  // If the button state is stable for longer than the debounce delay
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // Check if the stable state is different from the current state
    if (reading != printout_state) {
      printout_state = reading; // Update the current state

      if (printout_state == HIGH) {
        // Button pressed
        printout_time = millis();
        current_printout_state = PRINTOUT_IDLE;
      } else {
        // Button released
        unsigned long pressDuration = millis() - printout_time;

        // Determine if the printout is new or old based on the duration
        if (pressDuration >= PRINTOUT_OLD_MS) {
          current_printout_state = PRINTOUT_OLD;
        } else {
          current_printout_state = PRINTOUT_NEW;
        }
      }
    }
  }

  // Update the last state
  last_state = reading;

  return wdt_post_check(TASK2, 0x6666);
}

/**
 * @brief Retrieves the current printout state.
 *
 * This function returns the current state of the printout as determined by the
 * `check_printout` function.
 *
 * @return The current printout state.
 */
PrintoutState get_printout_state() {
  // Return the current state of the printout
  return current_printout_state;
}

/**
 * @brief Sets the light mode and updates the corresponding configurations.
 *
 * This function sets the light mode to the specified configuration. It updates
 * the EEPROM if the mode has changed, sets the button color, and configures the
 * light pulser accordingly.
 *
 * @param mode The desired light configuration mode.
 */
void set_light_mode(light_config mode) {
  // Check if the new mode is different from the current mode
  if (mode != calibration_data_eeprom.data.light_mode) {
    calibration_data_eeprom.data.light_mode =
        mode; // Update the light mode in EEPROM
    saveCalibrationValue(
        calibration_data_eeprom); // Save the updated calibration value to
                                  // EEPROM
  }

  // Set the light mode and configure the pulser based on the mode
  switch (mode) {
  case LIGHT_OFF:
    // Set button color to off and disable the pulser
    set_button_color(BTN_LIGHT, BTN_OFF);
    pulser_light.configure(STROBE_SLOW_ON_MS, STROBE_SLOW_OFF_MS,
                           PinPulser::DISABLED);
    break;

  case LIGHT_STROBE_SLOW:
    // Set button color to red and configure the pulser for slow strobe
    set_button_color(BTN_LIGHT, BTN_RED);
    pulser_light.configure(STROBE_SLOW_ON_MS, STROBE_SLOW_OFF_MS,
                           PinPulser::PULSE);
    break;

  case LIGHT_STROBE_FAST:
    // Set button color to yellow and configure the pulser for fast strobe
    set_button_color(BTN_LIGHT, BTN_YEL);
    pulser_light.configure(STROBE_FAST_ON_MS, STROBE_FAST_OFF_MS,
                           PinPulser::PULSE);
    break;

  case LIGHT_STEADY:
    // Set button color to green and configure the pulser for steady state
    set_button_color(BTN_LIGHT, BTN_GRN);
    pulser_light.configure(STROBE_SLOW_ON_MS, STROBE_SLOW_OFF_MS,
                           PinPulser::STEADY_STATE);
    break;
  }
}

/**
 * @brief Sets the buzzer mode and updates the corresponding configurations.
 *
 * This function sets the buzzer mode to the specified configuration. It updates
 * the EEPROM if the mode has changed, sets the button color, and configures the
 * buzzer pulser accordingly.
 *
 * @param mode The desired buzzer configuration mode.
 */
void set_buzzer_mode(buzzer_config mode) {
  // Check if the new mode is different from the current mode
  if (mode != calibration_data_eeprom.data.buzzer_mode) {
    calibration_data_eeprom.data.buzzer_mode =
        mode; // Update the buzzer mode in EEPROM
    saveCalibrationValue(
        calibration_data_eeprom); // Save the updated calibration value to
                                  // EEPROM
  }

  // Set the buzzer mode and configure the pulser based on the mode
  switch (mode) {
  case BUZZER_OFF:
    // Set button color to off and disable the pulser
    set_button_color(BTN_BUZZER, BTN_OFF);
    pulser_buzzer.configure(BUZZER_ONCE_ON_MS, STROBE_SLOW_OFF_MS,
                            PinPulser::DISABLED);
    break;

  case BUZZER_ONCE:
    // Set button color to red and configure the pulser for a single beep
    set_button_color(BTN_BUZZER, BTN_RED);
    pulser_buzzer.configure(BUZZER_ONCE_ON_MS, STROBE_SLOW_OFF_MS,
                            PinPulser::ONE_SHOT);
    break;

  case BUZZER_STROBE_SLOW:
    // Set button color to yellow and configure the pulser for slow strobe
    set_button_color(BTN_BUZZER, BTN_YEL);
    pulser_buzzer.configure(STROBE_SLOW_ON_MS, STROBE_SLOW_OFF_MS,
                            PinPulser::PULSE);
    break;

  case BUZZER_STROBE_FAST:
    // Set button color to green and configure the pulser for fast strobe
    set_button_color(BTN_BUZZER, BTN_GRN);
    pulser_buzzer.configure(STROBE_FAST_ON_MS, STROBE_FAST_OFF_MS,
                            PinPulser::PULSE);
    break;
  }
}

/**
 * @brief Sets up the initial configuration for the system.
 *
 * This function initializes the pin modes, sets initial states for LEDs and
 * buttons, configures the buttons, loads calibration values from EEPROM, and
 * sets the buzzer and light modes based on the loaded calibration data. It also
 * enables the watchdog timer.
 */
void setup() {
  currentTask = SETUP; // Set the current task to SETUP
  uint8_t result;

  // Set pin modes for buttons and LEDs
  pinMode(PIN_BTN_LIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_BUZZER, INPUT_PULLUP);
  pinMode(PIN_PRINTOUT, INPUT);
  pinMode(PIN_BTN_BUZZER_LED_GRN, OUTPUT);
  pinMode(PIN_BTN_BUZZER_LED_RED, OUTPUT);
  pinMode(PIN_BTN_LIGHT_LED_GRN, OUTPUT);
  pinMode(PIN_BTN_LIGHT_LED_RED, OUTPUT);
  pinMode(PIN_POWER_LED, OUTPUT);
  pinMode(PIN_NOTIFICATION_LIGHT, OUTPUT);
  pinMode(PIN_NOTIFICATION_BUZZER, OUTPUT);

  // Initialize button LEDs to off
  set_button_color(BTN_BUZZER, BTN_OFF);
  set_button_color(BTN_LIGHT, BTN_OFF);

  // Initialize power and notification LEDs
  digitalWrite(PIN_POWER_LED, HIGH);          // Turn on power LED
  digitalWrite(PIN_NOTIFICATION_BUZZER, LOW); // Turn off notification buzzer
  digitalWrite(PIN_NOTIFICATION_LIGHT, LOW);  // Turn off notification light

  // Configure buttons with event handlers
  config_buttons();

  // Load calibration values from EEPROM
  result = loadCalibrationValue(calibration_data_eeprom);

  // Check if loading EEPROM values was successful
  if (result != true) {
    Serial.println(F("FAILED to load EEPROM values!"));

    // Load default calibration values and save them to EEPROM
    load_cal_defaults(calibration_data_eeprom);
    saveCalibrationValue(calibration_data_eeprom);
  }

  // Set buzzer and light modes based on loaded calibration data
  set_buzzer_mode(calibration_data_eeprom.data.buzzer_mode);
  set_light_mode(calibration_data_eeprom.data.light_mode);

  // Enable watchdog timer with a 2-second timeout
  wdt_enable(WDTO_2S);
}

/**
 * @brief Handles the notification state and updates the buzzer and light
 * pulsers accordingly.
 *
 * This function manages the state of notifications, updating the buzzer and
 * light pulsers based on the current notification state. It also updates
 * configurations when transitioning between states.
 *
 * @param notification_state The desired notification state to set.
 */
void notification(notificationState notification_state) {
  // Static variable to keep track of the previous notification state
  static notificationState prev_state;

  // Handle the current notification state
  switch (notification_state) {
  case NOTIFICATION_OFF:
    // Turn off both the buzzer and light pulsers
    pulser_buzzer.update(false);
    pulser_light.update(false);
    prev_state = notification_state; // Update previous state

    // Reconfigure buzzer and light pulsers based on EEPROM settings if
    // necessary
    if (calibration_data_eeprom.data.buzzer_mode == BUZZER_STROBE_SLOW) {
      pulser_buzzer.configure(STROBE_SLOW_ON_MS, STROBE_SLOW_OFF_MS,
                              PinPulser::PULSE);
    }
    if (calibration_data_eeprom.data.light_mode == LIGHT_STROBE_SLOW) {
      pulser_light.configure(STROBE_SLOW_ON_MS, STROBE_SLOW_OFF_MS,
                             PinPulser::PULSE);
    }
    if (calibration_data_eeprom.data.light_mode == LIGHT_STEADY) {
      pulser_light.configure(STROBE_SLOW_ON_MS, STROBE_SLOW_OFF_MS,
                             PinPulser::STEADY_STATE);
    }
    break;

  case NOTIFICATION_NEW:
    // Turn on both the buzzer and light pulsers
    pulser_buzzer.update(true);
    pulser_light.update(true);
    prev_state = notification_state; // Update previous state
    break;

  case NOTIFICATION_OLD:
    // Check if transitioning from a different state to update configurations if
    // necessary
    if (prev_state != notification_state &&
        calibration_data_eeprom.data.buzzer_mode == BUZZER_STROBE_SLOW) {
      pulser_buzzer.configure(STROBE_FAST_ON_MS, STROBE_FAST_OFF_MS,
                              PinPulser::PULSE);
    }
    if (prev_state != notification_state &&
        (calibration_data_eeprom.data.light_mode == LIGHT_STROBE_SLOW ||
         calibration_data_eeprom.data.light_mode == LIGHT_STEADY)) {
      pulser_light.configure(STROBE_FAST_ON_MS, STROBE_FAST_OFF_MS,
                             PinPulser::PULSE);
    }

    // Turn on both the buzzer and light pulsers
    pulser_buzzer.update(true);
    pulser_light.update(true);
    prev_state = notification_state; // Update previous state
    break;

  default:
    // Default case to turn on both the buzzer and light pulsers
    pulser_buzzer.update(true);
    pulser_light.update(true);
    prev_state = notification_state; // Update previous state
  }
}

/**
 * @brief Handles the printout state and updates notifications accordingly.
 *
 * This function processes the current printout state and updates the
 * notification system. It ensures that the watchdog timer state transitions are
 * performed correctly.
 *
 * @return true if the function executes successfully without state errors.
 * @return false if there is a state error and the system needs to reset.
 */
bool handle_printout() {
  wdt_pre_check(TASK3, 0x8888);

  // Get the current printout state
  PrintoutState printout = get_printout_state();

  // Update notification system based on the printout state
  switch (printout) {
  case PRINTOUT_IDLE:
    // If the printout state is idle, turn off notifications
    notification(NOTIFICATION_OFF);
    break;

  case PRINTOUT_NEW:
    // If there is a new printout, set notification to new
    notification(NOTIFICATION_NEW);
    break;

  case PRINTOUT_OLD:
    // If the printout is old, set notification to old
    notification(NOTIFICATION_OLD);
    break;
  }

  return wdt_post_check(TASK3, 0x9999);
}

/**
 * @brief Checks the states of the buzzer and light buttons.
 *
 * This function checks the states of the buzzer and light buttons and ensures
 * that the watchdog timer state transitions are performed correctly. It
 * verifies and updates the watchdog state before and after checking the
 * buttons.
 *
 * @return true if the function executes successfully without state errors.
 * @return false if there is a state error and the system needs to reset.
 */
bool check_btns() {
  wdt_pre_check(TASK1, 0x2222);

  // Task body: Check the states of the buzzer and light buttons
  btnBuzzer.check();
  btnLight.check();

  return wdt_post_check(TASK1, 0x3333);
}

/**
 * @brief Logs an event message.
 *
 * This function prints an event message to the serial monitor. It can be used
 * to log important events or error messages for debugging and monitoring
 * purposes.
 *
 * @param event The event message to be logged.
 */
void logEvent(const char *event) {
  // Log the event message to the serial monitor
  Serial.println(event);
}

/**
 * @brief Checks the initial watchdog timer state at the start of the loop.
 *
 * This function verifies that the watchdog timer state, task state, and
 * completion flags are in their expected initial states at the start of the
 * main loop. If there is a state error, it logs an error message and halts the
 * system to wait for a watchdog reset.
 */
void wdt_loop_start_check() {
  noInterrupts(); // Disable interrupts to ensure atomic operations

  // Check if the watchdog state, task state, and completion flags are in their
  // expected initial states
  if (wdt_state != 0x1111 || wdt_redundant_state != 0x1111 ||
      currentTask != LOOP || taskCompletionFlags != 0 ||
      redundantTaskCompletionFlags != 0) {
    errorCode = STATE_ERROR; // Set the error code to STATE_ERROR
    logEvent(
        "State error in wdt_loop_start_check, system will reset"); // Log an
                                                                   // error
                                                                   // message
    haltAndWaitForWatchdog(); // Halt the system to wait for a watchdog reset
  }

  interrupts(); // Re-enable interrupts
}

/**
 * @brief Checks the final watchdog timer state at the end of the loop.
 *
 * This function verifies that the watchdog timer state and task completion
 * flags are in their expected final states at the end of the main loop. If
 * there is a state error, it logs an error message and halts the system to wait
 * for a watchdog reset. It also resets the watchdog timer state and task
 * completion flags for the next loop iteration.
 */
void wdt_loop_end_check() {
  noInterrupts(); // Disable interrupts to ensure atomic operations

  // Check if the watchdog state and task completion flags are in their expected
  // final states
  if (wdt_state != 0xEEEE || wdt_redundant_state != 0xEEEE ||
      taskCompletionFlags != 0b1111 || redundantTaskCompletionFlags != 0b1111) {
    errorCode = STATE_ERROR; // Set the error code to STATE_ERROR
    logEvent(
        "State error in wdt_loop_end_check, system will reset"); // Log an error
                                                                 // message
    haltAndWaitForWatchdog(); // Halt the system to wait for a watchdog reset
  }

  // Reset watchdog state and task completion flags for the next loop iteration
  wdt_state = 0;
  wdt_redundant_state = 0;
  taskCompletionFlags = 0;
  redundantTaskCompletionFlags = 0;

  interrupts(); // Re-enable interrupts
  wdt_reset();  // Reset the watchdog timer
}

/**
 * @brief Halts the system and waits for the watchdog timer to reset it.
 *
 * This inline function disables interrupts and enters an infinite loop,
 * effectively halting the system. The watchdog timer will eventually reset the
 * system.
 */
inline void haltAndWaitForWatchdog() {
  cli(); // Disable interrupts
  while (true) {
    // Wait for the watchdog timer to reset the system
  }
}

/**
 * @brief Checks if the task execution time is within the specified limits.
 *
 * This function calculates the difference between the start and end times of a
 * task execution and checks if it falls within the specified minimum and
 * maximum limits. If the task time is out of bounds, it logs an error message
 * and halts the system to wait for a watchdog reset.
 *
 * @param start The start time of the task.
 * @param end The end time of the task.
 * @param min The minimum allowed task execution time.
 * @param max The maximum allowed task execution time.
 * @return true if the task execution time is within the specified limits.
 * @return false if the task execution time is out of bounds (this function will
 * not return in this case).
 */
bool check_task_time(uint32_t start, uint32_t end, uint32_t min, uint32_t max) {
  // Calculate the difference between the end and start times
  uint32_t difference = end - start;

  // Check if the task execution time is within the specified limits
  if (difference < min || difference > max) {
    // Log an error message if the task time is out of bounds
    Serial.print(F("Timing error for task:"));
    Serial.print(currentTask);
    Serial.println(F(". System waiting for reset"));
    haltAndWaitForWatchdog(); // Halt the system to wait for a watchdog reset
  }

  return true; // Return true if the task execution time is within the limits
}

/**
 * @brief Prepares the watchdog timer state before executing a task.
 *
 * This function updates the watchdog timer state and records the start time
 * of the task. It ensures that these operations are performed atomically by
 * disabling interrupts.
 */
void wdt_before_task() {
  noInterrupts();      // Disable interrupts to ensure atomic operations
  wdt_state += 0x1111; // Modify the watchdog state before the task
  wdt_redundant_state += 0x1111; // Modify the redundant watchdog state
  wdt_start_time = millis();     // Record the start time of the task
  interrupts();                  // Re-enable interrupts
}

/**
 * @brief Finalizes the watchdog timer state after executing a task.
 *
 * This function records the end time of the task, calculates the task execution
 * time, updates the minimum and maximum times for the task, and checks if the
 * task execution time is within the specified limits. If the task time is out
 * of bounds, it logs an error message and halts the system to wait for a
 * watchdog reset.
 *
 * @param minTime The minimum allowed task execution time.
 * @param maxTime The maximum allowed task execution time.
 * @param taskIndex The index of the current task.
 */
void wdt_after_task(unsigned long minTime, unsigned long maxTime,
                    int taskIndex) {
  uint32_t taskTime;

  noInterrupts();          // Disable interrupts to ensure atomic operations
  wdt_end_time = millis(); // Record the end time of the task

  taskTime = wdt_end_time - wdt_start_time; // Calculate the task execution time

  // Update minimum and maximum times for the task
  if (taskTime < wdt_min_times[taskIndex]) {
    wdt_min_times[taskIndex] = taskTime;
  }
  if (taskTime > wdt_max_times[taskIndex]) {
    wdt_max_times[taskIndex] = taskTime;
  }

  // Check if the task execution time is within the specified limits
  if (taskTime < minTime || taskTime > maxTime) {
    logEvent("Timing error, system will reset"); // Log an error message
    haltAndWaitForWatchdog(); // Halt the system to wait for a watchdog reset
  }

  interrupts(); // Re-enable interrupts
}

/**
 * @brief Calls a task function with watchdog timer preparation and
 * finalization.
 *
 * This function wraps the execution of a task with pre-task and post-task
 * watchdog timer checks. It prepares the watchdog timer state before executing
 * the task, calls the task function, and then finalizes the watchdog timer
 * state after the task. If the task fails or the execution time is out of
 * bounds, it halts the system to wait for a watchdog reset.
 *
 * @param task The task function to be called.
 * @param minTime The minimum allowed task execution time.
 * @param maxTime The maximum allowed task execution time.
 * @param taskIndex The index of the current task.
 */
void task_caller(bool (*task)(), unsigned long minTime, unsigned long maxTime,
                 int taskIndex) {
  wdt_before_task(); // Prepare the watchdog timer state before the task
  if (!task()) {
    haltAndWaitForWatchdog(); // Halt the system if the task fails
  }
  wdt_after_task(minTime, maxTime,
                 taskIndex); // Finalize the watchdog timer state after the task
}

/**
 * @brief Initializes the watchdog timer and task states at the start of the
 * loop.
 *
 * This function sets the initial values for the watchdog timer state, the
 * current task, and the task completion flags. It prepares the system for a new
 * loop iteration.
 */
void wdt_loop_start() {
  wdt_state = 0x1111;               // Initialize watchdog state
  currentTask = LOOP;               // Set the current task to LOOP
  wdt_redundant_state = 0x1111;     // Initialize redundant watchdog state
  taskCompletionFlags = 0;          // Reset task completion flags
  redundantTaskCompletionFlags = 0; // Reset redundant task completion flags
}

/**
 * @brief Finalizes the watchdog timer and task states at the end of the loop.
 *
 * This function updates the watchdog timer state, sets the current task to
 * COMPLETE, and re-enables interrupts. It prepares the system to complete the
 * current loop iteration.
 */
void wdt_loop_end() {
  noInterrupts();      // Disable interrupts to ensure atomic operations
  wdt_state += 0x1111; // Modify the watchdog state before completing the loop
  wdt_redundant_state += 0x1111; // Modify the redundant watchdog state
  currentTask = COMPLETE;        // Set the current task to COMPLETE
  interrupts();                  // Re-enable interrupts
}

/**
 * @brief Prints the minimum and maximum execution times for each task.
 *
 * This function iterates through the stored minimum and maximum execution times
 * for each task and prints them to the serial monitor. It helps in analyzing
 * the performance and timing of tasks.
 */
void printTaskTimings() {
  Serial.println("Task Timings:"); // Print header

  // Iterate through the tasks and print their min and max execution times
  for (int i = 0; i < 4; i++) {
    Serial.print("Task ");
    Serial.print(i + 1); // Print task number
    Serial.print(": Min Time = ");
    Serial.print(wdt_min_times[i]); // Print minimum execution time for the task
    Serial.print(" ms, Max Time = ");
    Serial.print(wdt_max_times[i]); // Print maximum execution time for the task
    Serial.println(" ms");          // Print units and end the line
  }
}

/**
 * @brief Main loop of the program.
 *
 * This function is the main loop of the program. It initializes and checks the
 * watchdog timer, executes various tasks, and finalizes the watchdog timer
 * state. It also prints task timings for design tuning.
 */
void loop() {
  wdt_loop_start(); // Initialize watchdog state and task completion flags

  wdt_loop_start_check(); // Perform initial watchdog state check

  // Call tasks using the task_caller function
  task_caller(check_btns, TASK1_MIN_TIME, TASK1_MAX_TIME,
              0); // Task 1: Check buttons
  task_caller(check_printout, TASK2_MIN_TIME, TASK2_MAX_TIME,
              1); // Task 2: Check printout
  task_caller(handle_printout, TASK3_MIN_TIME, TASK3_MAX_TIME,
              2); // Task 3: Handle printout
  task_caller(handle_feedback, TASK4_MIN_TIME, TASK4_MAX_TIME,
              3); // Task 4: Handle feedback

  wdt_loop_end(); // Finalize watchdog state and set current task to COMPLETE

  // Print task timings for design tuning
  printTaskTimings();

  // wdt_state is 0xEEEE

  wdt_loop_end_check(); // Perform final watchdog state check and reset
}

/**
 * @brief Handles the event for the light button.
 *
 * This function handles the events triggered by the light button. It processes
 * the event type and updates the light mode accordingly. For a press event,
 * it increases the light mode. For a long press event, it sets the light mode
 * to off.
 *
 * @param button The pointer to the AceButton object that triggered the event.
 * @param eventType The type of event that occurred (e.g., pressed, long
 * pressed).
 * @param buttonState The state of the button when the event occurred.
 */
void handleBtnEventLight(AceButton *button, uint8_t eventType,
                         uint8_t buttonState) {
  // Handle different types of button events
  switch (eventType) {
  case AceButton::kEventPressed:
    // Increase the light mode on press
    increase_light_mode();
    break;
  case AceButton::kEventLongPressed:
    // Set the light mode to off on long press
    set_light_mode(LIGHT_OFF);
    break;
  }
}

/**
 * @brief Handles the event for the buzzer button.
 *
 * This function handles the events triggered by the buzzer button. It processes
 * the event type and updates the buzzer mode accordingly. For a press event,
 * it increases the buzzer mode. For a long press event, it sets the buzzer mode
 * to off.
 *
 * @param button The pointer to the AceButton object that triggered the event.
 * @param eventType The type of event that occurred (e.g., pressed, long
 * pressed).
 * @param buttonState The state of the button when the event occurred.
 */
void handleBtnEventBuzzer(AceButton *button, uint8_t eventType,
                          uint8_t buttonState) {
  // Handle different types of button events
  switch (eventType) {
  case AceButton::kEventPressed:
    // Increase the buzzer mode on press
    increase_buzzer_mode();
    break;
  case AceButton::kEventLongPressed:
    // Set the buzzer mode to off on long press
    set_buzzer_mode(BUZZER_OFF);
    break;
  }
}
