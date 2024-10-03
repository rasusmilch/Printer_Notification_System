// PinPulser.cpp

#include "PinPulser.h"

/**
 * @brief Constructs a PinPulser object to control a pin with pulsing behavior.
 *
 * This constructor initializes a PinPulser object with the specified pin. It
 * sets the initial pin state, timing parameters, mode, and sets the pin mode to
 * OUTPUT.
 *
 * @param pin The pin number to be controlled by the PinPulser.
 */
PinPulser::PinPulser(int pin)
    : pin(pin), pinState(LOW), lastChangeTime(0), highTime(0), lowTime(0),
      mode(DISABLED), oneShotComplete(false) {
  pinMode(pin, OUTPUT);   // Set the pin mode to OUTPUT
  digitalWrite(pin, LOW); // Initialize the pin state to LOW
}

/**
 * @brief Configures the timing and mode for the PinPulser.
 *
 * This function sets the high time, low time, and operating mode for the
 * PinPulser. These parameters control how the pin will be pulsed.
 *
 * @param highTime The duration (in milliseconds) for which the pin will stay
 * HIGH.
 * @param lowTime The duration (in milliseconds) for which the pin will stay
 * LOW.
 * @param mode The operating mode of the PinPulser (e.g., DISABLED, PULSE,
 * ONE_SHOT, STEADY_STATE).
 */
void PinPulser::configure(unsigned long highTime, unsigned long lowTime,
                          Mode mode) {
  this->highTime = highTime; // Set the high time duration
  this->lowTime = lowTime;   // Set the low time duration
  this->mode = mode;         // Set the operating mode
}

/**
 * @brief Updates the state of the pin based on the current mode and timing
 * parameters.
 *
 * This function updates the pin state according to the configured mode
 * (DISABLED, PULSE, ONE_SHOT, or STEADY_STATE) and the high and low timing
 * parameters. It is called repeatedly to ensure the pin state is set correctly
 * over time.
 *
 * @param currentState The current state to determine if the pin should be
 * updated.
 */
void PinPulser::update(bool currentState) {
  if (!currentState) {
    // If the current state is false, set the pin to LOW and reset timing and
    // state variables
    pinState = LOW;
    digitalWrite(pin, LOW);
    lastChangeTime = millis();
    oneShotComplete = false;
    return;
  }

  if (oneShotComplete) {
    // If one-shot mode is complete, do nothing
    return;
  }

  unsigned long currentTime = millis(); // Get the current time

  // Update the pin state based on the current mode
  switch (mode) {
  case DISABLED:
    // If the mode is DISABLED, set the pin to LOW
    pinState = LOW;
    digitalWrite(pin, LOW);
    break;

  case PULSE:
    // If the mode is PULSE, toggle the pin state based on the high and low
    // times
    if (pinState == HIGH) {
      if (currentTime - lastChangeTime >= highTime) {
        pinState = LOW;
        digitalWrite(pin, LOW);
        lastChangeTime = currentTime;
      }
    } else {
      if (currentTime - lastChangeTime >= lowTime) {
        pinState = HIGH;
        digitalWrite(pin, HIGH);
        lastChangeTime = currentTime;
      }
    }
    break;

  case ONE_SHOT:
    // If the mode is ONE_SHOT, set the pin to HIGH for the high time and then
    // LOW
    if (pinState == HIGH) {
      if (currentTime - lastChangeTime >= highTime) {
        pinState = LOW;
        digitalWrite(pin, LOW);
        lastChangeTime = currentTime;
        oneShotComplete = true; // Mark one-shot as complete
      }
    } else {
      if (currentTime - lastChangeTime >= lowTime) {
        pinState = HIGH;
        digitalWrite(pin, HIGH);
        lastChangeTime = currentTime;
      }
    }
    break;

  case STEADY_STATE:
    // If the mode is STEADY_STATE, set the pin to HIGH
    pinState = HIGH;
    digitalWrite(pin, HIGH);
    break;
  }
}
