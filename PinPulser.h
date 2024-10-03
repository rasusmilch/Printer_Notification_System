/**
 * @file PinPulser.h
 * @brief Defines the PinPulser class for controlling a pin with pulsing
 * behavior.
 *
 * This file contains the definition of the PinPulser class, which can be used
 * to control a pin on an Arduino board with various pulsing modes.
 */

#ifndef PINPULSER_H
#define PINPULSER_H

#include <Arduino.h>

/**
 * @brief The PinPulser class for controlling a pin with pulsing behavior.
 *
 * This class allows you to control a pin on an Arduino board with various modes
 * such as DISABLED, PULSE, ONE_SHOT, and STEADY_STATE. The pin can be
 * configured with specific high and low times for pulsing behavior.
 */
class PinPulser {
public:
  /**
   * @brief Enumeration of the different modes for the PinPulser.
   */
  enum Mode { DISABLED, PULSE, ONE_SHOT, STEADY_STATE };

  /**
   * @brief Constructs a PinPulser object to control a pin with pulsing
   * behavior.
   *
   * @param pin The pin number to be controlled by the PinPulser.
   */
  PinPulser(int pin);

  /**
   * @brief Configures the timing and mode for the PinPulser.
   *
   * @param highTime The duration (in milliseconds) for which the pin will stay
   * HIGH.
   * @param lowTime The duration (in milliseconds) for which the pin will stay
   * LOW.
   * @param mode The operating mode of the PinPulser (e.g., DISABLED, PULSE,
   * ONE_SHOT, STEADY_STATE).
   */
  void configure(unsigned long highTime, unsigned long lowTime, Mode mode);

  /**
   * @brief Updates the state of the pin based on the current mode and timing
   * parameters.
   *
   * @param currentState The current state to determine if the pin should be
   * updated.
   */
  void update(bool currentState);

private:
  int pin;       /**< The pin number controlled by the PinPulser */
  bool pinState; /**< The current state of the pin (HIGH or LOW) */
  unsigned long lastChangeTime; /**< The last time the pin state was changed */
  unsigned long highTime; /**< The duration (in milliseconds) for which the pin
                             will stay HIGH */
  unsigned long lowTime;  /**< The duration (in milliseconds) for which the pin
                             will stay LOW */
  Mode mode;              /**< The operating mode of the PinPulser */
  bool oneShotComplete; /**< Flag indicating if the one-shot mode is complete */
  bool firstHigh = false;
};

#endif // PINPULSER_H
