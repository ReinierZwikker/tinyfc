//
//


#include "heli_mixer.h"

#include <string.h>

#include "main_conf.h"
#include "utils.h"


int16_t states[STATE_CHANNEL_COUNT];

void mixer_init() {
  reset_mixer();
}

void disarm_mixer() {
  reset_mixer();
}

void reset_mixer() {
  memset(states, 0, STATE_CHANNEL_COUNT);
}


void update_mixer(const int16_t *inputs, uint16_t *outputs,
                  const enum flight_mode_t flight_mode) {

  #define SIN_60(x)  ((x) - (x)/8 - (x)/128 - (x)/512)   //  1000*sin(60) = 866.02... ~= 865

  int16_t new_states[STATE_CHANNEL_COUNT] = {0};

  const int16_t lat_cyc_sin60 = SIN_60(inputs[INPUT_CHANNEL_LAT_CYC]);
  const int16_t lon_cyc_half  = (int16_t) (inputs[INPUT_CHANNEL_LON_CYC] / 2);

  // SWASH PLATE
  new_states[ACTUATOR_SWASH_LEFT] = (int16_t)
          (  inputs[INPUT_CHANNEL_COLLECTIVE]
           + lon_cyc_half
           + lat_cyc_sin60);

  new_states[ACTUATOR_SWASH_RIGHT] = (int16_t) (-1 *
          (  inputs[INPUT_CHANNEL_COLLECTIVE]
           + lon_cyc_half
           - lat_cyc_sin60));

  new_states[ACTUATOR_SWASH_AFT] = (int16_t)
          (  inputs[INPUT_CHANNEL_COLLECTIVE]
           - inputs[INPUT_CHANNEL_LON_CYC]);

  new_states[ACTUATOR_MAIN_ROTOR] = inputs[INPUT_CHANNEL_THROTTLE];

  new_states[ACTUATOR_TAIL_ROTOR] = (int16_t)
          (  inputs[INPUT_CHANNEL_THROTTLE] * (1 / ROTOR_MAIN_TO_PEDAL_INV_GAIN)
           + inputs[INPUT_CHANNEL_COLLECTIVE] * (1 / COLL_TO_PEDAL_INV_GAIN)
           - inputs[INPUT_CHANNEL_PEDALS]
           + ROTOR_PEDAL_TRIM);

  // LIMITS
  for (uint8_t i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
    if (new_states[i] > 1000) { new_states[i] = 1000; }
    if (new_states[i] < -1000) { new_states[i] = -1000; }
  }

  // SWASHPLATE SERVOS

  // With low-pass
  // output = ((G-1 * old) + 1 * new) / G  <-- Low pass filter
  // current_actuator_channels[ACTUATOR_SWASH_LEFT] = (int16_t)
  //         ((  (ACTUATOR_SWASH_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_SWASH_LEFT]
  //           +                            1  *     mixer_output_channels[ACTUATOR_SWASH_LEFT])
  //                                                                                 / ACTUATOR_SWASH_LP_PARAM);
  // current_actuator_channels[ACTUATOR_SWASH_RIGHT] = (int16_t)
  //         ((  (ACTUATOR_SWASH_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_SWASH_RIGHT]
  //           +                            1  *     mixer_output_channels[ACTUATOR_SWASH_RIGHT])
  //                                                                                 / ACTUATOR_SWASH_LP_PARAM);
  // current_actuator_channels[ACTUATOR_SWASH_AFT] = (int16_t)
  //         ((  (ACTUATOR_SWASH_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_SWASH_AFT]
  //           +                            1  *     mixer_output_channels[ACTUATOR_SWASH_AFT])
  //                                                                                 / ACTUATOR_SWASH_LP_PARAM);

  // Direct
  states[STATE_SWASH_LEFT] = new_states[STATE_SWASH_LEFT];
  states[STATE_SWASH_RIGHT] = new_states[STATE_SWASH_RIGHT];
  states[STATE_SWASH_AFT] = new_states[STATE_SWASH_AFT];

  // MAIN ROTOR THROTTLE

  // ROTOR THROTTLE
  if (new_states[STATE_MAIN_ROTOR] > states[STATE_MAIN_ROTOR] + ACTUATOR_MAIN_MAX_DELTA) {
    new_states[STATE_MAIN_ROTOR] = (int16_t) (states[STATE_MAIN_ROTOR] + ACTUATOR_MAIN_MAX_DELTA);
  }
  if (new_states[STATE_MAIN_ROTOR] < states[STATE_MAIN_ROTOR] - ACTUATOR_MAIN_MAX_DELTA) {
    new_states[STATE_MAIN_ROTOR] = (int16_t) (states[STATE_MAIN_ROTOR] - ACTUATOR_MAIN_MAX_DELTA);
  }

  states[ACTUATOR_MAIN_ROTOR] = (int16_t)
          ((  (ACTUATOR_MAIN_LP_PARAM - 1) *     states[ACTUATOR_MAIN_ROTOR]
            +                           1  * new_states[ACTUATOR_MAIN_ROTOR]) / ACTUATOR_MAIN_LP_PARAM);



  // TAIL ROTOR THROTTLE
  // current_actuator_channels[ACTUATOR_TAIL_ROTOR] = (int16_t)
  //         ((  (ACTUATOR_TAIL_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_TAIL_ROTOR]
  //           +                           1  *     mixer_output_channels[ACTUATOR_TAIL_ROTOR]) / ACTUATOR_TAIL_LP_PARAM);
  states[STATE_TAIL_ROTOR] = new_states[STATE_TAIL_ROTOR];

  // OUTPUTS
  switch (flight_mode) {
    case FLIGHT_MODE_POSITION:
    case FLIGHT_MODE_ANGLE:
    case FLIGHT_MODE_RATE:
      // NOT YET IMPLEMENTED
    default:
    case FLIGHT_MODE_DIRECT:
      outputs[ACTUATOR_SWASH_LEFT]  = denormalize_pwm(states[STATE_SWASH_LEFT]);
      outputs[ACTUATOR_SWASH_RIGHT] = denormalize_pwm(states[STATE_SWASH_RIGHT]);
      outputs[ACTUATOR_SWASH_AFT]   = denormalize_pwm(states[STATE_SWASH_AFT]);
      outputs[ACTUATOR_MAIN_ROTOR]  = denormalize_os1(states[STATE_MAIN_ROTOR]);
      outputs[ACTUATOR_TAIL_ROTOR]  = denormalize_os1(states[STATE_TAIL_ROTOR]);
      break;
  }


}
