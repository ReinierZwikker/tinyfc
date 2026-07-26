//
//


#include "utils.h"
#include "main_conf.h"

#include <stdint.h>
// #include <math.h>


uint16_t convert_crsf_to_pwm(uint16_t crsf_value) {
  return (uint16_t) (PWM_MIN + ((uint32_t)(crsf_value - CRSF_MIN) * (PWM_MAX - PWM_MIN)) / (CRSF_MAX - CRSF_MIN));
}

uint16_t convert_pwm_to_crsf(uint16_t pwm_value) {
  return (uint16_t) (CRSF_MIN + ((uint32_t)(pwm_value - PWM_MIN) * (CRSF_MAX - CRSF_MIN)) / (PWM_MAX - PWM_MIN));
}

int16_t normalize_crsf(uint16_t crsf_value) {
  return (int16_t) ((((int16_t) crsf_value - CRSF_MID) * NORM_RANGE) / (CRSF_MID - CRSF_MIN));
}

int16_t normalize_pwm(uint16_t pwm_value) {
  return (int16_t) ((((int16_t) pwm_value - PWM_MID) * NORM_RANGE) / (PWM_MID - PWM_MIN));
}

uint16_t denormalize_crsf(int16_t norm_value) {
  return (uint16_t) (CRSF_MID + (norm_value * (CRSF_MID - CRSF_MIN)) / NORM_RANGE);
}

uint16_t denormalize_pwm(int16_t norm_value) {
  return (uint16_t) (PWM_MID + (norm_value * (PWM_MID - PWM_MIN)) / NORM_RANGE);
}

uint16_t denormalize_os1(int16_t norm_value) {
  return (uint16_t) (ONESHOT125_MID + (norm_value * (ONESHOT125_MID - ONESHOT125_MIN)) / NORM_RANGE);
}
