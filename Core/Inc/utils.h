//
//


#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>



uint16_t convert_crsf_to_pwm(uint16_t crsf_value);
uint16_t convert_pwm_to_crsf(uint16_t pwm_value);
int16_t normalize_crsf(uint16_t crsf_value);
int16_t normalize_pwm(uint16_t pwm_value);
uint16_t denormalize_crsf(int16_t norm_value);
uint16_t denormalize_pwm(int16_t norm_value);

#endif // __UTILS_H
