//
//


#ifndef __MIXER_HELI_H
#define __MIXER_HELI_H

#include <stdint.h>
#include "tinyfc_types.h"

void mixer_init();

void update_mixer(const int16_t *inputs, uint16_t *outputs,
                  enum flight_mode_t flight_mode);

void disarm_mixer();
void reset_mixer();

#endif
