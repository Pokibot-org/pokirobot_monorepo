#ifndef POKARM_H
#define POKARM_H
#include "stdbool.h"

void pokarm_reset_pos(void);
void pokarm_deploy(bool state);
void pokarm_pinch(bool state);
void pokarm_lift(bool state);

#endif
