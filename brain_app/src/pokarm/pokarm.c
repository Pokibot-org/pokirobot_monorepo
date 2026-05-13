#include <zephyr/kernel.h>
#include <pokibot/drivers/pokstepper.h>
#include <pokibot/drivers/servomotor.h>
#include <pokibot/lib/pokutils.h>
#include "pokarm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pokarm);

#define MM_TO_STEP(x) ((200 * (x)) / 30)


void banner_deploy(bool state) {

}

void pokarm_reset_pos(void) {
}

void pokarm_deploy(bool state)
{
}

void pokarm_pinch(bool state)
{
}

void pokarm_lift(bool state)
{
}
