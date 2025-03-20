#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "pokdefs.h"

LOG_MODULE_REGISTER(const_pub);


int pub_const_init(void) {
    return 0;
}

SYS_INIT(pub_const_init, APPLICATION, 0);
