#ifndef POKIBOT_INCLUDE_DRIVERS_SERVOMOTOR_H_
#define POKIBOT_INCLUDE_DRIVERS_SERVOMOTOR_H_

#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
* Angle in radians
*/
typedef int (*servomotor_api_set_angle)(const struct device *dev, float angle);

struct servomotor_driver_api {
    servomotor_api_set_angle set_angle;
};

static inline int servomotor_set_angle(const struct device *dev, float angle)
{
    const struct servomotor_driver_api *api = (const struct servomotor_driver_api *)dev->api;

    if (api->set_angle == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->set_angle(dev, angle);
}



#ifdef __cplusplus
}
#endif

#endif /* POKIBOT_INCLUDE_DRIVERS_SERVOMOTOR_H_ */
