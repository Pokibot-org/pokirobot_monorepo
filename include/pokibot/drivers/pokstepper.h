#ifndef POKIBOT_INCLUDE_DRIVERS_POKSTEPPER_H_
#define POKIBOT_INCLUDE_DRIVERS_POKSTEPPER_H_

#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef int (*pokstepper_api_enable)(const struct device *dev, bool value);
typedef int (*pokstepper_api_set_speed)(const struct device *dev, int32_t speed);

struct pokstepper_driver_api {
    pokstepper_api_enable enable;
    pokstepper_api_set_speed set_speed;
};

static inline int pokstepper_enable(const struct device *dev, bool value)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->enable == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->enable(dev, value);
}

static inline int pokstepper_set_speed(const struct device *dev, int32_t speed)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->set_speed == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->set_speed(dev, speed);
}



#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_LIDAR_H_ */
