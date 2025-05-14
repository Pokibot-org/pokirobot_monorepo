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
typedef int (*pokstepper_api_set_speed)(const struct device *dev, int32_t speed_sps);
typedef int (*pokstepper_api_set_pos)(const struct device *dev, int32_t pos);
typedef int (*pokstepper_api_get_pos)(const struct device *dev, int32_t *pos);
typedef int (*pokstepper_api_move_by)(const struct device *dev, uint32_t speed_sps, int32_t steps);
typedef int (*pokstepper_api_move_to)(const struct device *dev, uint32_t speed_sps, int32_t steps);
typedef int (*pokstepper_api_go_to_stall)(const struct device *dev, uint32_t speed_sps,
                                          int32_t steps, uint8_t detection_threshold);

struct pokstepper_driver_api {
    pokstepper_api_enable enable;
    pokstepper_api_set_speed set_speed;
    pokstepper_api_set_pos set_pos;
    pokstepper_api_get_pos get_pos;
    pokstepper_api_move_by move_by;
    pokstepper_api_move_to move_to;
    pokstepper_api_go_to_stall go_to_stall;
};

static inline int pokstepper_enable(const struct device *dev, bool value)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->enable == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->enable(dev, value);
}

static inline int pokstepper_set_speed(const struct device *dev, int32_t speed_sps)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->set_speed == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->set_speed(dev, speed_sps);
}

static inline int pokstepper_set_pos(const struct device *dev, int32_t pos)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->set_pos == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->set_pos(dev, pos);
}

static inline int pokstepper_get_pos(const struct device *dev, int32_t *pos)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->get_pos == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->get_pos(dev, pos);
}

static inline int pokstepper_move_by(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->move_by == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->move_by(dev, speed_sps, steps);
}

static inline int pokstepper_move_to(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->move_to == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->move_to(dev, speed_sps, steps);
}

static inline int pokstepper_go_to_stall(const struct device *dev, uint32_t speed_sps,
                                         int32_t steps, uint8_t detection_threshold)
{
    const struct pokstepper_driver_api *api = (const struct pokstepper_driver_api *)dev->api;

    if (api->go_to_stall == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->go_to_stall(dev, speed_sps, steps, detection_threshold);
}

#ifdef __cplusplus
}
#endif

#endif /* POKIBOT_INCLUDE_DRIVERS_POKSTEPPER_H_ */
