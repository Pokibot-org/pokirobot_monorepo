#ifndef ZEPHYR_INCLUDE_DRIVERS_LIDAR_H_
#define ZEPHYR_INCLUDE_DRIVERS_LIDAR_H_


#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*lidar_api_start)(const struct device *dev);

/**
 * @brief LIDAR driver API
 */
struct lidar_driver_api {
	lidar_api_start start;
};

static inline int led_start(const struct device *dev)
{
	const struct lidar_driver_api *api =
		(const struct lidar_driver_api *)dev->api;

	if (api->start == NULL) {
		return -ENOSYS;
	}
	return api->start(dev);
}

#ifdef __cplusplus
}
#endif

#endif	/* ZEPHYR_INCLUDE_DRIVERS_LIDAR_H_ */
