#ifndef ZEPHYR_INCLUDE_DRIVERS_LIDAR_H_
#define ZEPHYR_INCLUDE_DRIVERS_LIDAR_H_

#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LiDAR point data structure.
 */
struct lidar_point {
    float angle;       // Angle of the measurement (in radians, counter clockwise)
    float distance;    // Distance of the measurement (in meters)
    uint8_t intensity; // Intensity or quality of the measurement (0-255)
};

struct lidar_callback {
    void (*clbk)(const struct lidar_point *points, size_t nb_points, void *user_data);
	void *user_data;

	/** Internally used field for list handling */
	sys_snode_t _node;
};

/**
 * @brief Function type for starting the LiDAR sensor.
 */
typedef int (*lidar_api_start)(const struct device *dev);

/**
 * @brief Function type for stopping the LiDAR sensor.
 */
typedef int (*lidar_api_stop)(const struct device *dev);

/**
 * @brief Function type for registering the LiDAR scan callback.
 */
typedef int (*lidar_api_register_callback)(const struct device *dev, struct lidar_callback *clbk);

/**
 * @brief LiDAR driver API structure.
 */
struct lidar_driver_api {
    lidar_api_start start;
    lidar_api_stop stop;
    lidar_api_register_callback register_callback;
};

/**
 * @brief Start the LiDAR sensor.
 *
 * @param dev Pointer to the LiDAR device structure.
 * @return 0 on success, negative error code on failure.
 */
static inline int lidar_start(const struct device *dev)
{
    const struct lidar_driver_api *api = (const struct lidar_driver_api *)dev->api;

    if (api->start == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->start(dev);
}

/**
 * @brief Stop the LiDAR sensor.
 *
 * @param dev Pointer to the LiDAR device structure.
 * @return 0 on success, negative error code on failure.
 */
static inline int lidar_stop(const struct device *dev)
{
    const struct lidar_driver_api *api = (const struct lidar_driver_api *)dev->api;

    if (api->stop == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->stop(dev);
}

/**
 * @brief Register a LiDAR scan callback.
 *
 * @param dev Pointer to the LiDAR device structure.
 * @param scan Pointer to the scan data structure to populate.
 * @return 0 on success, negative error code on failure.
 */
static inline int lidar_register_callback(const struct device *dev, struct lidar_callback *clbk)
{
    const struct lidar_driver_api *api = (const struct lidar_driver_api *)dev->api;

    if (api->register_callback == NULL) {
        return -ENOSYS; // Function not implemented
    }
    return api->register_callback(dev, clbk);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_LIDAR_H_ */
