#ifndef ZEPHYR_INCLUDE_POKMAC_H_
#define ZEPHYR_INCLUDE_POKMAC_H_


#include <errno.h>
#include <stddef.h>

#include <zephyr/sys/slist.h>
#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pokmac_recv_callback {
	void (*cb)(uint8_t *payload_data, size_t payload_size);
	/** Node for callback list */
	sys_snode_t node;
};


typedef int (*pokmac_api_register_recv_callback)(const struct device *dev, struct pokmac_recv_callback *clbk);
typedef int (*pokmac_api_send)(const struct device *dev, uint8_t *data, size_t size, bool confirmed);

/**
 * @brief LIDAR driver API
 */
struct pokmac_driver_api {
    pokmac_api_register_recv_callback register_recv_clbk;
	pokmac_api_send send;
};

static inline int pokmac_register_recv_callback(const struct device *dev, struct pokmac_recv_callback *clbk)
{
	const struct pokmac_driver_api *api =
		(const struct pokmac_driver_api *)dev->api;

	if (api->register_recv_clbk == NULL) {
		return -ENOSYS;
	}
	return api->register_recv_clbk(dev, clbk);
}

static inline int pokmac_send(const struct device *dev, uint8_t *payload_data, size_t payload_size, bool confirmed)
{
	const struct pokmac_driver_api *api =
		(const struct pokmac_driver_api *)dev->api;

	if (api->send == NULL) {
		return -ENOSYS;
	}
	return api->send(dev, payload_data, payload_size, confirmed);
}

#ifdef __cplusplus
}
#endif

#endif	/* ZEPHYR_INCLUDE_POKMAC_H_ */
