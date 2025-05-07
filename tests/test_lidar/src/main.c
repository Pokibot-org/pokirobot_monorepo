#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/lidar.h>

LOG_MODULE_REGISTER(main);

const struct device *lidar_dev = DEVICE_DT_GET(DT_NODELABEL(lidar));

void lidar_callback(const struct lidar_point *points, size_t nb_points, void *user_data)
{
    for (size_t i = 0; i < nb_points; i++) {
        const struct lidar_point *point = &points[i];
        LOG_RAW("LD>%0.3f;%0.3f,%d", (double)point->angle, (double)point->distance, point->intensity);
    }
}

int main(void)
{
    static struct lidar_callback lidar_clbk = {
        .clbk = lidar_callback,
    };
    lidar_register_callback(lidar_dev, &lidar_clbk);
    lidar_start(lidar_dev);

    return 0;
}
