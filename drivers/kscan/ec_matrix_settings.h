
#include <zephyr/device.h>

#include "zmk_kscan_ec_matrix.h"

int zmk_kscan_ec_matrix_settings_load_calibration(const struct device *dev);
int zmk_kscan_ec_matrix_settings_save_calibration(const struct device *dev);