
#include <sys/types.h>
#include <zephyr/settings/settings.h>

#include "ec_matrix_settings.h"
#include <stdio.h>
#include <string.h>
#include "zmk_kscan_ec_matrix.h"


#define LOG_LEVEL CONFIG_KSCAN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_kscan_ec_matrix_settings);

#define MAX_SETTING_LEN 32

struct load_state {
    char setting_name[MAX_SETTING_LEN];
    struct zmk_kscan_ec_matrix_calibration_entry *entries;
    size_t len;
};

static int settings_load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg, void *param) {
    struct load_state *state = (struct load_state *)param;
    
    ssize_t ret = read_cb(cb_arg, state->entries, state->len * sizeof(struct zmk_kscan_ec_matrix_calibration_entry));
    if (ret < 0) {
        LOG_ERR("Failed to load the settings from flash");
    }

    return ret;
}

static void load_cb(const struct device *dev, struct zmk_kscan_ec_matrix_calibration_entry *entries, size_t len, const void *user_data) {
    struct load_state state = (struct load_state) { .entries = entries, .len = len };
	snprintf(state.setting_name, MAX_SETTING_LEN, "zmk/ec/calibration/%s", dev->name);
    LOG_DBG("Loading the subtree directly for %s", state.setting_name);
    settings_load_subtree_direct(state.setting_name, settings_load_cb, &state);
}

static void save_cb(const struct device *dev, struct zmk_kscan_ec_matrix_calibration_entry *entries, size_t len, const void *user_data) {
	char setting_name[MAX_SETTING_LEN];
	snprintf(setting_name, MAX_SETTING_LEN, "zmk/ec/calibration/%s", dev->name);
    LOG_DBG("Saving the value for %s", setting_name);
	settings_save_one(setting_name, entries, len * sizeof(struct zmk_kscan_ec_matrix_calibration_entry));
}

int zmk_kscan_ec_matrix_settings_load_calibration(const struct device *dev) {
    int ret = zmk_kscan_ec_matrix_access_calibration(dev, &load_cb, NULL);
    return ret;
}

int zmk_kscan_ec_matrix_settings_save_calibration(const struct device *dev) {
    int ret = zmk_kscan_ec_matrix_access_calibration(dev, &save_cb, NULL);
    return ret;
}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS_INIT_LOAD)
#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#define LOAD_CALL(n) zmk_kscan_ec_matrix_settings_load_calibration(DEVICE_DT_GET(DT_DRV_INST(n)));

static int zmk_kscan_ec_matrix_settings_init(const struct device *dev) {
    DT_INST_FOREACH_STATUS_OKAY(LOAD_CALL)

	return 0;
}

SYS_INIT(zmk_kscan_ec_matrix_settings_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS_INIT_LOAD)
