
#include <sys/types.h>
#include <zephyr/settings/settings.h>

#include "ec_matrix_settings.h"
#include <stdio.h>
#include <string.h>
#include "zmk_kscan_ec_matrix.h"

struct load_state {
    char setting_name[32];
    struct zmk_kscan_ec_matrix_calibration_entry *entries;
    size_t len;
};

static int settings_load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg, void *param) {
    struct load_state *state = (struct load_state *)param;

    if (strcmp(key, state->setting_name) != 0) {
        return 0;
    }
    
    ssize_t ret = read_cb(cb_arg, state->entries, state->len * sizeof(struct zmk_kscan_ec_matrix_calibration_entry));
    if (ret < 0) {
        // OH NO!
    }

    return ret;
}

static void load_cb(const struct device *dev, struct zmk_kscan_ec_matrix_calibration_entry *entries, size_t len, const void *user_data) {
    struct load_state state = (struct load_state) { .entries = entries, .len = len };
	sprintf(state.setting_name, "zmk/ec/calibration/%s", dev->name);
    settings_load_subtree_direct(state.setting_name, settings_load_cb, &state);
}

static void save_cb(const struct device *dev, struct zmk_kscan_ec_matrix_calibration_entry *entries, size_t len, const void *user_data) {
	// const struct shell *sh = (const struct shell*)user_data;

	// shell_print(sh, "SAVE %d entries", len);

	char setting_name[32];
	sprintf(setting_name, "zmk/ec/calibration/%s", dev->name);
	settings_save_one(setting_name, entries, len * sizeof(struct zmk_kscan_ec_matrix_calibration_entry));
}

int zmk_kscan_ec_matrix_settings_load_calibration(const struct device *dev) {
    int ret = zmk_kscan_ec_matrix_access_calibration(dev, &load_cb, NULL);
    return 0;
}

int zmk_kscan_ec_matrix_settings_save_calibration(const struct device *dev) {
    int ret = zmk_kscan_ec_matrix_access_calibration(dev, &save_cb, NULL);
    return ret;
}