/*
 * Copyright (c) 2018 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <ctype.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>

#include "zmk_kscan_ec_matrix.h"
#include "ec_matrix_settings.h"

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ec_matrix_shell);

#define CMD_HELP_CALIBRATE \
	"EC Calibration Utilities.\n"
#define CMD_HELP_CALIBRATION_START 			\
	"Calibrate the EC Martix.\n"

#define CMD_HELP_CALIBRATION_SAVE 			\
	"Save the EC Martix Calibration To Flash.\n"


#define CMD_HELP_CALIBRATION_LOAD 			\
	"Load the EC Martix Calibration From Flash.\n"

#define DEVICES(n) DEVICE_DT_INST_GET(n),

#define INIT_MACRO() DT_INST_FOREACH_STATUS_OKAY(DEVICES) NULL

#define EC_MATRIX_ENTRY(dev_) { .dev = dev_ }

/* This table size is = ADC devices count + 1 (NA). */
static struct matrix_hdl {
	const struct device *dev;
} matrix_hdl_list[] = {
	FOR_EACH(EC_MATRIX_ENTRY, (,), INIT_MACRO())
};

static struct matrix_hdl *get_matrix(const char *device_label)
{
	for (int i = 0; i < ARRAY_SIZE(matrix_hdl_list); i++) {
		if (!strcmp(device_label, matrix_hdl_list[i].dev->name)) {
			return &matrix_hdl_list[i];
		}
	}

	/* This will never happen because ADC was prompted by shell */
	__ASSERT_NO_MSG(false);
	return NULL;
}

static void calibrate_cb(const struct zmk_kscan_ec_matrix_calibration_event *ev, const void *user_data) {
    const struct shell *sh = (const struct shell*)user_data;

    switch (ev->type) {
        case CALIBRATION_EV_POSITION_LOW_DETERMINED:
            shell_print(sh, "Key at (%d,%d) is calibrated with avg low %d, noise: %d\nPress the key!", ev->data.position_low_determined.strobe, ev->data.position_low_determined.input, ev->data.position_low_determined.low_avg, ev->data.position_low_determined.noise);
            break;
        case CALIBRATION_EV_POSITION_COMPLETE:
            shell_print(sh, "Key at (%d,%d) is calibrated with avg low %d, avg high %d, noise: %d, SNR: %d\nRelease the key",
                ev->data.position_complete.strobe, ev->data.position_complete.input,
                ev->data.position_complete.low_avg, ev->data.position_complete.high_avg,
                ev->data.position_complete.noise, ev->data.position_complete.snr);
            break;
		case CALIBRATION_EV_COMPLETE:
			shell_print(sh, "Calibration complete, try it out!");
			break;
    }
}

static int cmd_matrix_calibration_start(const struct shell *shell, size_t argc, char **argv,
			void *data)
{
	/* -2: index of ADC label name */
	struct matrix_hdl *matrix = get_matrix(argv[-2]);

    int ret = zmk_kscan_ec_matrix_calibrate(matrix->dev, &calibrate_cb, shell);
	if (ret < 0) {
		shell_print(shell, "Failed to start calibration (%d)", ret);
	}

    return ret;
}

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS)


static int cmd_matrix_calibration_save(const struct shell *shell, size_t argc, char **argv,
			void *data)
{
	struct matrix_hdl *matrix = get_matrix(argv[-2]);
	int ret = zmk_kscan_ec_matrix_settings_save_calibration(matrix->dev);
	if (ret < 0) {
		shell_print(shell, "Failed to initiate save calibration (%d)", ret);
	}

    return ret;
}

static int cmd_matrix_calibration_load(const struct shell *shell, size_t argc, char **argv,
			void *data)
{
	struct matrix_hdl *matrix = get_matrix(argv[-2]);
	int ret = zmk_kscan_ec_matrix_settings_load_calibration(matrix->dev);
	if (ret < 0) {
		shell_print(shell, "Failed to initiate save calibration (%d)", ret);
	}

    return ret;
}

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SETTINGS)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_matrix_calibration_cmds,
	/* Alphabetically sorted. */
	SHELL_CMD(start, NULL, CMD_HELP_CALIBRATION_START, cmd_matrix_calibration_start),
#if IS_ENABLED(CONFIG_SETTINGS)
	SHELL_CMD(save, NULL, CMD_HELP_CALIBRATION_SAVE, cmd_matrix_calibration_save),
	SHELL_CMD(load, NULL, CMD_HELP_CALIBRATION_LOAD, cmd_matrix_calibration_load),
#endif // IS_ENABLED(CONFIG_SETTINGS)
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_matrix_cmds,
	/* Alphabetically sorted. */
	SHELL_CMD(calibration, &sub_matrix_calibration_cmds, CMD_HELP_CALIBRATE, NULL),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

static void cmd_matrix_dev_get(size_t idx, struct shell_static_entry *entry)
{
	/* -1 because the last element in the list is a "list terminator" */
	if (idx < ARRAY_SIZE(matrix_hdl_list) - 1) {
		entry->syntax  = matrix_hdl_list[idx].dev->name;
		entry->handler = NULL;
		entry->subcmd  = &sub_matrix_cmds;
		entry->help    = "Select subcommand for matrix property label.\n";
	} else {
		entry->syntax  = NULL;
	}
}
SHELL_DYNAMIC_CMD_CREATE(sub_ec_matrix_dev, cmd_matrix_dev_get);

SHELL_CMD_REGISTER(ec, &sub_ec_matrix_dev, "EC Matrix commands", NULL);
