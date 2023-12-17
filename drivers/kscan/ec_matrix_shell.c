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

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ec_matrix_shell);

#define CMD_HELP_CALIBRATE 			\
	"Calibrate the EC Martix.\n"

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

static int cmd_matrix_calibrate(const struct shell *shell, size_t argc, char **argv,
			void *data)
{
	/* -2: index of ADC label name */
	struct matrix_hdl *matrix = get_matrix(argv[-1]);

    int ret = zmk_kscan_ec_matrix_calibrate(matrix->dev, &calibrate_cb, shell);
	if (ret < 0) {
		shell_print(shell, "Failed to start calibration (%d)", ret);
	}

    return ret;
}


// static int cmd_adc_print(const struct shell *shell, size_t argc, char **argv)
// {
// 	/* -1 index of ADC label name */
// 	struct adc_hdl *adc = get_adc(argv[-1]);

// 	shell_print(shell, "%s:\n"
// 			   "Gain: %s\n"
// 			   "Reference: %s\n"
// 			   "Acquisition Time: %u\n"
// 			   "Channel ID: %u\n"
// 			   "Resolution: %u",
// 			   adc->dev->name,
// 			   chosen_gain,
// 			   chosen_reference,
// 			   adc->channel_config.acquisition_time,
// 			   adc->channel_config.channel_id,
// 			   adc->resolution);
// 	return 0;
// }


SHELL_STATIC_SUBCMD_SET_CREATE(sub_matrix_cmds,
	/* Alphabetically sorted. */
	SHELL_CMD_ARG(calibrate, NULL, CMD_HELP_CALIBRATE, cmd_matrix_calibrate, 1, 0),
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

SHELL_CMD_REGISTER(ec_matrix, &sub_ec_matrix_dev, "EC Matrix commands", NULL);
