/*
 * Copyright (c) 2022, 2023 Kan-Ru Chen
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

#include "zmk_kscan_ec_matrix.h"

#define LOG_LEVEL CONFIG_KSCAN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_kscan_ec_matrix);

struct kscan_ec_matrix_config
{
    const struct pinctrl_dev_config *pcfg;
    struct gpio_dt_spec power;
    struct gpio_dt_spec drain;
    const struct adc_dt_spec adc_channel;
    const uint8_t strobes_len;
    const uint8_t inputs_len;
    const uint16_t matrix_warm_up_ms;
    const uint16_t matrix_relax_us;
    const uint16_t adc_read_settle_us;
    const uint16_t active_polling_interval_ms;
    const uint16_t idle_polling_interval_ms;
    const uint16_t sleep_polling_interval_ms;
    const struct gpio_dt_spec *inputs;
    const uint32_t *strobe_input_masks;
    const struct gpio_dt_spec strobes[];
};

struct kscan_ec_matrix_data
{
    kscan_callback_t callback;
    uint16_t poll_interval;
    struct k_thread thread;
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ZMK_KSCAN_EC_MATRIX_THREAD_STACK_SIZE);
    const struct device *dev;
    struct k_mutex mutex;
#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)
    zmk_kscan_ec_matrix_calibration_cb_t calibration_callback;
    const void *calibration_user_data;
    bool calibrating;
#endif // IS_DEFINED(CONFIG_ZMK_KSCAN_EC_MATRIX_CALIBRATOR)
    struct zmk_kscan_ec_matrix_calibration_entry *calibrations;
    uint64_t matrix_state[];
};

static int kscan_ec_matrix_configure(const struct device *dev, kscan_callback_t callback)
{
    struct kscan_ec_matrix_data *data = dev->data;
    if (!callback)
    {
        return -EINVAL;
    }
    data->callback = callback;
    return 0;
}

static int kscan_ec_matrix_enable(const struct device *dev)
{
    struct kscan_ec_matrix_data *data = dev->data;
    k_thread_resume(&data->thread);
    return 0;
}

static int kscan_ec_matrix_disable(const struct device *dev)
{
    struct kscan_ec_matrix_data *data = dev->data;
    k_thread_suspend(&data->thread);
    return 0;
}

struct zmk_kscan_ec_matrix_calibration_entry *calibration_entry_for_strobe_input(const struct device *dev, uint8_t strobe, uint8_t input) {
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;

    return &data->calibrations[(strobe * cfg->inputs_len) + input];
}

static uint16_t read_raw_matrix_state(const struct device *dev, uint8_t strobe, uint8_t input) {
    const struct kscan_ec_matrix_config *cfg = dev->config;

    int16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    adc_sequence_init_dt(&cfg->adc_channel, &sequence);

    gpio_pin_configure_dt(&cfg->inputs[input], GPIO_INPUT);

    // TODO: Only wait as long as is need after drain pin was set low.
    k_busy_wait(cfg->matrix_relax_us);

    if (cfg->drain.port != NULL) {
        gpio_pin_set_dt(&cfg->drain, 1);
    }

    const uint32_t lock = irq_lock();

    gpio_pin_set_dt(&cfg->strobes[strobe], 1);
    k_busy_wait(cfg->adc_read_settle_us);
    int ret = adc_read(cfg->adc_channel.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("ADC READ ERROR %d", ret);
    }

    irq_unlock(lock);

    gpio_pin_set_dt(&cfg->strobes[strobe], 0);

    if (cfg->drain.port != NULL) {
        gpio_pin_set_dt(&cfg->drain, 0);
    }

    gpio_pin_configure_dt(&cfg->inputs[input], GPIO_DISCONNECTED);

    return buf;
}

#define SAMPLE_COUNT 10

struct sample_results {
    uint16_t min;
    uint16_t max;
    uint16_t avg;
    uint16_t noise;
};

struct sample_results sample(const struct device *dev, int s, int i) {
    uint16_t min = 0, max = 0, avg = 0;

    for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
        uint16_t val = read_raw_matrix_state(dev, s, i);
        LOG_DBG("Val: %d", val);

        if (sample == 0) {
            avg = min = max = val;
        } else {
            max = MAX(val, max);
            min = MIN(val, min);
            avg = ((avg * sample) + val) / (sample + 1);
        }

        k_sleep(K_MSEC(100));
    }

    return (struct sample_results) {
        .min = min,
        .max = max,
        .avg = avg,
        .noise = max - min,
    };
}

uint16_t normalize (uint16_t val, uint16_t avg_low, uint16_t avg_high) {
    val = MAX(val, avg_low);
    val = MIN(val, avg_high);

    uint32_t numerator = UINT16_MAX * (val - avg_low);
    uint16_t denominator = avg_high - avg_low;

    return (uint16_t) (numerator / denominator);
}

void calibrate(const struct device *dev) {
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;

    for (int s = 0; s < cfg->strobes_len; s++) {
        for (int i = 0; i < cfg->inputs_len; i++) {
            if (cfg->strobe_input_masks && (cfg->strobe_input_masks[s] & BIT(i)) != 0) {
                continue;
            }

            struct sample_results low_res = sample(dev, s, i);

            LOG_DBG("Low avg for %d,%d using %d and %d is %d. Noise %d", s, i, low_res.max, low_res.min, low_res.avg, low_res.noise);
            if (data->calibration_callback) {
                struct zmk_kscan_ec_matrix_calibration_event ev = { .type = CALIBRATION_EV_POSITION_LOW_DETERMINED, .data = { .position_low_determined = { .low_avg = low_res.avg, .strobe = s, .input = i, .noise = low_res.noise } } };
                data->calibration_callback(&ev, data->calibration_user_data);
            }            

            while(read_raw_matrix_state(dev, s, i) < (low_res.avg * 2)) {
                k_sleep(K_SECONDS(1));
            }

            k_sleep(K_MSEC(500));

            struct sample_results high_res = sample(dev, s, i);
        
            uint16_t snr = (high_res.max - low_res.max) / low_res.noise;
            LOG_DBG("High avg for %d,%d is %d. SNR %d", s, i, high_res.avg, snr);

            if (data->calibration_callback) {
                struct zmk_kscan_ec_matrix_calibration_event ev = { .type = CALIBRATION_EV_POSITION_COMPLETE, .data = { .position_complete = { .high_avg = high_res.avg, .snr = snr, .low_avg = low_res.avg, .strobe = s, .input = i, .noise = low_res.noise} } };
                data->calibration_callback(&ev, data->calibration_user_data);
            }

            struct zmk_kscan_ec_matrix_calibration_entry *calibration = calibration_entry_for_strobe_input(dev, s, i);
            calibration->avg_low = low_res.avg;
            calibration->avg_high = high_res.avg;
            calibration->noise = MAX(low_res.noise, high_res.noise);

            while(read_raw_matrix_state(dev, s, i) > (low_res.avg * 2)) {
                k_sleep(K_MSEC(500));
            }
        }
    }

    if (data->calibration_callback) {
        struct zmk_kscan_ec_matrix_calibration_event ev = { .type = CALIBRATION_EV_COMPLETE, };
        data->calibration_callback(&ev, data->calibration_user_data);
    }

    data->calibration_callback = NULL;
    data->calibration_user_data = NULL;
}

int zmk_kscan_ec_matrix_calibrate(const struct device *dev, zmk_kscan_ec_matrix_calibration_cb_t callback, const void *user_data) {
    struct kscan_ec_matrix_data *data = dev->data;

    int ret = k_mutex_lock(&data->mutex, K_SECONDS(1));

    if (ret < 0) {
        return -EAGAIN;
    }

    data->calibration_callback = callback;
    data->calibration_user_data = user_data;

    k_mutex_unlock(&data->mutex);

    return 0;
}

int zmk_kscan_ec_matrix_access_calibration(const struct device *dev, zmk_kscan_ec_matrix_calibration_access_cb_t cb, const void *user_data) {
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;

    int ret = k_mutex_lock(&data->mutex, K_SECONDS(1));

    if (ret < 0) {
        return -EAGAIN;
    }

    cb(dev, data->calibrations, cfg->inputs_len * cfg->strobes_len, user_data);

    k_mutex_unlock(&data->mutex);

    return 0;
}


static void kscan_ec_matrix_read(const struct device *dev)
{
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;

    uint64_t rows[cfg->strobes_len];

    for (int s = 0; s < cfg->strobes_len; s++) {
        rows[s] = 0;
    }

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 1);
        // The board needs some time to be operational after powering up
        k_sleep(K_MSEC(cfg->matrix_warm_up_ms));
    }

    for (int r = 0; r < cfg->inputs_len; r++) {
        for (int s = 0; s < cfg->strobes_len; s++) {

            if (cfg->strobe_input_masks && (cfg->strobe_input_masks[s] & BIT(r)) != 0) {
                continue;
            }

            bool prev = (data->matrix_state[s] & BIT(r)) != 0;
            uint16_t buf = read_raw_matrix_state(dev, s, r);

            struct zmk_kscan_ec_matrix_calibration_entry *calibration = calibration_entry_for_strobe_input(dev, s, r);

            uint16_t press_limit = 3800;
            uint16_t release_limit = 3500;

            if (calibration && calibration->avg_high != 0) {
                buf = normalize(buf, calibration->avg_low, calibration->avg_high);

                uint16_t range = calibration->avg_high - calibration->avg_low;
                uint16_t press_limit_raw = calibration->avg_high - (range / 5);
                uint16_t hys_buffer = calibration->noise * 3;
                press_limit = normalize(press_limit_raw, calibration->avg_low, calibration->avg_high);
                release_limit = normalize(press_limit_raw - hys_buffer, calibration->avg_low, calibration->avg_high);
            }

            if (buf > press_limit && !prev) {
                WRITE_BIT(rows[s], r, 1);
            } else if (prev && buf < release_limit) {
                WRITE_BIT(rows[s], r, 0);
            } else {
                WRITE_BIT(rows[s], r, prev);
            }
        }

        k_yield();
    }

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 0);
    }

    for (int s = 0; s < cfg->strobes_len; s++) {
        uint64_t row = rows[s];
        for (int r = 0; r < cfg->inputs_len; r++) {
            if ((data->matrix_state[s] & BIT(r)) != (row & BIT(r))) {
                if (data->callback) {
                    data->callback(data->dev, s, r, row & BIT(r));
                }
            }
        }

        data->matrix_state[s] = row;
    }
}

static void kscan_ec_matrix_thread_main(void *arg1, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	const struct device *dev = (const struct device *)arg1;
	struct kscan_ec_matrix_data *data = dev->data;

	while (1) {
        k_mutex_lock(&data->mutex, K_FOREVER);
        if (data->calibration_callback) {
            calibrate(dev);
        } else {
		    kscan_ec_matrix_read(dev);
        }
        k_mutex_unlock(&data->mutex);
        k_sleep(K_MSEC(data->poll_interval));
	}
}

static int kscan_ec_matrix_init(const struct device *dev)
{
    int err;
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    data->dev = dev;

    k_mutex_init(&data->mutex);

    if (!device_is_ready(cfg->adc_channel.dev)) {
        LOG_ERR("ADC Channel device is not ready");
        return -ENODEV;
    }

    err = adc_channel_setup_dt(&cfg->adc_channel);
    if (err < 0) {
        LOG_ERR("Failed to set up ADC channnel (%d)", err);
        return err;
    }

    err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

    if (cfg->power.port != NULL) {
        if (!device_is_ready(cfg->power.port)) {
            LOG_ERR("Power port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->power, GPIO_OUTPUT_INACTIVE);
    }

    if (cfg->drain.port != NULL) {
        if (!device_is_ready(cfg->drain.port)) {
            LOG_ERR("Drain port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->drain, GPIO_OUTPUT_INACTIVE);
    }

    for (int i = 0; i < cfg->strobes_len; i++) {
        if (!device_is_ready(cfg->strobes[i].port)) {
            LOG_ERR("Strobe port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->strobes[i], GPIO_OUTPUT_INACTIVE);
    }

    for (int i = 0; i < cfg->inputs_len; i++) {
        if (!device_is_ready(cfg->inputs[i].port)) {
            LOG_ERR("Input port is not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->inputs[i], GPIO_DISCONNECTED);
    }

    data->poll_interval = cfg->active_polling_interval_ms;

    k_thread_create(
		&data->thread,
		data->thread_stack,
		CONFIG_ZMK_KSCAN_EC_MATRIX_THREAD_STACK_SIZE,
		kscan_ec_matrix_thread_main,
		(void *)dev,
		NULL,
		NULL,
		K_PRIO_COOP(CONFIG_ZMK_KSCAN_EC_MATRIX_THREAD_PRIORITY),
		0,
		K_NO_WAIT);

    k_thread_suspend(&data->thread);

    return 0;
}
static const struct kscan_driver_api kscan_ec_matrix_api = {
    .config = kscan_ec_matrix_configure,
    .enable_callback = kscan_ec_matrix_enable,
    .disable_callback = kscan_ec_matrix_disable,
};

#define ZKEM_GPIO_DT_SPEC_ELEM(n, prop, idx) \
    GPIO_DT_SPEC_GET_BY_IDX(n, prop, idx),

#define ZERO(n, idx) 0

#define ENTRIES(n) DT_INST_PROP_LEN(n, strobe_gpios) * DT_INST_PROP_LEN(n, input_gpios)

#define ZKEM_INIT(n)                                                       \
    PINCTRL_DT_INST_DEFINE(n);						\
    static struct zmk_kscan_ec_matrix_calibration_entry calibration_entries_##n[ENTRIES(n)] = { 0 }; \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, strobe_input_masks), (static const uint32_t strobe_input_masks_##n[] = DT_INST_PROP(n, strobe_input_masks);), ( )) \
    static struct kscan_ec_matrix_data kscan_ec_matrix_data##n = { \
        .calibrations = calibration_entries_##n, \
        .matrix_state = { LISTIFY(DT_INST_PROP_LEN(n, strobe_gpios), ZERO, (,)) },\
    };                         \
    static const struct gpio_dt_spec inputs_##n[] = {DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), input_gpios, ZKEM_GPIO_DT_SPEC_ELEM)}; \
    static const struct kscan_ec_matrix_config kscan_ec_matrix_config##n = {            \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
        .adc_channel = ADC_DT_SPEC_INST_GET(n), \
        .power = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}), \
        .drain = GPIO_DT_SPEC_INST_GET_OR(n, drain_gpios, {0}), \
        .strobes = {DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), strobe_gpios, ZKEM_GPIO_DT_SPEC_ELEM)}, \
        .strobes_len = DT_INST_PROP_LEN(n, strobe_gpios), \
        .inputs = inputs_##n, \
        .inputs_len = DT_INST_PROP_LEN(n, input_gpios), \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, strobe_input_masks), (.strobe_input_masks = strobe_input_masks_##n,), ( )) \
        .matrix_warm_up_ms = DT_INST_PROP_OR(n, matrix_warm_up_ms, 0),                        \
        .matrix_relax_us = DT_INST_PROP_OR(n, matrix_relax_us, 0),                            \
        .adc_read_settle_us = DT_INST_PROP_OR(n, adc_read_settle_us, 0),                      \
        .active_polling_interval_ms = DT_INST_PROP_OR(n, active_polling_interval_ms, 1),      \
        .idle_polling_interval_ms = DT_INST_PROP_OR(n, idle_polling_interval_ms, 5),          \
        .sleep_polling_interval_ms = DT_INST_PROP_OR(n, sleep_polling_interval_ms, 500),        \
    };                                                                                     \
    DEVICE_DT_INST_DEFINE(n,                                                            \
                          kscan_ec_matrix_init,                                            \
                          NULL,                                                            \
                          &kscan_ec_matrix_data##n,                                     \
                          &kscan_ec_matrix_config##n,                                   \
                          APPLICATION,                                                     \
                          CONFIG_APPLICATION_INIT_PRIORITY,                                \
                          &kscan_ec_matrix_api);

DT_INST_FOREACH_STATUS_OKAY(ZKEM_INIT)
