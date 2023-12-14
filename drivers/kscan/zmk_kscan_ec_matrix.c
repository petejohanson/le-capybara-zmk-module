/*
 * Copyright (c) 2022, 2023 Kan-Ru Chen
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_ec_matrix

#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

#define LOG_LEVEL CONFIG_KSCAN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_kscan_ec_matrix);

struct kscan_ec_matrix_config
{
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
    const struct gpio_dt_spec strobes[];
};

struct kscan_ec_matrix_data
{
    kscan_callback_t callback;
    uint16_t poll_interval;
    struct k_thread thread;
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ZMK_KSCAN_EC_MATRIX_THREAD_STACK_SIZE);
    const struct device *dev;
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

static void kscan_ec_matrix_read(const struct device *dev)
{
    const struct kscan_ec_matrix_config *cfg = dev->config;
    struct kscan_ec_matrix_data *data = dev->data;

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 1);
        // The board needs some time to be operational after powering up
        k_sleep(K_MSEC(cfg->matrix_warm_up_ms));
    }

    int16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    adc_sequence_init_dt(&cfg->adc_channel, &sequence);

    for (int r = 0; r < cfg->inputs_len; r++) {
        gpio_pin_configure_dt(&cfg->inputs[r], GPIO_INPUT);

        k_busy_wait(cfg->matrix_relax_us);
        for (int s = 0; s < cfg->strobes_len; s++)  {

            bool prev = (data->matrix_state[s] & BIT(r)) != 0;

            k_busy_wait(cfg->matrix_relax_us);

            if (cfg->drain.port != NULL) {
                gpio_pin_set_dt(&cfg->drain, 1);
            }

            const uint32_t lock = irq_lock();

            gpio_pin_set_dt(&cfg->strobes[s], 1);
            // k_busy_wait(10);
            int ret = adc_read(cfg->adc_channel.dev, &sequence);
            if (ret < 0) {
                LOG_ERR("ADC READ ERROR %d", ret);
            }

            irq_unlock(lock);

            // TODO:
            // 1. Normalize
            // 2. Compare against press/release limits
            // 3. Add to ire list if changed.
            if (buf > 100) {
                LOG_DBG("Buffer value: %d for %d, %d", buf, r, s);
            }

            gpio_pin_set_dt(&cfg->strobes[s], 0);

            if (cfg->drain.port != NULL) {
                gpio_pin_set_dt(&cfg->drain, 0);
            }
        }

        gpio_pin_configure_dt(&cfg->inputs[r], GPIO_DISCONNECTED);
    }

    if (cfg->power.port) {
        gpio_pin_set_dt(&cfg->power, 0);
    }

    // for (int r = 0; r < MATRIX_ROWS; ++r)
    // {
    //     for (int c = 0; c < MATRIX_COLS; ++c)
    //     {
    //         int cell = (r * MATRIX_COLS) + c;
    //         if (data->matrix_state[cell] != matrix_read[cell])
    //         {
    //             data->matrix_state[cell] = matrix_read[cell];
    //             data->callback(data->dev, r, c, matrix_read[cell]);
    //         }
    //     }
    // }
}

static void kscan_ec_matrix_thread_main(void *arg1, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	const struct device *dev = (const struct device *)arg1;
	struct kscan_ec_matrix_data *data = dev->data;

	while (1) {
		kscan_ec_matrix_read(dev);
        k_sleep(K_MSEC(data->poll_interval));
	}
}

static int kscan_ec_matrix_init(const struct device *dev)
{
    int err;
    struct kscan_ec_matrix_data *data = dev->data;
    const struct kscan_ec_matrix_config *cfg = dev->config;
    data->dev = dev;

    if (!device_is_ready(cfg->adc_channel.dev)) {
        LOG_ERR("ADC Channel device is not ready");
        return -ENODEV;
    }

    err = adc_channel_setup_dt(&cfg->adc_channel);
    if (err < 0) {
        LOG_ERR("Failed to set up ADC channnel (%d)", err);
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

#define ZKEM_INIT(n)                                                       \
    static struct kscan_ec_matrix_data kscan_ec_matrix_data##n = { \
        .matrix_state = { LISTIFY(DT_INST_PROP_LEN(n, strobe_gpios), ZERO, (,)) },\
    };                         \
    static const struct gpio_dt_spec inputs_##n[] = {DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), input_gpios, ZKEM_GPIO_DT_SPEC_ELEM)}; \
    static const struct kscan_ec_matrix_config kscan_ec_matrix_config##n = {            \
        .adc_channel = ADC_DT_SPEC_INST_GET(n), \
        .power = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}), \
        .drain = GPIO_DT_SPEC_INST_GET_OR(n, drain_gpios, {0}), \
        .strobes = {DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), strobe_gpios, ZKEM_GPIO_DT_SPEC_ELEM)}, \
        .strobes_len = DT_INST_PROP_LEN(n, strobe_gpios), \
        .inputs = inputs_##n, \
        .inputs_len = DT_INST_PROP_LEN(n, input_gpios), \
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
