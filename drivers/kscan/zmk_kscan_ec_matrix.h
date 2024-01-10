#pragma once


struct zmk_kscan_ec_matrix_calibration_event {
    enum zmk_kscan_ec_matrix_calibration_event_type {
        CALIBRATION_EV_LOW_SAMPLING_START,
        CALIBRATION_EV_HIGH_SAMPLING_START,
        CALIBRATION_EV_POSITION_LOW_DETERMINED,
        CALIBRATION_EV_POSITION_COMPLETE,
        CALIBRATION_EV_COMPLETE,
    } type;

    union zmk_kscan_ec_matrix_calibration_event_data {
        struct {
            uint8_t strobe;
            uint8_t input;

            int16_t low_avg;
            int16_t noise;
            int16_t snr;
        } position_low_determined;
        struct {
            uint8_t strobe;
            uint8_t input;

            int16_t low_avg;
            int16_t high_avg;
            int16_t noise;
            int16_t snr;
        } position_complete;

        struct {

        } calibration_complete;
    } data;
};

struct zmk_kscan_ec_matrix_calibration_entry {
    uint16_t avg_low;
    uint16_t avg_high;
    uint16_t noise;
};

typedef void (*zmk_kscan_ec_matrix_calibration_cb_t)(const struct zmk_kscan_ec_matrix_calibration_event *ev, const void *);
typedef void (*zmk_kscan_ec_matrix_calibration_access_cb_t)(const struct device *dev, struct zmk_kscan_ec_matrix_calibration_entry *entries, size_t len, const void *user_data);

int zmk_kscan_ec_matrix_calibrate(const struct device *dev, zmk_kscan_ec_matrix_calibration_cb_t cb, const void *user_data);

int zmk_kscan_ec_matrix_access_calibration(const struct device *dev, zmk_kscan_ec_matrix_calibration_access_cb_t cb, const void *user_data);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

uint64_t zmk_kscan_ec_matrix_max_scan_duration_ns(const struct device *dev);

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_SCAN_RATE_CALC)

#if IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)

struct zmk_kscan_ec_matrix_read_timing {
    uint64_t total_ns;
    uint64_t adc_sequence_init_ns;
    uint64_t gpio_input_ns;
    uint64_t relax_ns;
    uint64_t plug_drain_ns;
    uint64_t set_strobe_ns;
    uint64_t read_settle_ns;
    uint64_t adc_read_ns;
    uint64_t unset_strobe_ns;
    uint64_t pull_drain_ns;
    uint64_t input_disconnect_ns;
};

struct zmk_kscan_ec_matrix_read_timing zmk_kscan_ec_matrix_read_timing(const struct device *dev);

#endif // IS_ENABLED(CONFIG_ZMK_KSCAN_EC_MATRIX_READ_TIMING)