#pragma once


struct zmk_kscan_ec_matrix_calibration_event {
    enum zmk_kscan_ec_matrix_calibration_event_type {
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

typedef void (*zmk_kscan_ec_matrix_calibration_cb_t)(const struct zmk_kscan_ec_matrix_calibration_event *ev, const void *);

int zmk_kscan_ec_matrix_calibrate(const struct device *dev, zmk_kscan_ec_matrix_calibration_cb_t cb, const void *user_data);