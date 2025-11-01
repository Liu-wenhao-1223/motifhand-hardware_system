/*
 * mag_calibration.h
 *
 *  Created on: Apr 22, 2025
 *      Author: ASTRAYRMORS
 */

#ifndef BMM350_MAG_CALIBRATION_H_
#define BMM350_MAG_CALIBRATION_H_


void matmul(float* A, float* B, float* C, int m, int n, int p);
float round_x(float original, int power);
void magnet_norm(struct bmm350_mag_temp_data *mag_calibrated_data);
const float *megnet_soft_iron_matrix_match(const uint8_t id);
const float *megnet_hard_iron_bias_match(const uint8_t id);
void magnet_rescale(struct bmm350_mag_temp_data *mag_temp_data, struct bmm350_mag_temp_data *mag_rescaled_data);
void magnet_calibration(struct bmm350_mag_temp_data *mag_temp_data, struct bmm350_mag_temp_data *mag_calibrated_data);

typedef struct {
    const uint8_t *id;
    const float *matrix;
} IronEntry;

extern const float soft_iron_matrix_1[9];
extern const float soft_iron_matrix_2[9];
extern const float soft_iron_matrix_100[9];


extern const float hard_iron_bias_1[3];

extern const float hard_iron_bias_2[3];

extern const float hard_iron_bias_3[3];

extern const float hard_iron_bias_100[3];



#endif /* BMM350_MAG_CALIBRATION_H_ */
