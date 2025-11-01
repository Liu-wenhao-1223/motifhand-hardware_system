/*
 * mag_calibration.c
 *
 *  Created on: Apr 22, 2025
 *      Author: ASTRAYRMORS
 */
#include <math.h>
#include "main.h"
#include <string.h>
#include "bmm350.h"
#include "mag_calibration.h"

//extern struct bmm350_mag_temp_data mag_calibrated_data;
extern char board_id[8];
extern uint8_t board_id_slave;
//extern const float soft_iron_matrix_1[9];
//extern const float soft_iron_matrix_2[9];
//extern const float soft_iron_matrix_100[9];

const float soft_iron_matrix_100[9] = {
		1.46592931e-02, 2.46769937e-04, -1.07593648e-04,
		2.46769937e-04, 1.48174763e-02, 7.95311721e-06,
		-1.07593648e-04, 7.95311721e-06, 1.50610482e-02
	};

const float soft_iron_matrix_1[9] = {
		3.17526854, 0.01433931, 0.00560107,
		0.01433931, 3.21201626, -0.04379719,
		0.00560107, -0.04379719, 3.43354134
	};

const float soft_iron_matrix_2[9] = {
		3.40679702, 0.01101284, -0.00739935,
		0.01101284, 3.14079664, 0.02803951,
		-0.00739935, 0.02803951, 3.42249977
	};

const float soft_iron_matrix_3[9] = {
		3.10825056, -0.01522626, 0.02956639,
		-0.01522626, 3.15557408, 0.00546862,
		0.02956639, 0.00546862, 3.15313759
	};

const float soft_iron_matrix_4[9] = {
		3.18787940e+00, 3.68232375e-04, 1.68648207e-02,
		3.68232375e-04, 3.07634271e+00, -9.27872714e-02,
		1.68648207e-02, -9.27872714e-02, 3.40286241e+00
	};

const float soft_iron_matrix_5[9] = {
		3.26474038e+00, -6.15091974e-04, 4.09854735e-02,
		-6.15091974e-04, 3.23317510e+00, 5.32502749e-03,
		4.09854735e-02, 5.32502749e-03, 3.55153346e+00
	};

const float soft_iron_matrix_6[9] = {
		3.34563381, 0.04966031, -0.11611146,
		0.04966031, 3.17642495, -0.0523192,
		-0.11611146, -0.0523192, 3.53919831
	};

const float soft_iron_matrix_7[9] = {
		3.35625315, 0.01636216, 0.01588383,
		0.01636216, 3.2162667, -0.11601007,
		0.01588383, -0.11601007, 3.48377098
	};

const float soft_iron_matrix_8[9] = {
		3.2905412, 0.04165543, 0.01908564,
		0.04165543, 3.25393366, -0.1010209,
		0.01908564, -0.1010209, 3.47482703
	};

const float soft_iron_matrix_9[9] = {
		3.3106875, 0.01727411, -0.05970636,
		0.01727411, 3.23683039, -0.09071214,
		-0.05970636, -0.09071214, 3.528728
	};

const float soft_iron_matrix_10[9] = {
		3.29956609, 0.00968322, -0.02386983,
		0.00968322, 3.20955961, -0.09138983,
		-0.02386983, -0.09138983, 3.42318501
	};

const float soft_iron_matrix_11[9] = {
		3.33143568, -0.01251123, -0.01517412,
		-0.01251123, 3.21756251, -0.08394593,
		-0.01517412, -0.08394593, 3.52076365
	};

const float hard_iron_bias_1[3] = {-0.33674186, -0.44338661, -0.07028313};

const float hard_iron_bias_2[3] = {-0.26162356, -0.56563037, -0.25755601};

const float hard_iron_bias_3[3] = {-0.34402661, -0.50748356, 0.25467058};

const float hard_iron_bias_4[3] = {-0.11066087, -0.54258921, 0.04271536};

const float hard_iron_bias_5[3] = {-0.26398805, -0.55285707, 0.16365326};

const float hard_iron_bias_6[3] = {-0.38714808, -0.45714108, 0.03695996};

const float hard_iron_bias_7[3] = {-0.12371357, -0.41663714, 0.00081922};

const float hard_iron_bias_8[3] = {-0.13255471, -0.72956792, -0.01009385};

const float hard_iron_bias_9[3] = {-0.21299763, -0.51983022, 0.02051473};

const float hard_iron_bias_10[3] = {-0.37519862, -0.27842791, -0.21166588};

const float hard_iron_bias_11[3] = {-0.28930092, -0.55488829, -0.02090365};

const float hard_iron_bias_100[3] = {11.67525977, -37.35063903, -15.4416279};

//const IronEntry soft_iron_table[] = {
//    {"0-100", soft_iron_matrix_100},
//	{"1-1", soft_iron_matrix_1},
//	{"1-2", soft_iron_matrix_2},
//	{"2-1", soft_iron_matrix_3},
//	{"2-2", soft_iron_matrix_4},
//	{"2-3", soft_iron_matrix_5},
//	{"3-1", soft_iron_matrix_6},
//	{"3-2", soft_iron_matrix_7},
//	{"3-3", soft_iron_matrix_8},
//	{"4-1", soft_iron_matrix_9},
//	{"4-2", soft_iron_matrix_10},
//	{"4-3", soft_iron_matrix_11},
//};
const IronEntry soft_iron_table[] = {
    {100, soft_iron_matrix_100},
	{1, soft_iron_matrix_1},
	{2, soft_iron_matrix_2},
	{3, soft_iron_matrix_3},
	{4, soft_iron_matrix_4},
	{5, soft_iron_matrix_5},
	{6, soft_iron_matrix_6},
	{7, soft_iron_matrix_7},
	{8, soft_iron_matrix_8},
	{9, soft_iron_matrix_9},
	{10, soft_iron_matrix_10},
	{11, soft_iron_matrix_11},
};

const IronEntry hard_iron_table[] = {
//    {"0-1", hard_iron_bias_1},
//    {"0-2", hard_iron_bias_2},
    {100, hard_iron_bias_100},
	{1, hard_iron_bias_1},
	{2, hard_iron_bias_2},
	{3, hard_iron_bias_3},
	{4, hard_iron_bias_4},
	{5, hard_iron_bias_5},
	{6, hard_iron_bias_6},
	{7, hard_iron_bias_7},
	{8, hard_iron_bias_8},
	{9, hard_iron_bias_9},
	{10, hard_iron_bias_10},
	{11, hard_iron_bias_11},
};

float round_x(float original, int power)
{
	float rounded = ((int)(original * powf(10, power) + 0.5f)) / powf(10, power);

	return rounded;
}

void matmul(float* A, float* B, float* C, int m, int n, int p)
{
    // A: m x n
    // B: n x p
    // C: m x p

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < p; j++)
        {
            C[i * p + j] = 0;
            for (int k = 0; k < n; k++)
            {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

void magnet_norm(struct bmm350_mag_temp_data *mag_calibrated_data)
{
	float norm_mag_calibrated_data = sqrtf(powf(mag_calibrated_data->x, 2) + powf(mag_calibrated_data->y, 2) + powf(mag_calibrated_data->z, 2));

	mag_calibrated_data->x = mag_calibrated_data->x / norm_mag_calibrated_data;
	mag_calibrated_data->y = mag_calibrated_data->y / norm_mag_calibrated_data;
	mag_calibrated_data->z = mag_calibrated_data->z / norm_mag_calibrated_data;
}

//const float *megnet_soft_iron_matrix_match(const char *id)
const float *megnet_soft_iron_matrix_match(const uint8_t id)
{
//	float current_soft_iron_matrix[9] = {0};

	for (int i = 0; i < sizeof(soft_iron_table) / sizeof(soft_iron_table[0]); ++i)
	{
//		if (strcmp(id, soft_iron_table[i].id) == 0)
		if (id == soft_iron_table[i].id)
		{
			return soft_iron_table[i].matrix;
	    }
	}

	return soft_iron_matrix_100;
}

const float *megnet_hard_iron_bias_match(const uint8_t id)
{
//	float current_soft_iron_matrix[9] = {0};
	for (int i = 0; i < sizeof(hard_iron_table) / sizeof(hard_iron_table[0]); ++i)
	{
//		if (strcmp(id, hard_iron_table[i].id) == 0)
		if (id == hard_iron_table[i].id)
		{
			return hard_iron_table[i].matrix;
		}
	}
//	if (strcmp(id, "0-1") == 0)
//	{
//		return hard_iron_bias_1;
//	}
//	else if (strcmp(id, "0-2") == 0)
//	{
//		return hard_iron_bias_2;
//	}
//	else if (strcmp(id, "0-3") == 0)
//	{
//		return hard_iron_bias_100;
//	}

	return hard_iron_bias_100;
}

void magnet_rescale(struct bmm350_mag_temp_data *mag_temp_data, struct bmm350_mag_temp_data *mag_rescaled_data)
{
	mag_rescaled_data->x = mag_temp_data->x / 100;
	mag_rescaled_data->y = mag_temp_data->y / 100;
	mag_rescaled_data->z = mag_temp_data->z / 100;
}

void magnet_calibration(struct bmm350_mag_temp_data *mag_temp_data, struct bmm350_mag_temp_data *mag_calibrated_data)
{
//	float soft_iron_matrix[9] = {
//		1.45873693e-02, 2.40345601e-04, -7.76704804e-05,
//		2.40345601e-04, 1.46510234e-02, -5.82792561e-05,
//		-7.76704804e-05, -5.82792561e-05, 1.51113481e-02
//	};

//	float soft_iron_matrix[9] = {
//		1.46592931e-02, 2.46769937e-04, -1.07593648e-04,
//		2.46769937e-04, 1.48174763e-02, 7.95311721e-06,
//		-1.07593648e-04, 7.95311721e-06, 1.50610482e-02
//	};
	const float *magnet_soft_matrix = megnet_soft_iron_matrix_match(board_id_slave);
	const float *magnet_hard_bias = megnet_hard_iron_bias_match(board_id_slave);
	float soft_iron_matrix[9];
	float hard_iron_bias[3];
	memcpy(soft_iron_matrix, magnet_soft_matrix, sizeof(soft_iron_matrix));
	memcpy(hard_iron_bias, magnet_hard_bias, sizeof(hard_iron_bias));

	float u[3] = {mag_temp_data->x / 100, mag_temp_data->y / 100, mag_temp_data->z / 100};

//	float hard_iron_bias[3] = {-25.54635429, -55.29757269, -27.39608831};
//	float hard_iron_bias[3] = {11.67525977, -37.35063903, -15.4416279};

	float u_bias[3] = {u[0] - hard_iron_bias[0], u[1] - hard_iron_bias[1], u[2] - hard_iron_bias[2]};

	float result[3];

	matmul(soft_iron_matrix, u_bias, result, 3, 3, 1);

	mag_calibrated_data->x = result[0];
	mag_calibrated_data->y = result[1];
	mag_calibrated_data->z = result[2];

	magnet_norm(mag_calibrated_data);

	// Round
//	mag_calibrated_data->x = round_x(mag_calibrated_data->x, 4);
//	mag_calibrated_data->y = round_x(mag_calibrated_data->y, 4);
//	mag_calibrated_data->z = round_x(mag_calibrated_data->z, 4);

	// Match IMU
	float mag_x = mag_calibrated_data->x;
	float mag_y = mag_calibrated_data->y;
	float mag_z = mag_calibrated_data->z;
	mag_calibrated_data->x = mag_y;
	mag_calibrated_data->y = mag_x;
	mag_calibrated_data->z = -mag_z;
}

