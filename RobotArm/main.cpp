#include <iostream>

using namespace std;
typedef unsigned int uint;

#include "RobotArmDLL.h"
#pragma comment(lib, "RobotArmDLL.lib")

#if defined(_WIN32)
static void load_data(string file_name, unsigned int row, unsigned int col, double *data) {
	FILE *fp_in;
	const int buffer = 1000000;
	char *ptr, basic[buffer], *token;
	fopen_s(&fp_in, file_name.c_str(), "r");
	uint i = 0, j = 0;
	while (fgets(basic, buffer, fp_in) != NULL)
	{
		j = 0;
		ptr = strtok_s(basic, "\t", &token);
		while (ptr != NULL) {
			data[i * col + j] = atof(ptr);
			ptr = strtok_s(NULL, "\t", &token);
			j++;
		}
		i++;
		if (i >= row) break;
	}
	fclose(fp_in);
}
#endif

int main() {
	pRobotArm robot = RobotArmConstruct();

	double epsilon = 1e9;
	unsigned int maximum_iter = 1;
	double damped_factor = 0.0001;
	RobotArmInitialization(epsilon, maximum_iter, damped_factor, robot);

	uint data_size = 2001;
	uint col_size = 7;
	double *input = new double[data_size * col_size];

	load_data("inverse_kinematics_input_end.txt", data_size, col_size, input);

	double q_init[7] = { -6.86884120000000e-09 , 1.57079630000000, -3.62142600000000e-08 ,-0.785398130000000, -3.00355730000000e-08, 0.523598730000000, 3.51772050000000e-08 };
	double Xd[6] = { 0, }, q_input[7] = { 0, }, q_output[7] = { 0, }, X_output[6] = { 0, };

	for (uint indx = 0; indx < data_size; indx++) {
		Xd[0] = input[indx * col_size + 1];
		Xd[1] = input[indx * col_size + 2];
		Xd[2] = input[indx * col_size + 3];
		Xd[3] = input[indx * col_size + 4];
		Xd[4] = input[indx * col_size + 5];
		Xd[5] = input[indx * col_size + 6];

		if (indx == 0) {
			memcpy(q_input, q_init, sizeof(double) * 7);
		}
		else {
			memcpy(q_input, q_output, sizeof(double) * 7);
		}

		RobotArmInverseKinematics(q_input, Xd, q_output, X_output, robot);

		cout << "indx : " << indx << endl;
	}

	return 0;
}