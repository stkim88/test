#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <iostream>

//
#include <vector>
#include <string>
#include <algorithm>
//
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include "map_utils.h"
#include "Projection.h"
#include "fasttrigo.h"

using namespace cv;

// ==============================================================================
//                               INLINE HELPER FUNCTIONS
// ==============================================================================

inline void inverse_matrix3x3(float(*A)[3], float(*Ai)[3])
{
	const float Det = A[0][0] * A[1][1] * A[2][2] - A[0][0] * A[1][2] * A[2][1] - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

	Ai[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / Det;
	Ai[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / Det;
	Ai[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / Det;
	Ai[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / Det;
	Ai[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / Det;
	Ai[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / Det;
	Ai[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / Det;
	Ai[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / Det;
	Ai[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / Det;
}

inline void multiply_3x3and3x3(float(*A)[3], float(*B)[3], float(*R)[3])
{
	int i, j, k;

	float val;

	for (j = 0; j < 3; ++j)
	{
		for (i = 0; i < 3; ++i)
		{
			val = 0;
			for (k = 0; k < 3; ++k)
			{
				val += A[j][k] * B[k][i];
			}
			R[j][i] = val;
		}
	}
}

inline void multiply_3x3and3x1(float(*A)[3], float *B, float *R)
{
	const float a00 = A[0][0];
	const float a01 = A[0][1];
	const float a02 = A[0][2];

	const float a10 = A[1][0];
	const float a11 = A[1][1];
	const float a12 = A[1][2];

	const float a20 = A[2][0];
	const float a21 = A[2][1];
	const float a22 = A[2][2];
	
	const float b0 = B[0];
	const float b1 = B[1];
	const float b2 = B[2];

	R[0] = a00 * b0 + a01 * b1 + a02 * b2;
	R[1] = a10 * b0 + a11 * b1 + a12 * b2;
	R[2] = a20 * b0 + a21 * b1 + a22 * b2;
}

inline void xyz2tp(float *xyz, float *tp)
{
	const float x = xyz[0];
	const float y = xyz[1];
	const float z = xyz[2];

	float x2y2 = FTA::sqrt(x * x + y * y);

	tp[1] = (y > 0 ? 1 : y < 0 ? -1 : 0) * FTA::acos(x / x2y2);
	tp[0] = FTA::acos(z);
}

inline float matrix_norm(float *A)
{
	float Norm = FTA::sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
	return Norm;
}

class VR360_Player
{
public:
	void player_init(cv::Size omnidirectinalSize, float phi_i, float theta_i, float psi_i, float W_i, float H_i, float fov_i);

	void player_reinit(float fov_i);

	void getImage(cv::Mat src);

	void key_input();

	void viewport_rotation();

	void mapping();

	void show(float sizeRatio, float o_time = 0);

	void release();

	float phi;
	float theta;
	float psi;

	int stopIndex;
	int frameIndex;

	// view port frame
	cv::Mat viewportImage;
	
private:
	Projection fisheyeCam;

	cv::Mat viewportNormPlane;
	cv::Mat viewportTable;

	// omni directional video frame
	cv::Mat imgOmnidirectional;
	cv::Mat imgOmnidirectionalShow;
	
	int omnidirectionalW;
	int omnidirectionalH;
	int omnidirectionalWidthStep;

	float viewportW;
	float viewportH;
	int viewportWstep;
	float fov; // degree
	
	float ctheta;
	float cphi;
	float cpsi;
	float stheta;
	float sphi;
	float spsi;

	char c = NULL;
	
	//rotation
	float Rx[3][3];
	float Ry[3][3];
	float Rz[3][3];
	float R1[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	float R_wc[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

	//viewport inrinsic
	float K[3][3];
};

