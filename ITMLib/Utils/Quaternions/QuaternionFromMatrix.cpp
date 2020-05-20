//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/3/20.
//  Copyright (c) 2020 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
//stdlib
#include <cassert>

//local
#include "QuaternionFromMatrix.h"

namespace ITMLib{

static int QuaternionFromRotationMatrix_variant(const double* matrix) {
	int variant = 0;
	if ((matrix[4] > -matrix[8]) && (matrix[0] > -matrix[4]) && (matrix[0] > -matrix[8])) {
		variant = 0;
	} else if ((matrix[4] < -matrix[8]) && (matrix[0] >
	                                        matrix[4]) && (matrix[0] > matrix[8])) {
		variant = 1;
	} else if ((matrix[4] > matrix[8]) && (matrix[0] <
	                                       matrix[4]) && (matrix[0] < -matrix[8])) {
		variant = 2;
	} else if ((matrix[4] <
	            matrix[8]) && (matrix[0] < -matrix[4]) && (matrix[0] < matrix[8])) {
		variant = 3;
	}
	return variant;
}

void QuaternionFromRotationMatrix(const double* matrix, double* q) {
/* taken from "James Diebel. Representing Attitude: Euler
	Angles, Quaternions, and Rotation Vectors. Technical Report, Stanford
	University, Palo Alto, CA."*/

	// choose the numerically best variant...
	int variant = QuaternionFromRotationMatrix_variant(matrix);
	double denom = 1.0;
	if (variant == 0) {
		denom += matrix[0] + matrix[4] + matrix[8];
	} else {
		int tmp = variant * 4;
		denom += matrix[tmp - 4];
		denom -= matrix[tmp % 12];
		denom -= matrix[(tmp + 4) % 12];
	}
	denom = sqrt(denom);
	q[variant] = 0.5*denom;

	denom *= 2.0;
	switch (variant) {
		case 0:
			q[1] = (matrix[5] - matrix[7]) / denom;
			q[2] = (matrix[6] - matrix[2]) / denom;
			q[3] = (matrix[1] - matrix[3]) / denom;
			break;
		case 1:
			q[0] = (matrix[5] - matrix[7]) / denom;
			q[2] = (matrix[1] + matrix[3]) / denom;
			q[3] = (matrix[6] + matrix[2]) / denom;
			break;
		case 2:
			q[0] = (matrix[6] - matrix[2]) / denom;
			q[1] = (matrix[1] + matrix[3]) / denom;
			q[3] = (matrix[5] + matrix[7]) / denom;
			break;
		case 3:
			q[0] = (matrix[1] - matrix[3]) / denom;
			q[1] = (matrix[6] + matrix[2]) / denom;
			q[2] = (matrix[5] + matrix[7]) / denom;
			break;
		default:
			assert(false);
			break;
	}

	if (q[0] < 0.0f) for (int i = 0; i < 4; ++i) q[i] *= -1.0f;
}
} // namespace ITMLib