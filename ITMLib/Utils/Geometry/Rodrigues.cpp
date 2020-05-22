//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/21/20.
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
#include "Rodrigues.h"

namespace ITMLib {

Matrix3f Rodrigues(Vector3f euler_rotation_vector){
	Matrix3f rotation_matrix;

	Matrix3f& R = rotation_matrix;
	Vector3f& w = euler_rotation_vector;

	const float one_6th = 1.0f / 6.0f;
	const float one_20th = 1.0f / 20.0f;

	float theta_sq = dot(w, w);
	float theta = sqrt(theta_sq);

	float A, B;

	if (theta_sq < 1e-8f){
		A = 1.0f - one_6th * theta_sq; B = 0.5f;
	} else {
		if (theta_sq < 1e-6f) {
			float C = one_6th * (1.0f - one_20th * theta_sq);
			A = 1.0f - theta_sq * C;
			B = 0.5f - 0.25f * one_6th * theta_sq;
		} else {
			float inv_theta = 1.0f / theta;
			A = sinf(theta) * inv_theta;
			B = (1.0f - cosf(theta)) * (inv_theta * inv_theta);
		}
	}

	float wx2 = w.x * w.x, wy2 = w.y * w.y, wz2 = w.z * w.z;
	R.m[0 + 3 * 0] = 1.0f - B*(wy2 + wz2);
	R.m[1 + 3 * 1] = 1.0f - B*(wx2 + wz2);
	R.m[2 + 3 * 2] = 1.0f - B*(wx2 + wy2);

	float a, b;
	a = A * w.z, b = B * (w.x * w.y);
	R.m[0 + 3 * 1] = b - a;
	R.m[1 + 3 * 0] = b + a;

	a = A * w.y, b = B * (w.x * w.z);
	R.m[0 + 3 * 2] = b + a;
	R.m[2 + 3 * 0] = b - a;

	a = A * w.x, b = B * (w.y * w.z);
	R.m[1 + 3 * 2] = b - a;
	R.m[2 + 3 * 1] = b + a;

	return rotation_matrix;
}

} // namespace ITMLib
