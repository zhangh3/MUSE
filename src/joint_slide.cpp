/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */


#include "joint.h"
#include "math_extra.h"

using namespace MUSE_NS;
using namespace Eigen;
void Joint::constrainteq_slide()
{
	Matrix<double, 3, 4> T1_I, T2_I;
	Matrix3d Aax, crsom1, crsom2;
	Vector3d Dp1, Dp2, x1, x2, v1, v2, dx, dv, dDp, b_part1, b_part2, b_part3, b_part4, Tdqd1_I, Tdqd2_I;

	T1_I = body[0]->DCM * body[0]->T;
	T2_I = body[1]->DCM * body[1]->T;

	Aax = MathExtra::crs(body[0]->DCM * axis1);


	x1 = body[0]->pos;
	x2 = body[1]->pos;
	v1 = body[0]->vel;
	v2 = body[1]->vel;
	dx = x1 - x2;
	dv = v1 - v2;

	Dp1 = body[0]->DCM * point1;
	Dp2 = body[1]->DCM * point2;
	dDp = Dp1 - Dp2;

	crsom1 = MathExtra::crs(body[0]->DCM * body[0]->omega);
	crsom2 = MathExtra::crs(body[1]->DCM * body[1]->omega);


	Tdqd1_I = body[0]->DCM * body[0]->Td * body[0]->quatd;
	Tdqd2_I = body[1]->DCM * body[1]->Td * body[1]->quatd;


	b_part1 = MathExtra::crs(dx + dDp) * (crsom1 * crsom1 * body[0]->DCM * axis1- Aax* Tdqd1_I);
	b_part2 = crsom1 * Aax * (crsom1 * Dp1 - crsom2 * Dp2 + dv);
	b_part3 = Aax * (crsom1 * crsom1 * Dp1 - crsom2 * crsom2 * Dp2 - MathExtra::crs(Dp1) * Tdqd1_I + MathExtra::crs(Dp2) * Tdqd2_I);
	b_part4 = Tdqd2_I - Tdqd1_I;

	A1 << Aax, Aax * MathExtra::crs(dx - Dp2) * T1_I,
		Matrix3d::Zero(), T1_I;

	A2 << -Aax, Aax * MathExtra::crs(Dp2) * T2_I,
		Matrix3d::Zero(), -T2_I;

	b << b_part1 - 2 * b_part2 - b_part3,
		b_part4;
}