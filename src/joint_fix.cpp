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

/* ---------------------------------------------------------------------- */
void Joint::constrainteq_fix()
{
    

	Matrix<double, 3, 4> T1_I, T2_I;
	Vector3d point1_I, point2_I, b_part1, b_part2, brot1, brot2;

	Matrix3d crsom1, crsom2, crsp1_I, crsp2_I;

	T1_I = body[0]->DCM * body[0]->T;
	T2_I = body[1]->DCM * body[1]->T;

	point1_I = body[0]->DCM * (point1);
	point2_I = body[1]->DCM * (point2);

	crsp1_I = MathExtra::crs(point1_I);
	crsp2_I = MathExtra::crs(point2_I);

	crsom1 = MathExtra::crs(body[0]->DCM * body[0]->omega);
	crsom2 = MathExtra::crs(body[1]->DCM * body[1]->omega);

	brot1 = body[0]->DCM * body[0]->Td * body[0]->quatd;
	brot2 = body[1]->DCM * body[1]->Td * body[1]->quatd;

	b_part1 = -crsom1 * crsom1 * point1_I + crsp1_I * brot1;
	b_part2 = -crsom2 * crsom2 * point2_I + crsp2_I * brot2;

	A1 << Matrix3d::Identity(), crsp1_I* T1_I,
		Matrix3d::Zero(), T1_I;
	A2 << -Matrix3d::Identity(), crsp2_I* T2_I,
		Matrix3d::Zero(), -T2_I;
	b << b_part1 - b_part2,
		brot2 - brot1;
}


