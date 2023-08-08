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
void Joint::constrainteq_sphere()
{
	Matrix3d crsom1 = MathExtra::crs(body[0]->DCM * body[0]->omega);
	Matrix3d crsom2 = MathExtra::crs(body[1]->DCM * body[1]->omega);

	Vector3d point1_I = body[0]->DCM * (point1);
	Vector3d point2_I = body[1]->DCM * (point2);

	Matrix<double, 3, 4> A_part1, A_part2;
	Vector3d b_part1, b_part2;

	A_part1 = -MathExtra::crs(point1_I) * body[0]->DCM * body[0]->T;
	A_part2 =  MathExtra::crs(point2_I) * body[1]->DCM * body[1]->T;

	b_part1 = -crsom1 * crsom1 * point1_I + point1_I.cross(body[0]->DCM * body[0]->Td * body[0]->quatd);
	b_part2 = -crsom2 * crsom2 * point2_I + point2_I.cross(body[1]->DCM * body[1]->Td * body[1]->quatd);


	A1 <<  Matrix3d::Identity(), A_part1;
	A2 << -Matrix3d::Identity(), A_part2;
	b << b_part1 - b_part2;
}


