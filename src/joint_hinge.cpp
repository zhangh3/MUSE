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
void Joint::constrainteq_hinge()
{
	Matrix<double, 3, 4> T1_I, T2_I;
	Vector3d ap11, ap12, ap21, ap22, b_part11, b_part12, b_part21, b_part22, Tdqd1_I, Tdqd2_I;
	Matrix3d crsap11, crsap12, crsap21, crsap22, crsom1, crsom2, scrsom1, scrsom2;

	axis2 = body[1]->DCM.transpose() * body[0]->DCM * axis1;

	T1_I = body[0]->DCM * body[0]->T;
	T2_I = body[1]->DCM * body[1]->T;

	ap11 = body[0]->DCM * (point1 + axis1);
	ap12 = body[0]->DCM * (point1 - axis1);
	ap21 = body[1]->DCM * (point2 + axis2);
	ap22 = body[1]->DCM * (point2 - axis2);
	crsap11 = MathExtra::crs(ap11);
	crsap12 = MathExtra::crs(ap12);
	crsap21 = MathExtra::crs(ap21);
	crsap22 = MathExtra::crs(ap22);

	crsom1 = MathExtra::crs(body[0]->DCM * body[0]->omega);
	crsom2 = MathExtra::crs(body[1]->DCM * body[1]->omega);
	scrsom1 = crsom1 * crsom1;
	scrsom2 = crsom2 * crsom2;

	Tdqd1_I = body[0]->DCM * body[0]->Td * body[0]->quatd;
	Tdqd2_I = body[1]->DCM * body[1]->Td * body[1]->quatd;

	b_part11 = -scrsom1 * ap11 + crsap11 * Tdqd1_I;
	b_part12 = -scrsom1 * ap12 + crsap12 * Tdqd1_I;
	b_part21 = -scrsom2 * ap21 + crsap21 * Tdqd2_I;
	b_part22 = -scrsom2 * ap22 + crsap22 * Tdqd2_I;

	A1 << Matrix3d::Identity(),-crsap11 * T1_I,
		  Matrix3d::Identity(),-crsap12 * T1_I;
	A2 <<-Matrix3d::Identity(), crsap21 * T2_I,
		 -Matrix3d::Identity(), crsap22 * T2_I;

	b << b_part11 - b_part21,
		 b_part12 - b_part22;
}


