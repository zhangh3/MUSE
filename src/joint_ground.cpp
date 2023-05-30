/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

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
void Joint::constrainteq_ground()
{
	A1 << Matrix3d::Identity(), MatrixXd::Zero(3,4),
		MatrixXd::Zero(4, 3), Matrix4d::Identity();

	b << VectorXd::Zero(7);
}


