/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "joint.h"
#include "enums.h"
#include "error.h"

using namespace MUSE_NS;
using namespace Eigen;

Joint::Joint(MUSE *muse) : Pointers(muse)
{
	name=NULL;
	type=FIX;
	point1 << 0,0,0;
	point2 << 0,0,0;
	axis1 << 0,0,1;
	axis2 << 0,1,0;
	body[0] = NULL;
	body[1] = NULL;

	constrainteq = NULL;

	mySystem = NULL;
	IDinSystem = -1;
	IDinMuse = -1;
}

/* ---------------------------------------------------------------------- */

void Joint::set_Name(char* newname)
{
	int n = strlen(newname) + 1;
	if (n < 2) error->one(FLERR, "Joint name is empty!");
	name = new char[n];
	strcpy(name, newname);
}

void Joint::set_type(int newtype)
{
	switch (newtype)
	{
	case SPHERE:
		type = newtype;
		constrainteq = &MUSE_NS::Joint::constrainteq_sphere;
		A1.resize(3, 7);
		A2.resize(3, 7);
		b.resize(3);
		break;
	case GROUND:
		type = newtype;
		constrainteq = &MUSE_NS::Joint::constrainteq_ground;
		A1.resize(7, 7);
		A2.resize(0, 0);
		b.resize(7);
		break;
	default:
		error->all(FLERR, "Undefined joint type!");
		break;
	}
	
}


void Joint::uglyconstrainteq()
{
	switch (type)
	{
	case SPHERE:
		constrainteq_sphere();
		break;
	case GROUND:
		constrainteq_ground();
		break;
	default:
		error->all(FLERR, "Undefined joint type!");
		break;
	}
}

int MUSE_NS::Joint::get_type()
{
	return type;
}

Joint::~Joint()
{
	delete [] name;
}