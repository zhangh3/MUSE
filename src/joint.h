/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_JOINT_H
#define MUSE_JOINT_H

#include "pointers.h"
#include "body.h"
#include "MUSEsystem.h"
#include "Eigen/Eigen"
namespace MUSE_NS {

class Joint : protected Pointers {
public:

    char *name;
	
	Eigen::Vector3d axis1;
	Eigen::Vector3d axis2;
	Eigen::Vector3d point1;
	Eigen::Vector3d point2;

	Body *body[2];
	System * mySystem;
	int IDinSystem;
	int IDinMuse;

	Eigen::MatrixXd A1;
	Eigen::MatrixXd A2;
	Eigen::VectorXd b;

	
	void constrainteq_sphere();
	void constrainteq_ground();
	void (MUSE_NS::Joint::* constrainteq)();
	void uglyconstrainteq();


	Joint(class MUSE *);
	~Joint();

	void set_Name(char*);
	void set_type(int);
	int get_type();

private:
	int type;

};

}

#endif