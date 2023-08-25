/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_BODY_H
#define MUSE_BODY_H

#define QUATERR 1E-8
#define NORMERR 1E-3

#include "pointers.h"
#include "Eigen/Eigen"
#include "MUSEsystem.h"

namespace MUSE_NS {

class Body : protected Pointers {
public:

    char *name;                        //body name

	Eigen::Vector3d pos;               //centroid position in inertial frame
	Eigen::Vector3d vel;               //centroid velocity in inertial frame
	Eigen::Vector4d quat;              //pose quaternion

/* omega is calculated according to quatd
   the quatd must be modified when setting omega */

	Eigen::Vector4d quatd;             //time derivative quat
	Eigen::Vector3d omega;             //angular velocity in body frame
	double mass;
	Eigen::Matrix3d inertia;           //inertia in body frame


	Eigen::Matrix<double, 3, 4> T,Td;  //T: transformation matrix from quatd to omega
	                                   //Td: time derivative of T
	Eigen::Matrix3d DCM;               //transformation matrix from body frame to inertial frame
	Eigen::Matrix4d inertia4;          //inertia in quaternion form

	int IDinSystem;
	int IDinMuse;

	Body(class MUSE *);
	~Body();




	void set_Name(char *);
	void set_Quaternion(double *);
	void set_Omega(double, double, double);
	void set_Mass(double);
	void set_Inertia(double,double,double,double,double,double);
	void refresh();
	

private:

};


}

#endif