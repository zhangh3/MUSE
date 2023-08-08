/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "string.h"
#include "body.h"
#include "math_extra.h"
#include "error.h"
#include "memory.h"

using namespace MUSE_NS;

Body::Body(MUSE *muse) : Pointers(muse)
{
	name=NULL;
	pos   << 0,0,0 ;
	vel   << 0,0,0 ;
	quat  << 0, 0, 0, 1;          //  {w,x,y,z}   w + xi + yj + zk;
	omega << 0, 0, 0;
	quatd << 0, 0, 0, 0;
	mass = 1;
	set_Inertia(1,1,1,0,0,0);

	mySystem = NULL;
	IDinSystem = -1;
	IDinMuse = -1;
}
Body::~Body()
{
	delete [] name;
}

void Body::set_Name(char *newname)
{
  int n = strlen(newname) + 1;
  if(n<2) error->one(FLERR,"Body name is empty!");
  name = new char[n];
  strcpy(name,newname);
}


void Body::set_Mass(double newmass)
{
  if(newmass<=0){
	  char str[128];
	  sprintf(str,"Mass of body %s is not positive",name);
	  error->one(FLERR,str);
  }
  mass = newmass;
}


void Body::set_Quaternion(double *newquat)
{
	quat=Eigen::Map<Eigen::Vector4d>(newquat, 4);
	double len = quat.norm();
	if(len < QUATERR){
		char str[128];
		sprintf(str,"The quaternion modulus of body %s is too small",name);
		error->all(FLERR,str);
	}

	if(fabs(len-1) > NORMERR){
		char str[128];
		sprintf(str,"Quaternion of body %s is not normalized",name);
		error->warning(FLERR,str);
	}
	quat.normalize();
	T << 2 * (quat(3) * Eigen::Matrix3d::Identity() - MathExtra::crs(quat.head(3))), -2 * quat.head(3);
	quatd = 0.25 * T.transpose() * omega; // 修改quatd
}

void MUSE_NS::Body::set_Omega(double wx, double wy, double wz)
{
	omega << wx, wy, wz;
	T << 2 * (quat(3) * Eigen::Matrix3d::Identity() - MathExtra::crs(quat.head(3))), -2 * quat.head(3);
	quatd = 0.25 * T.transpose() * omega;
}


void Body::set_Inertia(double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
{
	if(Ixx<=0 ||Iyy<=0||Izz<=0){
		char str[128];
		sprintf(str,"Inertia of body %s is not positive",name);
		error->one(FLERR,str);
	}
	inertia << Ixx, Ixy, Ixz,
		       Ixy, Iyy, Iyz,
		       Ixz, Iyz, Izz;
}

void Body::refresh()
{
	quat.normalize();// FIXME:归一化quat，应该返回x
	T  << 2 * ( quat(3) * Eigen::Matrix3d::Identity() - MathExtra::crs( quat.head(3))), -2 *  quat.head(3);
	DCM = (quat(3) * quat(3) - quat.head(3).transpose() * quat.head(3)) * Eigen::Matrix3d::Identity() 
		 + 2 * quat.head(3) * quat.head(3).transpose() + 2 * quat(3) * MathExtra::crs(quat.head(3));
	omega = T * quatd;
	quatd = 0.25 * T.transpose() * omega; // FIXME:归一化quatd，应该返回xd
	Td << 2 * (quatd(3) * Eigen::Matrix3d::Identity() - MathExtra::crs(quatd.head(3))), -2 * quatd.head(3);
	inertia4 = T.transpose() * inertia * T;
}