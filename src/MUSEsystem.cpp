/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "string.h"
#include "MUSEsystem.h"
#include "body.h"
#include "joint.h"
#include "memory.h"
#include "error.h"
#include "enums.h"

//////test
#include <iostream>
#include <iomanip>
/////

#define EPS 1E-16

using namespace MUSE_NS;

System::System(MUSE *muse) : Pointers(muse)
{
	name = NULL;
    nBodies = maxBodies = 0;
	nJoints = maxJoints = 0;
	body  = NULL;
	joint = NULL;
	IDinMuse = -1;
	timenow = 0;
	dt = 1E-4;
	ga << 0, -9.8, 0;

	logflag = true;
}

/* ---------------------------------------------------------------------- */

System::~System()
{
	memory->sfree(body);
	memory->sfree(joint);
	delete[] name;
}


void System::set_Name(char* newname)
{
	int n = strlen(newname) + 1;
	if (n < 2) error->one(FLERR, "Body name is empty!");
	name = new char[n];
	strcpy(name, newname);
}

void System::solve(double endtime)
{
	int ibody, ijoint;

	for (ibody = 0; ibody < nBodies; ibody++) muse->body[ibody]->refresh();
	for (ijoint = 0; ijoint < nJoints; ijoint++) muse->joint[ijoint]->uglyconstrainteq();

	for (ibody = 0; ibody < nBodies; ibody++) {
		x.segment(ibody * 7, 3) = body[ibody]->pos;
		x.segment(ibody * 7+3, 4) = body[ibody]->quat;
		xd.segment(ibody * 7, 3) = body[ibody]->vel;
		xd.segment(ibody * 7 + 3, 4) = body[ibody]->quatd;
	}
	calxdd();

	if (logflag)
	{
		xlognow << timenow, x, xd, xdd;
		xlog.clear();
		xlog.push_back(xlognow);
	}
	while (timenow < endtime) {
//		update_euler();
		update_RK4();
		//using namespace std;
		//cout << setprecision(2) << x.transpose() << endl << endl;
		if (logflag)
		{
			xlognow << timenow, x, xd, xdd;
			xlog.push_back(xlognow);
		}
	}
	
}
void System::calxdd()
{
	using namespace Eigen;
	makeBigM();
	makeBigF();
	makeBigAb();

#ifdef SPARSE
	JacobiSVD<MatrixXd> svdA(MatrixXd(A), ComputeThinU | ComputeThinV);
#else
	JacobiSVD<MatrixXd> svdA(A, ComputeThinU | ComputeThinV);
#endif //SPARSE
	MatrixXd singularValues_inv = svdA.singularValues();//奇异值
	double pinvtoler = singularValues_inv(0) * EPS;
	for (int i = 0; i < singularValues_inv.rows(); ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	MatrixXd AA = (MatrixXd::Identity(7 * nBodies, 7 * nBodies) - svdA.matrixV() * singularValues_inv.asDiagonal() * svdA.matrixU().transpose() * A) * M;
	MatrixXd Mbar(AA.rows() + A.rows(), AA.cols());

#ifdef SPARSE
	Mbar << AA, MatrixXd(A);
#else
	Mbar << AA, A;
#endif //SPARSE

	JacobiSVD<MatrixXd> svdMbar(Mbar, ComputeThinU | ComputeThinV);
	VectorXd longb(F.rows() + b.rows());
	longb << F, b;

	xdd = svdMbar.solve(longb);
}
void System::update_euler()
{
	int ibody, ijoint;
	x +=  xd * dt;
	xd += xdd * dt;
	x2body();
	calxdd();
	timenow += dt;

}

void System::update_RK4()
{
	using namespace Eigen;
	double halfdt = (dt / 2.0);
	VectorXd x0, xd0, xd1, xd2, xd3, xdd0, xdd1, xdd2, xdd3, singularValues_inv, longb;
	int ibody, ijoint;

	x0 = x;
	xd0 = xd;
	xdd0 = xdd; //K1

	x  = x0 + xd0  * halfdt;
	xd = xd0 + xdd0 * halfdt;
	timenow += halfdt;
	x2body();
	calxdd();
	xd1 = xd;
	xdd1 = xdd; //K2
	

	x = x0 + xd1 * halfdt;
	xd = xd0 + xdd1 * halfdt;
	x2body();
	calxdd();
	xd2 = xd;
	xdd2 = xdd;  //K3
	

	x = x0 + xd2 * dt;
	xd = xd0 + xdd2 * dt;
	timenow += halfdt;
	x2body();
	calxdd();
	xd3 = xd;
	xdd3 = xdd;  //K4

	x  = x0  + (xd0  + 2 * xd1  + 2 * xd2  + xd3)  * (dt / 6.0);
	xd = xd0 + (xdd0 + 2 * xdd1 + 2 * xdd2 + xdd3) * (dt / 6.0);
	x2body();
	calxdd();
}

void System::x2body()
{
	int ibody, ijoint, ibegin=0;
	for (ibody = 0; ibody < nBodies; ibody++)
	{
		body[ibody]->pos  = x.segment(ibegin, 3);
		body[ibody]->vel = xd.segment(ibegin, 3);
		ibegin += 3;
		body[ibody]->quat  =  x.segment(ibegin, 4);
	//	body[ibody]->quat.normalize(); //注意将其变成修正x等

		body[ibody]->quatd = xd.segment(ibegin, 4);
		ibegin += 4;
	}
	for (ibody = 0; ibody < nBodies; ibody++) muse->body[ibody]->refresh();
	for (ijoint = 0; ijoint < nJoints; ijoint++) muse->joint[ijoint]->uglyconstrainteq();
}

int System::add_Body(Body *bodynow)
{
	int ibody;

	for (ibody = 0; ibody < nBodies; ibody++)
		if (strcmp(bodynow->name, body[ibody]->name) == 0) break;

	if (ibody < nBodies) {
		error->all(FLERR, "Repeatedly added the same body into the system!");
	}
	else {
		if (nBodies == maxBodies) {
			maxBodies += DELTA;
			body = (Body**)memory->srealloc(body, maxBodies * sizeof(Body*), "muse:body");
		}
	}
	body[ibody] = bodynow;
	body[ibody]->IDinSystem = ibody;
	body[ibody]->mySystem=this;
	nBodies++;
	return ibody;
}

int System::add_Joint(Joint *jointnow)
{
	int ijoint;

	for (ijoint = 0; ijoint < nJoints; ijoint++)
		if (strcmp(jointnow->name, joint[ijoint]->name) == 0) break;

	if (ijoint < nJoints) {
		error->all(FLERR, "Repeatedly added the same joint into the system!");
	}
	else {
		if (nJoints == maxJoints) {
			maxJoints += DELTA;
			joint = (Joint**)memory->srealloc(joint, maxJoints * sizeof(Joint*), "muse:joint");
		}
	}
	joint[ijoint] = jointnow;
	joint[ijoint]->IDinSystem = ijoint;
	joint[ijoint]->mySystem = this;

	nJoints++;
	return ijoint;
}


void System::setup()
{
	int rowsum, ijoint;
	rowsum = 0;
	for (ijoint = 0; ijoint < nJoints; ijoint++)
		rowsum += joint[ijoint]->A1.rows();

	A.resize(rowsum+ nBodies, 7 * nBodies);
	b.resize(rowsum + nBodies);
	M.resize(7 * nBodies, 7 * nBodies);
	F.resize(7 * nBodies);
	x.resize(7 * nBodies);
	xd.resize(7 * nBodies);
	xdd.resize(7 * nBodies);
	xlognow.resize(21 * nBodies + 1);
}

void System::makeBigF()
{
	int ibody;
	F.setZero();
	for (ibody = 0; ibody < nBodies; ibody++) 
		F.segment(ibody * 7, 3) << body[ibody]->mass * ga;
}
void System::makeBigM()
{
	int ibody,ibegin,i,j;
	double nowdata;

	M.setZero();

#ifdef SPARSE
	std::vector < Eigen::Triplet <double> > triplets;
	for (ibody = 0; ibody < nBodies; ibody++)
	{
		ibegin = ibody * 7;
		triplets.emplace_back(ibegin, ibegin, body[ibody]->mass);
		ibegin++;
		triplets.emplace_back(ibegin, ibegin, body[ibody]->mass);
		ibegin++;
		triplets.emplace_back(ibegin, ibegin, body[ibody]->mass);
		ibegin++;
		for (i= 0; i < 4; i++)
			for (j = 0; j < 4; j++)
			{
				nowdata = body[ibody]->inertia4(i, j);
				if (nowdata != 0) triplets.emplace_back(ibegin + i, ibegin + j, nowdata);
			}
	}
	M.setFromTriplets(triplets.begin(), triplets.end());
#else
	for (ibody = 0; ibody < nBodies; ibody++)
	{
		ibegin = ibody * 7;
		M(ibegin, ibegin) = body[ibody]->mass;
		ibegin++;
		M(ibegin, ibegin) = body[ibody]->mass;
		ibegin++;
		M(ibegin, ibegin) = body[ibody]->mass;
		ibegin++;
		M.block(ibegin, ibegin, 4, 4) = body[ibody]->inertia4;
	}
#endif // SPARSE
}

void System::makeBigAb()
{
	int ibody, ijoint, ibegin, i, j, nowrows, b1, b2;
	double nowdata;
	A.setZero();
#ifdef SPARSE
	std::vector < Eigen::Triplet <double> > triplets;
	ibegin = 0;
	for (ijoint = 0; ijoint < nJoints; ijoint++)
	{
		nowrows = joint[ijoint]->A1.rows();
		b1 = 7 * joint[ijoint]->body[0]->IDinSystem;
		if (joint[ijoint]->get_type() == GROUND)
		{
			for (i = 0; i < nowrows; i++)
				for (j = 0; j < 7; j++)
				{
					nowdata = joint[ijoint]->A1(i, j);
					if (nowdata != 0) triplets.emplace_back(ibegin + i, b1 + j, nowdata);
				}
		}
		else
		{
			b2 = 7 * joint[ijoint]->body[1]->IDinSystem;
			for (i = 0; i < nowrows; i++)
				for (j = 0; j < 7; j++)
				{
					nowdata = joint[ijoint]->A1(i, j);
					if (nowdata != 0) triplets.emplace_back(ibegin + i, b1 + j, nowdata);
					nowdata = joint[ijoint]->A2(i, j);
					if (nowdata != 0) triplets.emplace_back(ibegin + i, b2 + j, nowdata);
				}
		}
		b.segment(ibegin, nowrows) << joint[ijoint]->b;

		ibegin += nowrows;
	}
	for (ibody = 0; ibody < nBodies; ibody++)
	{
		for (j = 0; j < 4; j++)
		{
			nowdata = 2 * body[ibody]->quat(j);
			if (nowdata != 0) triplets.emplace_back(ibegin, 3 + j + 7 * ibody, nowdata);
		}

		b(ibegin) = -2.0 * body[ibody]->quatd.dot(body[ibody]->quatd);

		ibegin++;
	}
	A.setFromTriplets(triplets.begin(), triplets.end());
#else	
	ibegin = 0;
	for (ijoint = 0; ijoint < nJoints; ijoint++)
	{
		nowrows = joint[ijoint]->A1.rows();
		b1 = 7 * joint[ijoint]->body[0]->IDinSystem;
		A.block(ibegin, b1, nowrows, 7) = joint[ijoint]->A1;
		if (joint[ijoint]->get_type() != GROUND)
		{
			b2 = 7 * joint[ijoint]->body[1]->IDinSystem;
			A.block(ibegin, b2, nowrows, 7) = joint[ijoint]->A2;
		}
		b.segment(ibegin, nowrows) << joint[ijoint]->b;
		ibegin += nowrows;
	}
	for (ibody = 0; ibody < nBodies; ibody++)
	{
		for (j = 0; j < 4; j++)
		{
			A(ibegin, 3 + j + 7 * ibody) = 2 * body[ibody]->quat(j);
		}
		b(ibegin) = -2.0 * body[ibody]->quatd.dot(body[ibody]->quatd);
		ibegin++;
	}

#endif // SPARSE
}