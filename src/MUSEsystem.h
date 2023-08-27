/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_SYSTEM_H
#define MUSE_SYSTEM_H

#include "pointers.h"
#include "Eigen/Eigen"
#include <vector>

namespace MUSE_NS {

class System : protected Pointers {
public:

#ifdef SPARSE
	Eigen::SparseMatrix<double> A;
	Eigen::SparseMatrix<double> M;
#else
	Eigen::MatrixXd A;
	Eigen::MatrixXd M;
#endif // SPARSE

	Eigen::VectorXd b;
	Eigen::VectorXd F;

	Eigen::VectorXd x;
	Eigen::VectorXd xd;
	Eigen::VectorXd xdd;

	Eigen::Vector3d ga;
	double timenow;
	double dt;

	int ntimestep;
	int nsteps;
	int firststep;
	int laststep;
	int beginstep;
	int endstep;
	int first_run;

	int nBodies;
	int nJoints;

	bool logflag;
	Eigen::VectorXd xlognow;
	std::vector<Eigen::VectorXd> xlog;

	class Body **body;
	class Joint **joint;

	System(class MUSE *);
	~System();

	void command(int, char**);
	int add_Body(Body *);
	int remove_Body(Body*);
	int add_Joint(Joint *);
	int remove_Joint(Joint*);

	void setup();
	void makeBigAb();
	void makeBigM();
	void makeBigF();
	void update_euler();
	void update_RK4();
	void calxdd();
	void x2body();
	void solve(int);



	


private:

	int maxBodies;
	int maxJoints;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
};

}

#endif