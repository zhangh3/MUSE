/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "mpi.h"
#include "muse.h"
#include "input.h"
#include <iostream>
#include <fstream>
//////////////////
#include "body.h"
#include "joint.h"
#include "joint_enums.h"
#include "MUSEsystem.h"
#include "math_extra.h"
#include "memory.h"
#include "Eigen/Eigen"
#include "Eigen/SparseQR"

#include<ctime>
#include <iomanip>
/////////////////
using namespace MUSE_NS;
using namespace Eigen;
using namespace std;

int main(int argc, char **argv){

	MPI_Init(&argc,&argv);
	
	MUSE *muse = new MUSE(argc,argv,MPI_COMM_WORLD);

	char name1[] = "good1";
	char name2[] = "good2";
	char name3[] = "good3";
	char name4[] = "good4";
	char name5[] = "good5";

	muse->add_Body(name1);
	muse->add_Body(name2);
	muse->add_Body(name3);

	muse->add_Joint(name1);
	muse->add_Joint(name2);
	muse->add_Joint(name3);


	muse->body[0]->pos << 0, 0, 0;
	muse->body[1]->pos << 1, 0, 0;
	muse->body[2]->pos << 2, 0, 0;
	muse->body[0]->quat << 0, 0, 0, 1;
	muse->body[1]->quat << 0, 0, 0, 1;
	muse->body[2]->quat << 0, 0, 0, 1;


	muse->joint[0]->body[0] = muse->body[0];
	muse->joint[0]->body[1] = muse->body[1];
	muse->joint[0]->set_type(SPHERE);
	muse->joint[0]->point1 << 0.5,0,0;
	muse->joint[0]->point2 << -0.5,0,0;


	muse->joint[1]->body[0] = muse->body[1];
	muse->joint[1]->body[1] = muse->body[2];
	muse->joint[1]->set_type(SPHERE);
	muse->joint[1]->point1 << 0.5, 0, 0;
	muse->joint[1]->point2 << -0.5, 0, 0;

	//muse->joint[1]->set_type(HINGE);  //铰链示例
	//muse->joint[1]->point1 << 0.5, 0, 0;
	//muse->joint[1]->point2 << -0.5, 0, 0;
	//muse->joint[1]->set_axis(0, 0, 1, 1);



	muse->joint[2]->body[0] = muse->body[0];
	muse->joint[2]->set_type(GROUND);


	muse->system->add_Body(muse->body[0]);
	muse->system->add_Body(muse->body[1]);
	muse->system->add_Body(muse->body[2]);

	muse->system->add_Joint(muse->joint[0]);
	muse->system->add_Joint(muse->joint[1]);
	muse->system->add_Joint(muse->joint[2]);


	muse->system->setup();
	muse->system->dt = 1E-3;

	clock_t start, end;
	start = clock();
	muse->system->solve(30);
	end = clock();

	cout << "run time: " << fixed << setprecision(2) << 1000 * (double)(end - start) / CLOCKS_PER_SEC << "ms" << endl;

	ofstream out;
	out.open("./resat.txt",  ios::out);
	for (int i = 0; i < muse->system->xlog.size();i++)
	{
		out << muse->system->xlog[i].transpose() << endl;
	}
	delete muse;
	MPI_Finalize();
}