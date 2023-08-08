/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

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

#include "MUSEsystem.h"
#include "memory.h"
/////////////////
using namespace MUSE_NS;
using namespace std;

int main(int argc, char **argv){

	MPI_Init(&argc,&argv);


	
	MUSE *muse = new MUSE(argc,argv,MPI_COMM_WORLD);

	ofstream out;
	out.open("./res.txt", ios::trunc);
	out << "MUSE OUTPUT:" << endl;
	out.close();

	muse->input->file();



	delete muse;
	MPI_Finalize();
}