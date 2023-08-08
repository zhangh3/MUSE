/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */


#ifndef MUSE_MUSE_H
#define MUSE_MUSE_H
#include "stdio.h"
namespace MUSE_NS{
    class MUSE {
	public:
		// fundamental MUSE classes

		class Memory *memory;          // list of ptrs to the body objects
		class Error *error;            // error handling
		class Ensemble *ensemble;      // ensemble of simulation
		class Input* input;            // input script processing
		class Timer* timer;            // CPU timing info


		// ptrs to top-level MUSE-specific classes
		class System **system;         // list of ptrs to multibody systems
		class Body **body;             // list of ptrs to the bodies 
		class Joint **joint;           // list of ptrs to the joints 

		
		MPI_Comm world;
		FILE *screen;
		FILE *logfile;
		FILE *infile;
		int me;

		
		int nBodies;
		int nJoints;
		int nSystems;





		MUSE(int, char **, MPI_Comm);
		~MUSE();

		int add_Body(char *);
		int add_Joint(char*);
		int add_System(char*);

	private:

		int maxBodies;
		int maxJoints;
		int maxSystems;

	};

}
#endif