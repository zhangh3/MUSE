/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */


#ifndef MUSE_OUTPUT_H
#define MUSE_OUTPUT_H

#include "pointers.h"

namespace MUSE_NS {

	class Output : protected Pointers {
	public:
		int next;                    // next timestep for any kind of output

		int next_stats;              // next timestep for stats output
		int stats_every;             // stats output every this many steps
		int last_stats;              // last timestep stats was output
		char* var_stats;             // variable name for stats frequency
		int ivar_stats;              // variable index for stats frequency
		class Stats* stats;          // statistical output

		int nresult;                     // # of result defined
		int max_result;                  // max size of result list
		int next_result_any;             // next timestep for any result
		int* every_result;               // output of each result every this many steps
		int* next_result;                // next timestep to do each result
		int* last_result;                // last timestep each snapshot was output
		char** var_result;               // variable name for result frequency
		int* ivar_result;                // variable index for result frequency
		class Result** result;           // list of defined results

		int restart_flag;             // 1 if restart files are written
		int restart_every;            // restart file write freq, 0 if var
		int next_restart;             // next timestep to write restart file
		int last_restart;             // last timestep restart file was output
		char* var_restart;            // variable name for restart freq
		int ivar_restart;             // index of var_restart_double
		char* restartf;               // name restart file
		class WriteRestart* restart;  // class for writing restart files

		Output(class MUSE*);
		~Output();
		void init();
		void setup(int);                   // initial output before run/min
		void write(int);                   // output for current timestep
		void write_result(int);            // force output of result snapshots
		void write_restart(int);           // force output of a restart file
		void reset_timestep(int);          // reset next timestep for all output

		void add_result(int, char**);       // add a result to result list
		void modify_result(int, char**);    // modify a result
		void delete_result(char*);          // delete a result from result list

		void set_stats(int, char**);       // set stats output frequency
		void create_stats(int, char**);    // create a Stats style
		void create_restart(int, char**);  // create Restart and restart files
	};

}

#endif

/* ERROR/WARNING messages:

*/
