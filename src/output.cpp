/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "output.h"
#include "input.h"
#include "variable.h"
#include "memory.h"
#include "error.h"
#include "muse.h"

using namespace MUSE_NS;

#define DELTA 1

/* ---------------------------------------------------------------------- */

Output::Output(MUSE* muse) : Pointers(muse)
{
    //stats = new Stats(muse);

    stats_every = 0;
    var_stats = NULL;

    nresult = 0;
    max_result = 0;
    every_result = NULL;
    next_result = NULL;
    last_result = NULL;
    var_result = NULL;
    ivar_result = NULL;
    result = NULL;

    restart_flag = 0;
    restart_every = 0;
    last_restart = -1;
    restartf = NULL;
    var_restart = NULL;
    restart = NULL;
}

/* ---------------------------------------------------------------------- */

Output::~Output()
{
    //if (stats) delete stats;
    delete[] var_stats;

    memory->destroy(every_result);
    memory->destroy(next_result);
    memory->destroy(last_result);
    for (int i = 0; i < nresult; i++) delete[] var_result[i];
    memory->sfree(var_result);
    memory->destroy(ivar_result);
    for (int i = 0; i < nresult; i++) delete result[i];
    memory->sfree(result);

    delete[] restartf;
    delete[] var_restart;
    delete restart;
}
