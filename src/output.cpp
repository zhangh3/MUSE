/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

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
#include "stats.h"
#include "result.h"
#include "variable.h"
#include "memory.h"
#include "error.h"
#include "muse.h"
#include "MUSEsystem.h"
#include "modify.h"


#include <iostream>
using namespace MUSE_NS;

#define DELTA 1

/* ---------------------------------------------------------------------- */

Output::Output(MUSE* muse) : Pointers(muse)
{
    stats = new Stats(muse);

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

    //restart_flag = 0;
    //restart_every = 0;
    //last_restart = -1;
    //restartf = NULL;
    //var_restart = NULL;
    //restart = NULL;
}

/* ---------------------------------------------------------------------- */

Output::~Output()
{
    if (stats) delete stats;
    delete[] var_stats;

    //memory->destroy(every_result);
    //memory->destroy(next_result);
    //memory->destroy(last_result);
    //for (int i = 0; i < nresult; i++) delete[] var_result[i];
    //memory->sfree(var_result);
    //memory->destroy(ivar_result);
    //for (int i = 0; i < nresult; i++) delete result[i];
    //memory->sfree(result);

    //delete[] restartf;
    //delete[] var_restart;
    //delete restart;
}
void Output::init()
{
    stats->init();
    if (var_stats) {
        ivar_stats = input->variable->find(var_stats);
        if (ivar_stats < 0)
            error->all(FLERR, "Variable name for stats every does not exist");
        if (!input->variable->equal_style(ivar_stats))
            error->all(FLERR, "Variable for stats every is invalid style");
    }
    //FIXME:����ȱ�����������ĳ�ʼ��
    
}

/* ----------------------------------------------------------------------
   perform output for setup of run/min
   do dump first, so memory_usage will include dump allocation
   do stats last, so will print after memory_usage
   memflag = 0/1 for printing out memory usage
------------------------------------------------------------------------- */

void Output::setup(int memflag)
{
    int ntimestep = muse->system->ntimestep;

    //FIXME:����ȱ������������setup
    int writeflag;

   

    // print memory usage unless being called between multiple runs

    // set next_stats to multiple of every or variable eval if var defined
    // insure stats output on last step of run
    // stats may invoke computes so wrap with clear/add

    modify->clearstep_compute();

    stats->header();
    stats->compute(0);
    last_stats = ntimestep;

    if (var_stats) {
        next_stats = static_cast<int>
            (input->variable->compute_equal(ivar_stats));
        if (next_stats <= ntimestep)
            error->all(FLERR, "Stats every variable returned a bad timestep");
    }
    else if (stats_every) {
        next_stats = (ntimestep / stats_every) * stats_every + stats_every;
        next_stats = MIN(next_stats, muse->system->laststep);
    }
    else next_stats = muse->system->laststep;

    modify->addstep_compute(next_stats);

    // next = next timestep any output will be done
    next = next_stats;
    //next = MIN(next_dump_any, next_restart);
    //next = MIN(next, next_stats);
}

/* ----------------------------------------------------------------------
   perform all output for this timestep
   only perform output if next matches current step and last output doesn't
   do dump/restart before stats so stats CPU time will include them
------------------------------------------------------------------------- */

void Output::write(int ntimestep)
{
    //FIXME:����ȱ�����������
    // insure next_thermo forces output on last step of run
    // thermo may invoke computes so wrap with clear/add

    if (next_stats == ntimestep) {
        modify->clearstep_compute();
        if (last_stats != ntimestep) stats->compute(1);
        last_stats = ntimestep;
        if (var_stats) {
            next_stats = static_cast<int>
                (input->variable->compute_equal(ivar_stats));
            if (next_stats <= ntimestep)
                error->all(FLERR, "Stats every variable returned a bad timestep");
        }
        else if (stats_every) next_stats += stats_every;
        else next_stats = muse->system->laststep;
        next_stats = MIN(next_stats, muse->system->laststep);
        modify->addstep_compute(next_stats);
    }

    // next = next timestep any output will be done
    //FIXME:����ȱ�����������
    next = next_stats;
}

/* ----------------------------------------------------------------------
   timestep is being changed, called by update->reset_timestep()
   reset next timestep values for dumps, restart, thermo output
   reset to smallest value >= new timestep
   if next timestep set by variable evaluation,
     eval for ntimestep-1, so current ntimestep can be returned if needed
     no guarantee that variable can be evaluated for ntimestep-1
       if it depends on computes, but live with that rare case for now
------------------------------------------------------------------------- */

void Output::reset_timestep(int ntimestep)
{
    //FIXME:����ȱ�����������

    if (var_stats) {
        modify->clearstep_compute();
        muse->system->ntimestep--;
        next_stats = static_cast<int>
            (input->variable->compute_equal(ivar_stats));
        if (next_stats < ntimestep)
            error->all(FLERR, "Stats_modify every variable returned a bad timestep");
        muse->system->ntimestep++;
        next_stats = MIN(next_stats, muse->system->laststep);
        modify->addstep_compute(next_stats);
    }
    else if (stats_every) {
        next_stats = (ntimestep / stats_every) * stats_every;
        if (next_stats < ntimestep) next_stats += stats_every;
        next_stats = MIN(next_stats, muse->system->laststep);
    }
    else next_stats = muse->system->laststep;

    //FIXME:����ȱ�����������
    next = next_stats;
}

void MUSE_NS::Output::add_result(int narg, char** arg)
{
    if (narg < 4) error->all(FLERR, "Illegal result command");

    // error checks
    if (atoi(arg[1]) <= 0) error->all(FLERR, "Invalid result frequency");

    for (int iresult = 0; iresult < nresult; iresult++)
        if (strcmp(arg[2], result[iresult]->filename) == 0) {
            char str[128];
            sprintf(str, "Reuse of result file: %s", arg[2]);
            error->all(FLERR, str);
        }
    // extend Result list if necessary
    if (nresult == max_result) {
        max_result += DELTA;
        result = (Result**)
            memory->srealloc(result, max_result * sizeof(Result*), "output:result");
        memory->grow(every_result, max_result, "output:every_result");
        memory->grow(next_result, max_result, "output:next_result");
        memory->grow(last_result, max_result, "output:last_result");
        var_result = (char**)
            memory->srealloc(var_result, max_result * sizeof(char*), "output:var_result");
        memory->grow(ivar_result, max_result, "output:ivar_result");
    }

    // create the Result

    if (0) return;         // dummy line to enable else-if macro expansion

#define RESULT_CLASS
#define ResultStyle(key,Class) \
  else if (strcmp(arg[0],#key) == 0) result[nresult] = new Class(muse,narg,arg);
#include "style_result.h"
#undef RESULT_CLASS

    else error->all(FLERR, "Unrecognized result style");

    every_result[nresult] = atoi(arg[1]);
    last_result[nresult] = -1;
    var_result[nresult] = NULL;
    nresult++;
}

/* ----------------------------------------------------------------------
   set stats output frequency from input script
------------------------------------------------------------------------- */

void Output::set_stats(int narg, char** arg)
{
    if (narg != 1) error->all(FLERR, "Illegal stats command");

    if (strstr(arg[0], "v_") == arg[0]) {
        delete[] var_stats;
        int n = strlen(&arg[0][2]) + 1;
        var_stats = new char[n];
        strcpy(var_stats, &arg[0][2]);
    }
    else {
        stats_every = atoi(arg[0]);
        if (stats_every < 0) error->all(FLERR, "Illegal stats command");
    }
}

/* ----------------------------------------------------------------------
   new Stats style
------------------------------------------------------------------------- */

void Output::create_stats(int narg, char** arg)
{
    if (narg < 1) error->all(FLERR, "Illegal stats_style command");
    stats->set_fields(narg, arg);
}
