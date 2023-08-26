/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "stdio.h"
#include "string.h"
#include "modify.h"

#include "compute.h"
#include "style_compute.h"
#include "memory.h"
#include "error.h"
#include "MUSEsystem.h"

using namespace MUSE_NS;

#define DELTA 4

// mask settings - same as in fix.cpp

#define START_OF_STEP  1
#define END_OF_STEP    2

/* ---------------------------------------------------------------------- */

Modify::Modify(MUSE* muse) : Pointers(muse)
{
	n_start_of_step = n_end_of_step = 0;

	list_start_of_step = list_end_of_step = NULL;

	end_of_step_every = NULL;
	list_timeflag = NULL;

	ncompute = maxcompute = 0;
	compute = NULL;

	// n_pergrid needs to be initialized here because ReadSurf calls
	//  Modify::reset_grid_count without calling Modify::init

	n_pergrid = 0;
}

/* ---------------------------------------------------------------------- */

Modify::~Modify()
{
	// delete all computes

	for (int i = 0; i < ncompute; i++) delete compute[i];
	memory->sfree(compute);

	delete[] list_start_of_step;
	delete[] list_end_of_step;

	delete[] end_of_step_every;
	delete[] list_timeflag;
}

/* ----------------------------------------------------------------------
   initialize all and computes
------------------------------------------------------------------------- */

void Modify::init()
{
	int i;

	// create other lists of computes

	list_init_computes();

	// init each compute
	// set invoked_scalar,vector,etc to -1 to force new run to re-compute them
	// add initial timestep to all computes that store invocation times
	//   since any of them may be invoked by initial thermo
	// do not clear out invocation times stored within a compute,
	//   b/c some may be holdovers from previous run, like for ave fixes

	for (i = 0; i < ncompute; i++) {
		compute[i]->init();
		compute[i]->invoked_scalar = -1;
		compute[i]->invoked_vector = -1;
		compute[i]->invoked_array = -1;
	}
	addstep_compute_all();
}

void Modify::setup()
{
	//FIXME:有fix添加
}

/* ----------------------------------------------------------------------
   start-of-timestep call, only for relevant fixes
------------------------------------------------------------------------- */

void Modify::start_of_step()
{
	//FIXME:有fix添加
}

/* ----------------------------------------------------------------------
   end-of-timestep call, only for relevant fixes
   only call fix->end_of_step() on timesteps that are multiples of nevery
------------------------------------------------------------------------- */

void Modify::end_of_step()
{
	//FIXME:有fix添加
}


/* ----------------------------------------------------------------------
   add a new compute
------------------------------------------------------------------------- */
void Modify::add_compute(int narg, char** arg)
{
	if (narg < 2) error->all(FLERR, "Illegal compute command");

	// error check

	for (int icompute = 0; icompute < ncompute; icompute++)
		if (strcmp(arg[0], compute[icompute]->name) == 0)
			error->all(FLERR, "Reuse of compute name");

	// extend Compute list if necessary

	if (ncompute == maxcompute) {
		maxcompute += DELTA;
		compute = (Compute**)
			memory->srealloc(compute, maxcompute * sizeof(Compute*), "modify:compute");
	}

	// create the Compute

	if (0) return;

#define COMPUTE_CLASS
#define ComputeStyle(key,Class) \
else if (strcmp(arg[2],#key) == 0) \
    compute[ncompute] = new Class(muse,narg,arg);
#include "style_compute.h"
#undef ComputeStyle
#undef COMPUTE_CLASS

	else error->all(FLERR, "Unrecognized compute style");

	ncompute++;
}

/* ----------------------------------------------------------------------
   delete a Compute from list of Computes
------------------------------------------------------------------------- */

void Modify::delete_compute(const char* id)
{
	int icompute = find_compute(id);
	if (icompute < 0) error->all(FLERR, "Could not find compute ID to delete");
	delete compute[icompute];

	// move other Computes down in list one slot

	for (int i = icompute + 1; i < ncompute; i++) compute[i - 1] = compute[i];
	ncompute--;
}

/* ----------------------------------------------------------------------
   find a compute by ID
   return index of compute or -1 if not found
------------------------------------------------------------------------- */

int Modify::find_compute(const char* id)
{
	int icompute;
	for (icompute = 0; icompute < ncompute; icompute++)
		if (strcmp(id, compute[icompute]->name) == 0) break;
	if (icompute == ncompute) return -1;
	return icompute;
}

/* ----------------------------------------------------------------------
   clear invoked flag of all computes
   called everywhere that computes are used, before computes are invoked
   invoked flag used to avoid re-invoking same compute multiple times
   and to flag computes that store invocation times as having been invoked
------------------------------------------------------------------------- */

void Modify::clearstep_compute()
{
	for (int icompute = 0; icompute < ncompute; icompute++)
		compute[icompute]->invoked_flag = 0;
}

/* ----------------------------------------------------------------------
   loop over computes that store invocation times
   if its invoked flag set on this timestep, schedule next invocation
   called everywhere that computes are used, after computes are invoked
------------------------------------------------------------------------- */

void Modify::addstep_compute(int newstep)
{
	for (int icompute = 0; icompute < n_timeflag; icompute++)
		if (compute[list_timeflag[icompute]]->invoked_flag)
			compute[list_timeflag[icompute]]->addstep(newstep);
}

/* ----------------------------------------------------------------------
   loop over all computes
   schedule next invocation for those that store invocation times
   called when not sure what computes will be needed on newstep
   do not loop only over n_timeflag, since may not be set yet
------------------------------------------------------------------------- */

void Modify::addstep_compute_all()
{
	int newstep;
	for (int icompute = 0; icompute < ncompute; icompute++) {
		newstep = muse->system->ntimestep;

		if (compute[icompute]->timeflag) compute[icompute]->addstep(newstep);
	}
}

/* ----------------------------------------------------------------------
   create list of indices for computes which various attributes
------------------------------------------------------------------------- */

void Modify::list_init_computes()
{
	delete[] list_timeflag;

	n_timeflag = 0;
	for (int i = 0; i < ncompute; i++)
		if (compute[i]->timeflag) n_timeflag++;
	list_timeflag = new int[n_timeflag];

	n_timeflag = 0;
	for (int i = 0; i < ncompute; i++)
		if (compute[i]->timeflag) list_timeflag[n_timeflag++] = i;
}