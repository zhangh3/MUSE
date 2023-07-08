/* ----------------------------------------------------------------------
   MUSE - Stochastic PArallel Rarefied-gas Time-accurate Analyzer
   http://muse.sandia.gov
   Steve Plimpton, sjplimp@sandia.gov, Michael Gallis, magalli@sandia.gov
   Sandia National Laboratories

   Copyright (2014) Sandia Corporation.  Under the terms of Contract
   DE-AC04-94AL85000 with Sandia Corporation, the U.S. Government retains
   certain rights in this software.  This software is distributed under
   the GNU General Public License.

   See the README file in the top-level MUSE directory.
------------------------------------------------------------------------- */

#include "ctype.h"
#include "stdlib.h"
#include "string.h"
#include "create_body.h"
#include "error.h"

using namespace MUSE_NS;

/* ---------------------------------------------------------------------- */

CreateBody::CreateBody(MUSE *muse) : Pointers(muse) {}

/* ---------------------------------------------------------------------- */

void CreateBody::command(int narg, char **arg)
{
	if(narg < 1) error->all(FLERR, "Illegal create_body command");
	int n = strlen(arg[0]);

	for (int i = 0; i < n; i++)
		if (!isalnum(arg[0][i]) && arg[0][i] != '_')
			error->all(FLERR, "Body name must be alphanumeric or underscore characters");
	muse->add_Body(arg[0]);
}
