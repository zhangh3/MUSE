/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
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
