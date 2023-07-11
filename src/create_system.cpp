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
#include "create_system.h"
#include "error.h"
#include "joint.h"
#include "body.h"
#include "MUSEsystem.h"
#include "input.h"
#include "enums.h"

using namespace MUSE_NS;

/* ---------------------------------------------------------------------- */

CreateSystem::CreateSystem(MUSE *muse) : Pointers(muse)
{
    id = -1;
}

/* ---------------------------------------------------------------------- */

void CreateSystem::command(int narg, char **arg)
{
	if(narg < 2) error->all(FLERR, "Illegal create_system command");
	int n = strlen(arg[0]);

	for (int i = 0; i < n; i++)
		if (!isalnum(arg[0][i]) && arg[0][i] != '_')
			error->all(FLERR, "System name must be alphanumeric or underscore characters");
	id = muse->add_System(arg[0]);

    int iarg = 1;

    while (iarg < narg) {
        if (strcmp(arg[iarg], "dt") == 0) {
            if( narg <= iarg + 1 ) error->all(FLERR, "Illegal create_system command");
            muse->system[id]->dt = input->numeric(FLERR, arg[iarg + 1]);
            if (muse->system[id]->dt <= 0) error->all(FLERR, "The time step must be a posotive value");
            iarg = iarg + 2;
        }
        else if (strcmp(arg[iarg], "gravity") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_system command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->system[id]->ga << px, py, pz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "bodys") == 0) {
            int count = 1;
            while (true)
            {
                if (narg <= iarg + count) error->all(FLERR, "Illegal create_system command");
                if (strcmp(arg[iarg + count], "/bodys") == 0) break;

                int ibody;

                for (ibody = 0; ibody < muse->nBodies; ibody++)
                    if (strcmp(arg[iarg + count], muse->body[ibody]->name) == 0) break;

                if (ibody < muse->nBodies) {
                    muse->system[id]->add_Body(muse->body[ibody]);
                }
                else {
                    char str[128];
                    sprintf(str, "Cannot find body with name: %s", arg[iarg + count]);
                    error->all(FLERR, str);
                }
                count++;
            }
            iarg = iarg + count + 1;
        }
        else if (strcmp(arg[iarg], "joints") == 0) {
            int count = 1;
            while (true)
            {
                if (narg <= iarg + count) error->all(FLERR, "Illegal create_system command");
                if (strcmp(arg[iarg + count], "/joints") == 0) break;

                int ijoint;

                for (ijoint = 0; ijoint < muse->nJoints; ijoint++)
                    if (strcmp(arg[iarg + count], muse->joint[ijoint]->name) == 0) break;

                if (ijoint < muse->nJoints) {
                    muse->system[id]->add_Joint(muse->joint[ijoint]);
                }
                else {
                    char str[128];
                    sprintf(str, "Cannot find joint with name: %s", arg[iarg + count]);
                    error->all(FLERR, str);
                }
                count++;
            }
            iarg = iarg + count + 1;
        }
        else error->all(FLERR, "Illegal create_joint command");
    }

}
