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
#include "create_joint.h"
#include "error.h"
#include "joint.h"
#include "body.h"
#include "input.h"
#include "enums.h"

using namespace MUSE_NS;

/* ---------------------------------------------------------------------- */

CreateJoint::CreateJoint(MUSE *muse) : Pointers(muse)
{
    id = -1;
}

/* ---------------------------------------------------------------------- */

void CreateJoint::command(int narg, char **arg)
{
	if(narg < 2) error->all(FLERR, "Illegal create_body command");
	int n = strlen(arg[0]);

	for (int i = 0; i < n; i++)
		if (!isalnum(arg[0][i]) && arg[0][i] != '_')
			error->all(FLERR, "Joint name must be alphanumeric or underscore characters");
	id = muse->add_Joint(arg[0]);

    if (strcmp(arg[1], "sphere") == 0) muse->joint[id]->set_type(SPHERE);
    else if (strcmp(arg[1], "ground") == 0)  muse->joint[id]->set_type(GROUND);
    else {
        char str[128];
        sprintf(str, "Illegal joint type: %s", arg[1]);
        error->all(FLERR, str);
    }
    int iarg = 2;

    while (iarg < narg) {
        if (strcmp(arg[iarg], "body1") == 0) {
            if( narg <= iarg + 1 ) error->all(FLERR, "Illegal create_joint command");
            int ibody;
            for (ibody = 0; ibody < muse->nBodies; ibody++)
                if (strcmp(arg[iarg+1], muse->body[ibody]->name) == 0) break;
            if (ibody < muse->nBodies) {
                muse->joint[id]->body[0] = muse->body[ibody];
            }
            else {
                char str[128];
                sprintf(str, "Cannot body with name: %s", arg[iarg + 1]);
                error->all(FLERR, str);
            }
            iarg = iarg + 2;
        }
        else if (strcmp(arg[iarg], "body2") == 0) {
            if (narg <= iarg + 1) error->all(FLERR, "Illegal create_joint command");
            int ibody;
            for (ibody = 0; ibody < muse->nBodies; ibody++)
                if (strcmp(arg[iarg + 1], muse->body[ibody]->name) == 0) break;
            if (ibody < muse->nBodies) {
                muse->joint[id]->body[1] = muse->body[ibody];
            }
            else {
                char str[128];
                sprintf(str, "Cannot body with name: %s", arg[iarg + 1]);
                error->all(FLERR, str);
            }
            iarg = iarg + 2;
        }
        else if (strcmp(arg[iarg], "point1") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->point1 << px, py, pz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "point2") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->point2 << px, py, pz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "axis1") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->set_axis(px, py, pz, 1);
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "axis2") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->set_axis(px, py, pz, 2);
            iarg = iarg + 4;
        }
        else error->all(FLERR, "Illegal create_joint command");
    }

}
