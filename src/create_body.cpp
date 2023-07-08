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
#include "body.h"
#include "input.h"

using namespace MUSE_NS;

/* ---------------------------------------------------------------------- */

CreateBody::CreateBody(MUSE *muse) : Pointers(muse) 
{
    id = -1;
}

/* ---------------------------------------------------------------------- */

void CreateBody::command(int narg, char **arg)
{
	if(narg < 1) error->all(FLERR, "Illegal create_body command");
	int n = strlen(arg[0]);

	for (int i = 0; i < n; i++)
		if (!isalnum(arg[0][i]) && arg[0][i] != '_')
			error->all(FLERR, "Body name must be alphanumeric or underscore characters");
	id = muse->add_Body(arg[0]);

    int iarg = 1;

    while (iarg < narg) {
        if (strcmp(arg[iarg], "mass") == 0) {
            if( narg <= iarg + 1 ) error->all(FLERR, "Illegal create_grid command");
            double mass = input->numeric(FLERR, arg[iarg + 1]);
            muse->body[id]->set_Mass(mass);
            iarg = iarg + 2;
        }
        else if (strcmp(arg[iarg], "pos") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_grid command");
            double posx = input->numeric(FLERR, arg[iarg + 1]);
            double posy = input->numeric(FLERR, arg[iarg + 2]);
            double posz = input->numeric(FLERR, arg[iarg + 3]);
            muse->body[id]->pos << posx, posy, posz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "inertia") == 0) {
            if (narg <= iarg + 6) error->all(FLERR, "Illegal create_grid command");
            
            double Ixx = input->numeric(FLERR, arg[iarg + 1]);
            double Iyy = input->numeric(FLERR, arg[iarg + 2]);
            double Izz = input->numeric(FLERR, arg[iarg + 3]);
            double Ixy = input->numeric(FLERR, arg[iarg + 4]);
            double Ixz = input->numeric(FLERR, arg[iarg + 5]);
            double Iyz = input->numeric(FLERR, arg[iarg + 6]);
       
            muse->body[id]->set_Inertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
            iarg = iarg + 7;
        }
        else if (strcmp(arg[iarg], "quat") == 0) {
            if (narg <= iarg + 4) error->all(FLERR, "Illegal create_grid command");
            
            double* q = new double[4];

            q[0] = input->numeric(FLERR, arg[iarg + 1]);
            q[1] = input->numeric(FLERR, arg[iarg + 2]);
            q[2] = input->numeric(FLERR, arg[iarg + 3]);
            q[3] = input->numeric(FLERR, arg[iarg + 4]);

            muse->body[id]->set_Quaternion(q);
            iarg = iarg + 5;
            delete []q;
        }
        else if (strcmp(arg[iarg], "vel") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_grid command");
            double vx = input->numeric(FLERR, arg[iarg + 1]);
            double vy = input->numeric(FLERR, arg[iarg + 2]);
            double vz = input->numeric(FLERR, arg[iarg + 3]);
            muse->body[id]->pos << vx, vy, vz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "omega") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create_grid command");
            double wx = input->numeric(FLERR, arg[iarg + 1]);
            double wy = input->numeric(FLERR, arg[iarg + 2]);
            double wz = input->numeric(FLERR, arg[iarg + 3]);
            muse->body[id]->set_Omega(wx, wy, wz);
            iarg = iarg + 4;
        }
        else error->all(FLERR, "Illegal create_grid command");
    }

}
