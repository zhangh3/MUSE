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
#include "create.h"
#include "error.h"
#include "joint.h"
#include "body.h"
#include "MUSEsystem.h"
#include "input.h"
#include "enums.h"

#include <iostream>

using namespace MUSE_NS;

/* ---------------------------------------------------------------------- */

Create::Create(MUSE *muse) : Pointers(muse)
{
    id = -1;
}

/* ---------------------------------------------------------------------- */

void Create::command(int narg, char** arg)
{
    //std::cout << "narg= " << narg << std::endl;
    //for (int i = 0;i < narg;i++) {
    //    std::cout << "arg" << i << "= " << arg[i] << std::endl;
    //}
    if (narg < 1) error->all(FLERR, "Illegal change command");
    if (strcmp(arg[0], "body") == 0) create_body(narg - 1, &arg[1]);
    else if (strcmp(arg[0], "joint") == 0)  create_joint(narg - 1, &arg[1]);
    else if (strcmp(arg[0], "system") == 0)  create_system(narg - 1, &arg[1]);
    else {
        char str[128];
        sprintf(str, "Illegal create type: %s", arg[1]);
        error->all(FLERR, str);
    }
}

void Create::create_body(int narg, char** arg)
{
    if (narg < 1) error->all(FLERR, "Illegal create body command");
    int n = strlen(arg[0]);

    for (int i = 0; i < n; i++)
        if (!isalnum(arg[0][i]) && arg[0][i] != '_')
            error->all(FLERR, "Body name must be alphanumeric or underscore characters");
    id = muse->add_Body(arg[0]);

    int iarg = 1;

    while (iarg < narg) {
        if (strcmp(arg[iarg], "mass") == 0) {
            if (narg <= iarg + 1) error->all(FLERR, "Illegal create body command");
            double mass = input->numeric(FLERR, arg[iarg + 1]);
            muse->body[id]->set_Mass(mass);
            iarg = iarg + 2;
        }
        else if (strcmp(arg[iarg], "pos") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create body command");
            double posx = input->numeric(FLERR, arg[iarg + 1]);
            double posy = input->numeric(FLERR, arg[iarg + 2]);
            double posz = input->numeric(FLERR, arg[iarg + 3]);
            muse->body[id]->pos << posx, posy, posz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "inertia") == 0) {
            if (narg <= iarg + 6) error->all(FLERR, "Illegal create body command");

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
            if (narg <= iarg + 4) error->all(FLERR, "Illegal create body command");

            double* q = new double[4];

            q[0] = input->numeric(FLERR, arg[iarg + 1]);
            q[1] = input->numeric(FLERR, arg[iarg + 2]);
            q[2] = input->numeric(FLERR, arg[iarg + 3]);
            q[3] = input->numeric(FLERR, arg[iarg + 4]);

            muse->body[id]->set_Quaternion(q);
            iarg = iarg + 5;
            delete[]q;
        }
        else if (strcmp(arg[iarg], "vel") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create body command");
            double vx = input->numeric(FLERR, arg[iarg + 1]);
            double vy = input->numeric(FLERR, arg[iarg + 2]);
            double vz = input->numeric(FLERR, arg[iarg + 3]);
            muse->body[id]->pos << vx, vy, vz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "omega") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create body command");
            double wx = input->numeric(FLERR, arg[iarg + 1]);
            double wy = input->numeric(FLERR, arg[iarg + 2]);
            double wz = input->numeric(FLERR, arg[iarg + 3]);
            muse->body[id]->set_Omega(wx, wy, wz);
            iarg = iarg + 4;
        }
        else error->all(FLERR, "Illegal create body command");
    }
}

void Create::create_joint(int narg, char** arg)
{
    if (narg < 2) error->all(FLERR, "Illegal create joint command");
    int n = strlen(arg[0]);

    for (int i = 0; i < n; i++)
        if (!isalnum(arg[0][i]) && arg[0][i] != '_')
            error->all(FLERR, "Joint name must be alphanumeric or underscore characters");
    id = muse->add_Joint(arg[0]);
    muse->joint[id]->set_type_by_name(arg[1]);

    int iarg = 2;

    while (iarg < narg) {
        if (strcmp(arg[iarg], "body1") == 0) {
            if (narg <= iarg + 1) error->all(FLERR, "Illegal create joint command");
            int ibody;
            for (ibody = 0; ibody < muse->nBodies; ibody++)
                if (strcmp(arg[iarg + 1], muse->body[ibody]->name) == 0) break;
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
            if (narg <= iarg + 1) error->all(FLERR, "Illegal create joint command");
            int ibody;
            for (ibody = 0; ibody < muse->nBodies; ibody++)
                if (strcmp(arg[iarg + 1], muse->body[ibody]->name) == 0) break;
            if (ibody < muse->nBodies) {
                muse->joint[id]->body[1] = muse->body[ibody];
            }
            else {
                char str[128];
                sprintf(str, "Cannot find body with name: %s", arg[iarg + 1]);
                error->all(FLERR, str);
            }
            iarg = iarg + 2;
        }
        else if (strcmp(arg[iarg], "point1") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->point1 << px, py, pz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "point2") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->point2 << px, py, pz;
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "axis1") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->set_axis(px, py, pz, 1);
            iarg = iarg + 4;
        }
        else if (strcmp(arg[iarg], "axis2") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create joint command");
            double px = input->numeric(FLERR, arg[iarg + 1]);
            double py = input->numeric(FLERR, arg[iarg + 2]);
            double pz = input->numeric(FLERR, arg[iarg + 3]);
            muse->joint[id]->set_axis(px, py, pz, 2);
            iarg = iarg + 4;
        }
        else error->all(FLERR, "Illegal create joint command");
    }

}

void Create::create_system(int narg, char** arg)
{
    if (narg < 1) error->all(FLERR, "Illegal create system command");
    int n = strlen(arg[0]);

    for (int i = 0; i < n; i++)
        if (!isalnum(arg[0][i]) && arg[0][i] != '_')
            error->all(FLERR, "System name must be alphanumeric or underscore characters");
    id = muse->add_System(arg[0]);

    int iarg = 1;

    while (iarg < narg) {
        if (strcmp(arg[iarg], "dt") == 0) {
            if (narg <= iarg + 1) error->all(FLERR, "Illegal create system command");
            muse->system[id]->dt = input->numeric(FLERR, arg[iarg + 1]);
            if (muse->system[id]->dt <= 0) error->all(FLERR, "The time step must be a posotive value");
            iarg = iarg + 2;
        }
        else if (strcmp(arg[iarg], "gravity") == 0) {
            if (narg <= iarg + 3) error->all(FLERR, "Illegal create system command");
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
                if (narg <= iarg + count) error->all(FLERR, "Illegal create system command");
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
                if (narg <= iarg + count) error->all(FLERR, "Illegal create system command");
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
        else error->all(FLERR, "Illegal create system command");
    }

}

