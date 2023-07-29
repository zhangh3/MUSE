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
#include "change.h"
#include "error.h"
#include "joint.h"
#include "body.h"
#include "MUSEsystem.h"
#include "input.h"
#include "enums.h"

#include <iostream>

using namespace MUSE_NS;

/* ---------------------------------------------------------------------- */

Change::Change(MUSE *muse) : Pointers(muse)
{
    id = -1;
}

/* ---------------------------------------------------------------------- */

void Change::command(int narg, char** arg)
{
    //std::cout << "narg= " << narg << std::endl;
    //for (int i = 0;i < narg;i++) {
    //    std::cout << "arg" << i << "= " << arg[i] << std::endl;
    //}
    if (narg < 1) error->all(FLERR, "Illegal change command");
    if (strcmp(arg[0], "body") == 0) change_body(narg - 1, &arg[1]);
    else if (strcmp(arg[0], "joint") == 0)  change_joint(narg - 1, &arg[1]);
    else if (strcmp(arg[0], "system") == 0)  change_system(narg - 1, &arg[1]);
    else {
        char str[128];
        sprintf(str, "Illegal change type: %s", arg[1]);
        error->all(FLERR, str);
    }
}

void Change::change_body(int narg, char** arg)
{
    std::cout << "bodynarg= " << narg << std::endl;
    for (int i = 0;i < narg;i++) {
        std::cout << "arg" << i << "= " << arg[i] << std::endl;
    }
}

void Change::change_joint(int narg, char** arg)
{
    std::cout << "jointnarg= " << narg << std::endl;
    for (int i = 0;i < narg;i++) {
        std::cout << "arg" << i << "= " << arg[i] << std::endl;
    }
}

void Change::change_system(int narg, char** arg)
{
    std::cout << "systemnarg= " << narg << std::endl;
    for (int i = 0;i < narg;i++) {
        std::cout << "arg" << i << "= " << arg[i] << std::endl;
    }
}


