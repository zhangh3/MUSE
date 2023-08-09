/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "string.h"
#include "compute_body.h"
#include "MUSEsystem.h"
#include "Body.h"
#include "modify.h"
#include "memory.h"
#include "error.h"

using namespace MUSE_NS;

// user keywords

enum{POS, VEL, QUAT, EULER, OMG, MASS,
    INTERIAL, ENER, MOME, ANGMOME, NMOME, NANGMOME};

// max # of quantities to accumulate for any user value
#define MAXACCUMULATE 2

/* ---------------------------------------------------------------------- */

ComputeBody::ComputeBody(MUSE *muse, int narg, char **arg) :
  Compute(muse, narg, arg)
{
  if (narg < 5) error->all(FLERR,"Illegal compute body command");

  for (bodyid = 0; bodyid < muse->system[sysid]->nBodies; bodyid++)
      if (strcmp(arg[3], muse->system[sysid]->body[bodyid]->name) == 0) {
          break;
      }
  if (bodyid == muse->system[sysid]->nBodies) {
      char str[128];
      sprintf(str, "Cannot find body %s in system: %s", arg[4], muse->system[sysid]->name);
      error->all(FLERR, str);
  }
  else
  {
      bodyid = muse->system[sysid]->body[bodyid]->IDinMuse; //get id in muse
  }

  nvalue = narg - 4;
  value = new int[nvalue];

  nmap = new int[nvalue];
  memory->create(map,nvalue,MAXACCUMULATE,"body:map");
  for (int i = 0; i < nvalue; i++) nmap[i] = 0;

  int ivalue = 0;
  int iarg = 4;
  vcount = 0;
  while (iarg < narg) {
    if (strcmp(arg[iarg],"pos") == 0) {
      value[ivalue] = POS;
      nmap[ivalue] = 3;
      vcount += nmap[ivalue];
    } else if (strcmp(arg[iarg],"vel") == 0) {
      value[ivalue] = VEL;
      nmap[ivalue] = 3;
      vcount += nmap[ivalue];
    } else if (strcmp(arg[iarg],"quat") == 0) {
      value[ivalue] = QUAT;
      nmap[ivalue] = 4;
      vcount += nmap[ivalue];
    } else if (strcmp(arg[iarg],"omega") == 0) {
      value[ivalue] = OMG;
      nmap[ivalue] = 3;
      vcount += nmap[ivalue];
    } else error->all(FLERR,"Illegal compute body command");
    ivalue++;
    iarg++;
  }
}

/* ---------------------------------------------------------------------- */

ComputeBody::~ComputeBody()
{
  if (copymode) return;

  delete [] value;
  delete [] unique;
  delete [] nmap;
  memory->destroy(map);
  memory->destroy(tally);
}

/* ---------------------------------------------------------------------- */

void ComputeBody::init()
{
  reallocate();
}



void ComputeBody::reallocate()
{
}


