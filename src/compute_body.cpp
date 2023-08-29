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
#include "body.h"
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


  if (narg < 4) error->all(FLERR,"Illegal compute body command");

  for (bodyid = 0; bodyid < muse->system->nBodies; bodyid++)
      if (strcmp(arg[2], muse->system->body[bodyid]->name) == 0) {
          break;
      }
  if (bodyid == muse->system->nBodies) {
      char str[128];
      sprintf(str, "Cannot find body %s in system", arg[4]);
      error->all(FLERR, str);
  }
  else
  {
      bodyid = muse->system->body[bodyid]->IDinMuse; //get id in muse
  }

  nvalue = narg - 3;
  value = new int[nvalue];
  nlen = new int[nvalue];
  nstart = new int[nvalue];

  int ivalue = 0;
  int iarg = 3;
  ncount = 0;
  while (iarg < narg) {
    if (strcmp(arg[iarg],"pos") == 0) {
      value[ivalue] = POS;
      nlen[ivalue] = 3;
      nstart[ivalue] = ncount;
      ncount += nlen[ivalue];
    } else if (strcmp(arg[iarg],"vel") == 0) {
      value[ivalue] = VEL;
      nlen[ivalue] = 3;
      nstart[ivalue] = ncount;
      ncount += nlen[ivalue];
    } else if (strcmp(arg[iarg],"quat") == 0) {
      value[ivalue] = QUAT;
      nlen[ivalue] = 4;
      nstart[ivalue] = ncount;
      ncount += nlen[ivalue];
    } else if (strcmp(arg[iarg],"omega") == 0) {
      value[ivalue] = OMG;
      nlen[ivalue] = 3;
      nstart[ivalue] = ncount;
      ncount += nlen[ivalue];
    } else error->all(FLERR,"Illegal compute body command");
    ivalue++;
    iarg++;
  }

  // initialization



  vector = NULL;
  vector = new double[ncount];

  if (ncount == 1) scalar_flag = 1;
  else {
      vector_flag = 1;
      size_vector = ncount;
  }
}

/* ---------------------------------------------------------------------- */

ComputeBody::~ComputeBody()
{
  if (copymode) return;

  delete [] value;
  delete [] nstart;
  delete [] nlen;
  delete [] vector;

}

/* ---------------------------------------------------------------------- */

void ComputeBody::init()
{
  reallocate();
}


double ComputeBody::compute_scalar()
{
    invoked_scalar = muse->system->ntimestep;
    compute_one(0);
    scalar = vector[0];
    return scalar;
}

/* ---------------------------------------------------------------------- */

void ComputeBody::compute_vector()
{
    invoked_scalar = muse->system->ntimestep;
 
    for (int i = 0; i < nvalue; i++) compute_one(i);
}

void ComputeBody::compute_one(int i)
{
    int startid = nstart[i];
    switch (value[i]) {
    case POS:
        vector[startid    ] = muse->body[bodyid]->pos(0);
        vector[startid + 1] = muse->body[bodyid]->pos(1);
        vector[startid + 2] = muse->body[bodyid]->pos(2);
        break;
    case VEL:
        vector[startid    ] = muse->body[bodyid]->vel(0);
        vector[startid + 1] = muse->body[bodyid]->vel(1);
        vector[startid + 2] = muse->body[bodyid]->vel(2);
        break;
    case QUAT:
        vector[startid    ] = muse->body[bodyid]->quat(0);
        vector[startid + 1] = muse->body[bodyid]->quat(1);
        vector[startid + 2] = muse->body[bodyid]->quat(2);
        vector[startid + 3] = muse->body[bodyid]->quat(3);
        break;
    case OMG:
        vector[startid    ] = muse->body[bodyid]->omega(0);
        vector[startid + 1] = muse->body[bodyid]->omega(1);
        vector[startid + 2] = muse->body[bodyid]->omega(2);
        break;
    default:
        error->all(FLERR, "Program problem, debug me!");
    }
}

void ComputeBody::reallocate()
{
}


