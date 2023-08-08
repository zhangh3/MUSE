/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_ENSEMBLE_H
#define MUSE_ENSEMBLE_H

#include "pointers.h"

namespace MUSE_NS {

class Ensemble : protected Pointers {
 public:

  MPI_Comm world;         // communicator for entire universe
  int me,nprocs;          // my place in universe

  FILE *uscreen;          // universe screen output
  FILE *ulogfile;         // universe logfile

  int nSim;            // Simulations number

  class RanMars* ranmaster;   // master random number generator

  Ensemble(class MUSE *, MPI_Comm);
  ~Ensemble();
};

}

#endif