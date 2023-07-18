/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */


#include "mpi.h"
#include "ensemble.h"
#include "memory.h"
#include "random_mars.h"

using namespace MUSE_NS;

Ensemble::Ensemble(MUSE *muse, MPI_Comm communicator) : Pointers(muse)
{
  world = communicator;
  MPI_Comm_rank(world,&me);
  MPI_Comm_size(world,&nprocs);

  uscreen = stdout;
  ulogfile = NULL;
  nSim = 1;
  ranmaster = new RanMars(muse);

}

/* ---------------------------------------------------------------------- */

Ensemble::~Ensemble()
{
}
