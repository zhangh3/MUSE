/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "mpi.h"
#include "error.h"
#include "ensemble.h"

using namespace MUSE_NS;
Error::Error(MUSE *muse) : Pointers(muse) {}


void Error::all(const char *file, int line, const char *str)
{
  MPI_Barrier(world);

  int me;
  MPI_Comm_rank(world,&me);

  if (me == 0) {
    if (screen) fprintf(screen,"ERROR: %s (%s:%d)\n",str,file,line);
    if (logfile) fprintf(logfile,"ERROR: %s (%s:%d)\n",str,file,line);
  }

  if (screen && screen != stdout) fclose(screen);
  if (logfile) fclose(logfile);

  MPI_Finalize();
  exit(1);
}


void Error::one(const char *file, int line, const char *str)
{
  int me;
  MPI_Comm_rank(world,&me);
  if (screen) fprintf(screen,"ERROR on proc %d: %s (%s:%d)\n",me,str,file,line);
  if (logfile) fprintf(logfile,"ERROR on proc %d: %s (%s:%d)\n",me,str,file,line);
  MPI_Abort(world,1);
}



void Error::warning(const char *file, int line, const char *str, int logflag)
{
	if (screen) fprintf(screen,"WARNING: %s (%s:%d)\n",str,file,line);
	if (logfile) fprintf(screen,"WARNING: %s (%s:%d)\n",str,file,line);

}


void Error::message(const char *file, int line, const char *str, int logflag)
{
  if (screen) fprintf(screen,"%s (%s:%d)\n",str,file,line);
  if (logfile) fprintf(screen,"%s (%s:%d)\n",str,file,line);
}