/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */


#include "stdlib.h"
#include "string.h"
#include "run.h"
#include "input.h"
#include "error.h"
#include "MUSEsystem.h"


#include <iostream>
#include <fstream>
#include<ctime>
#include <iomanip>
using namespace MUSE_NS;

/* ---------------------------------------------------------------------- */

Run::Run(MUSE *muse) : Pointers(muse) {
    sysid = -1;
}

/* ---------------------------------------------------------------------- */

void Run::command(int narg, char **arg)
{
  if (narg < 2) error->all(FLERR,"Illegal run command");

  int isys;

  for (isys = 0; isys < muse->nSystems; isys++)
      if (strcmp(arg[0], muse->system[isys]->name) == 0) break;

  if (isys < muse->nSystems) {
      sysid= isys;
  }
  else {
      char str[128];
      sprintf(str, "Cannot find system with name: %s", arg[0]);
      error->all(FLERR, str);
  }

  int nsteps_input = input->inumeric(FLERR, arg[1]);

  // parse optional args

  int uptoflag = 0;
  int startflag = 0;
  int stopflag = 0;
  int start,stop;
  int preflag = 1;
  int postflag = 1;
  int nevery = 0;
  int ncommands = 0;
  int first,last;

  int iarg = 2;
  while (iarg < narg) {
    if (strcmp(arg[iarg],"upto") == 0) {
      if (iarg+1 > narg) error->all(FLERR,"Illegal run command");
      uptoflag = 1;
      iarg += 1;
    } else if (strcmp(arg[iarg],"start") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal run command");
      startflag = 1;

      start = input->inumeric(FLERR, arg[iarg+1]);
      iarg += 2;
    } else if (strcmp(arg[iarg],"stop") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal run command");
      stopflag = 1;
      stop = input->inumeric(FLERR, arg[iarg+1]);
      iarg += 2;
    } else if (strcmp(arg[iarg],"pre") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal run command");
      if (strcmp(arg[iarg+1],"no") == 0) preflag = 0;
      else if (strcmp(arg[iarg+1],"yes") == 0) preflag = 1;
      else error->all(FLERR,"Illegal run command");
      iarg += 2;
    } else if (strcmp(arg[iarg],"post") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal run command");
      if (strcmp(arg[iarg+1],"no") == 0) postflag = 0;
      else if (strcmp(arg[iarg+1],"yes") == 0) postflag = 1;
      else error->all(FLERR,"Illegal run command");
      iarg += 2;

      // all remaining args are commands
      // first,last = arg index of first/last commands
      // set ncommands = 0 if single command and it is NULL

    } else if (strcmp(arg[iarg],"every") == 0) {
      if (iarg+3 > narg) error->all(FLERR,"Illegal run command");
      nevery = atoi(arg[iarg+1]);
      if (nevery <= 0) error->all(FLERR,"Illegal run command");
      first = iarg+2;
      last = narg-1;
      ncommands = last-first + 1;
      if (ncommands == 1 && strcmp(arg[first],"NULL") == 0) ncommands = 0;
      iarg = narg;
    } else error->all(FLERR,"Illegal run command");
  }

  // set nsteps as integer, using upto value if specified

  int nsteps;
  if (!uptoflag) {
    if (nsteps_input < 0 || nsteps_input > INT_MAX)
      error->all(FLERR,"Invalid run command N value");
    nsteps = static_cast<int> (nsteps_input);
  } else {
    int delta = nsteps_input - muse->system[sysid]->ntimestep;
    if (delta < 0 || delta > INT_MAX)
      error->all(FLERR,"Invalid run command upto value");
    nsteps = static_cast<int> (delta);
  }

  // error check

  if (startflag) {
    if (start < 0 || start > INT_MAX)
      error->all(FLERR,"Invalid run command start/stop value");
    if (start > muse->system[sysid]->ntimestep)
      error->all(FLERR,"Run command start value is after start of run");
  }
  if (stopflag) {
    if (stop < 0 || stop > INT_MAX)
      error->all(FLERR,"Invalid run command start/stop value");
    if (stop < muse->system[sysid]->ntimestep + nsteps)
      error->all(FLERR,"Run command stop value is before end of run");
  }

  // if nevery, make copies of arg strings that are commands
  // required because re-parsing commands via input->one() will wipe out args

  char **commands = NULL;
  if (nevery && ncommands > 0) {
    commands = new char*[ncommands];
    ncommands = 0;
    for (int i = first; i <= last; i++) {
      int n = strlen(arg[i]) + 1;
      commands[ncommands] = new char[n];
      strcpy(commands[ncommands],arg[i]);
      ncommands++;
    }
  }

  // perform a single run
  // use start/stop to set begin/end step
  // if pre or 1st run, do System init/setup,
  //   else just init timer and setup output
  // if post, do full Finish, else just print time

  

  if (nevery == 0) {

    muse->system[sysid]->nsteps = nsteps;
    muse->system[sysid]->firststep = muse->system[sysid]->ntimestep;
    muse->system[sysid]->laststep = muse->system[sysid]->ntimestep + nsteps;
    if (muse->system[sysid]->laststep < 0 || muse->system[sysid]->laststep > INT_MAX)
      error->all(FLERR,"Too many timesteps");

    if (startflag) muse->system[sysid]->beginstep = start;
    else muse->system[sysid]->beginstep = muse->system[sysid]->firststep;
    if (stopflag) muse->system[sysid]->endstep = stop;
    else muse->system[sysid]->endstep = muse->system[sysid]->laststep;

    if (preflag || muse->system[sysid]->first_update == 0) {
    //第一次运行需要设置的东西
        muse->system[sysid]->setup();
    } 

    clock_t start, end;
    start = clock();
    muse->system[sysid]->solve(nsteps);
    end = clock();
    std::cout << "run time: " << std::fixed << std::setprecision(2) << 1000 * (double)(end - start) / CLOCKS_PER_SEC << "ms" << std::endl;

    std::ofstream fout;
    fout.open("./res.txt", std::ios::app);
    for (int i = 1; i < muse->system[sysid]->xlog.size();i++)
    {
        fout << muse->system[sysid]->xlog[i].transpose() << std::endl;
    }
    fout.close();

  // perform multiple runs optionally interleaved with invocation command(s)
  // use start/stop to set begin/end step
  // if pre or 1st iteration of multiple runs, do System init/setup,
  //   else just init timer and setup output
  // if post or last iteration, do full Finish, else just print time

  } else {
    int iter = 0;
    int nleft = nsteps;
    double time_multiple_runs = 0.0;

    while (nleft > 0 || iter == 0) {
      nsteps = MIN(nleft,nevery);

      muse->system[sysid]->nsteps = nsteps;
      muse->system[sysid]->firststep = muse->system[sysid]->ntimestep;
      muse->system[sysid]->laststep = muse->system[sysid]->ntimestep + nsteps;
      if (muse->system[sysid]->laststep < 0 || muse->system[sysid]->laststep > INT_MAX)
        error->all(FLERR,"Too many timesteps");

      if (startflag) muse->system[sysid]->beginstep = start;
      else muse->system[sysid]->beginstep = muse->system[sysid]->firststep;
      if (stopflag) muse->system[sysid]->endstep = stop;
      else muse->system[sysid]->endstep = muse->system[sysid]->laststep;

      if (preflag || iter == 0) {
          muse->system[sysid]->setup();
      } 


      clock_t start, end;
      start = clock();
      muse->system[sysid]->solve(nsteps);
      end = clock();
      std::cout << "run time: " << std::fixed << std::setprecision(2) << 1000 * (double)(end - start) / CLOCKS_PER_SEC << "ms" << std::endl;

      std::ofstream fout;
      fout.open("./res.txt", std::ios::app);
      for (int i = 1; i < muse->system[sysid]->xlog.size();i++)
      {
          fout << muse->system[sysid]->xlog[i].transpose() << std::endl;
      }
      fout.close();
 

      // wrap command invocation with clearstep/addstep
      // since a command may invoke computes via variables

      if (ncommands) {
            error->warning(FLERR, "ncommands apear!!"); //zhangh3:FIXME
      }

      nleft -= nsteps;
      iter++;
    }
  }

  muse->system[sysid]->firststep = muse->system[sysid]->laststep = 0;
  muse->system[sysid]->beginstep = muse->system[sysid]->endstep = 0;

  if (commands) {
    for (int i = 0; i < ncommands; i++) delete [] commands[i];
    delete [] commands;
  }
}
