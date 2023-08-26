/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_MODIFY_H
#define MUSE_MODIFY_H

#include "pointers.h"

namespace MUSE_NS {

class Modify : protected Pointers {
 public:
  int nfix,maxfix;
  int n_start_of_step,n_end_of_step;
  int n_pergrid,n_add_particle,n_gas_react,n_surf_react;

  int ncompute,maxcompute;   // list of computes
  class Compute **compute;

  Modify(class MUSE *);
  ~Modify();
  void init();
  void setup();
  virtual void start_of_step();
  virtual void end_of_step();


  void add_compute(int, char **);
  void delete_compute(const char *);
  int find_compute(const char *);

  void clearstep_compute();
  void addstep_compute(int);
  void addstep_compute_all();

  void list_init_computes();


 protected:

  // lists of fixes to apply at different stages of timestep

  int *list_start_of_step,*list_end_of_step;

  int *end_of_step_every;

  int n_timeflag;            // list of computes that store time invocation
  int *list_timeflag;

};

}

#endif

