/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_COMPUTE_H
#define MUSE_COMPUTE_H

#include "pointers.h"

namespace MUSE_NS {

class Compute : protected Pointers {
 public:

  int sysid;
  char *name,*style;

  double scalar;            // computed global scalar
  double *vector;           // computed global vector
  double **array;           // computed global array
  double *vector_body;  // computed per-body vector
  double **array_body;  // computed per-body array

  int scalar_flag;          // 0/1 if compute_scalar() function exists
  int vector_flag;          // 0/1 if compute_vector() function exists
  int array_flag;           // 0/1 if compute_array() function exists
  int size_vector;          // length of global vector
  int size_array_rows;      // rows in global array
  int size_array_cols;      // columns in global array

  int per_body_flag;      // 0/1 if compute_per_body() function exists
  int size_per_body_cols; // 0 = vector, N = columns in per-body array

  int timeflag;       // 1 if Compute stores list of timesteps it's called on
  int ntime;          // # of entries in time list
  int maxtime;        // max # of entries time list can hold
  int *tlist;         // list of timesteps the Compute is called on

  int invoked_flag;        // non-zero if invoked or accessed this step, 0 if not
  int invoked_scalar;      // last timestep on which compute_scalar() was invoked
  int invoked_vector;      // ditto for compute_vector()
  int invoked_array;       // ditto for compute_array()
  int invoked_per_body;    // ditto for compute_per_body()

  Compute(class MUSE *, int, char **);
  Compute(class MUSE* muse) : Pointers(muse) {} 
  virtual ~Compute();
  virtual void init() {}

  virtual double compute_scalar() {return 0.0;}
  virtual void compute_vector() {}
  virtual void compute_array() {}
  virtual void compute_per_body() {}

  virtual void clear() {}


  virtual void reallocate() {}

  // methods in compute.cpp

  void addstep(int);
  int matchstep(int);
  void clearstep();


  int copy,copymode;        // 1 if copy of class (prevents deallocation of
                            //  base class when child copy is destroyed)
};

}

#endif
