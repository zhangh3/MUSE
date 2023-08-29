/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifdef COMPUTE_CLASS

ComputeStyle(body,ComputeBody)

#else

#ifndef MUSE_COMPUTE_BODY_H
#define MUSE_COMPUTE_BODY_H

#include "compute.h"

namespace MUSE_NS {

class ComputeBody : public Compute {
 public:
  ComputeBody(class MUSE *, int, char **);
  ~ComputeBody();
  void init();
  virtual void reallocate();

  

 protected:
  int bodyid;                //
  int imix,nvalue;
  int *value;                // keyword for each user requested value
  int ncount;


  int *nlen;                 // # of tally quantities each user value uses
  int* nstart;                // # of tally quantities each user value uses



  void set_map(int, int);
  void reset_map();

  double compute_scalar();
  void compute_vector();
  void compute_one(int);
};

}

#endif
#endif
