/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_RAN_MARS_H
#define MUSE_RAN_MARS_H

#include "pointers.h"

namespace MUSE_NS {

class RanMars : protected Pointers {
 public:
  RanMars(class MUSE *);
  ~RanMars();
  void init(int);
  double uniform();
  double gaussian();

 private:
  int initflag,save;
  int i97,j97;
  double c,cd,cm;
  double second;
  double *u;
};

}

#endif
