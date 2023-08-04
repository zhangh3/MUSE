/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_TIMER_H
#define MUSE_TIMER_H

#include "pointers.h"

namespace MUSE_NS {

enum{TIME_LOOP,TIME_COMM,TIME_MODIFY,TIME_OUTPUT,TIME_N};

class Timer : protected Pointers {
 public:
  double *array;

  Timer(class MUSE *);
  ~Timer();
  void init();
  void stamp();
  void stamp(int);
  void barrier_start(int);
  void barrier_stop(int);
  double elapsed(int);

 private:
  double previous_time;
};

}

#endif
