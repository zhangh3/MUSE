/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_RAN_PARK_H
#define MUSE_RAN_PARK_H

namespace MUSE_NS {

class RanPark {
 public:
  RanPark(int);
  RanPark(double);
  ~RanPark() {}
  void reset(double, int, int);
  double uniform();
  double gaussian();

 private:
  int seed,save;
  double second;
};

}

#endif
