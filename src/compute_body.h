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
  int vcount;

  int *unique;               // unique keywords for tally, len = npergroup
  int npergroup;             // # of unique tally quantities per group
  int cellcount,cellmass;    // 1 if total cell count/mass is tallied
  int ntotal;                // total # of columns in tally array
  int nglocal;               // # of owned grid cells

  int *nmap;                 // # of tally quantities each user value uses
  int **map;                 // which tally columns each output value uses
  double **tally;            // array of tally quantities, cells by ntotal

  double eprefactor;         // conversion from velocity^2 to energy
  double tprefactor;         // conversion from KE to temperature
  double rvprefactor;        // conversion from rot/vib E to temperature

  void set_map(int, int);
  void reset_map();
};

}

#endif
#endif
