/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifdef COMMAND_CLASS

CommandStyle(run,Run)

#else

#ifndef MUSE_RUN_H
#define MUSE_RUN_H

#include "pointers.h"

namespace MUSE_NS {

class Run : protected Pointers {
 public:
  int sysid;
  Run(class MUSE *);
  void command(int, char **);
};

}

#endif
#endif
