/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifdef COMMAND_CLASS

CommandStyle(create_system,CreateSystem)

#else

#ifndef MUSE_CREATE_SYSTEM_H
#define MUSE_CREATE_SYSTEM_H

#include "pointers.h"

namespace MUSE_NS {

class CreateSystem : protected Pointers {
 public:
	 int id;
	 CreateSystem(class MUSE *);
  void command(int, char **);
};

}

#endif
#endif

/* ERROR/WARNING messages:

*/
