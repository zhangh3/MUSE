/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifdef COMMAND_CLASS

CommandStyle(create,Create)

#else

#ifndef MUSE_CREATE_H
#define MUSE_CREATE_H

#include "pointers.h"

namespace MUSE_NS {

class Create : protected Pointers {
 public:
	 int id;
	 Create(class MUSE *);
	 void command(int, char**);
	 void create_body(int, char**);
	 void create_joint(int, char**);
	 void create_system(int, char**);
};

}

#endif
#endif

/* ERROR/WARNING messages:

*/
