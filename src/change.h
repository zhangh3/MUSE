/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifdef COMMAND_CLASS

CommandStyle(change,Change)

#else

#ifndef MUSE_CHANGE_H
#define MUSE_CHANGE_H

#include "pointers.h"

namespace MUSE_NS {

class Change : protected Pointers {
 public:
	 int id;
	 Change(class MUSE *);
	 void command(int, char**);
	 void change_body(int, char**);
	 void change_joint(int, char**);
	 void change_system(int, char**);
};

}

#endif
#endif

/* ERROR/WARNING messages:

*/
