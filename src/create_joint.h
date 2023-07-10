/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifdef COMMAND_CLASS

CommandStyle(create_joint,CreateJoint)

#else

#ifndef MUSE_CREATE_JOINT_H
#define MUSE_CREATE_JOINT_H

#include "pointers.h"

namespace MUSE_NS {

class CreateJoint : protected Pointers {
 public:
	 int id;
	 CreateJoint(class MUSE *);
  void command(int, char **);
};

}

#endif
#endif

/* ERROR/WARNING messages:

*/
