/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

// Pointers class contains ptrs to master copy of
//   fundamental MUSE class ptrs stored in muse.h
// every MUSE class inherits from Pointers to access muse.h ptrs
// these variables are auto-initialized by Pointer class constructor
// *& variables are really pointers to the pointers in muse.h
// & enables them to be accessed directly in any class.

#ifndef MUSE_POINTERS_H
#define MUSE_POINTERS_H

#include "mpi.h"
#include "stdio.h"
#include "muse.h"

namespace MUSE_NS {

#define FLERR __FILE__,__LINE__
#define MIN(A,B) ((A) < (B)) ? (A) : (B)
#define MAX(A,B) ((A) > (B)) ? (A) : (B)

class Pointers {
 public:
  Pointers(MUSE *ptr) :   //del: ���ﹹ�캯�������ð��������ʼ����Ա������e.g. Pointers��ensemble��ʼ��Ϊptr->ensemble
    muse(ptr),
	memory(ptr->memory),
    ensemble(ptr->ensemble),
    error(ptr->error),
	world(ptr->world),
    input(ptr->input),
    timer(ptr->timer),
    infile(ptr->infile),
    screen(ptr->screen),
    logfile(ptr->logfile) {}

  virtual ~Pointers() {}

 protected:
  MUSE *muse;
  Memory *&memory;
  Error *&error;
  Ensemble *&ensemble;
  Input *&input;
  Timer *&timer;

  MPI_Comm &world;
  FILE *&infile;
  FILE *&screen;
  FILE *&logfile;
 };

}

#endif
