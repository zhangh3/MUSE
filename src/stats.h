/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_STATS_H
#define MUSE_STATS_H

#include "pointers.h"

namespace MUSE_NS {

class Stats : protected Pointers {
 public:
  Stats(class MUSE *);
  ~Stats();

  void init();
  void modify_params(int, char **);
  void set_fields(int, char **);
  void header();
  void compute(int);
  int evaluate_keyword(char *, double *);

 private:
  char *line;
  char **keyword;
  int *vtype;

  int nfield;
  int me;

  char **format;
  char *format_line_user;
  char *format_float_user,*format_int_user;
  char **format_column_user;

  char *format_float_def,*format_int_def;

  int firststep;
  int flushflag,lineflag;

  double wall0;
  double last_tpcpu,last_spcpu;
  double last_time;
  int last_step;
                         // data used by routines that compute single values
  int ivalue;            // integer value to print
  double dvalue;         // double value to print
  int ifield;            // which field in thermo output is being computed
  int *field2index;      // which compute, variable calcs this field
  int *argindex1;        // indices into compute, scalar,vector
  int *argindex2;

  int ncompute;                // # of Compute objects called by stats
  char **id_compute;           // their IDs
  int *compute_which;          // 0/1/2 if should call scalar,vector,array
  class Compute **computes;    // list of ptrs to the Compute objects


  int nvariable;               // # of variables evaulated by stats
  char **id_variable;          // list of variable names
  int *variables;              // list of Variable indices

  // private methods

  void allocate();
  void deallocate();

  int add_compute(const char *, int);
  int add_surf_collide(const char *);
  int add_surf_react(const char *);
  int add_variable(const char *);

  typedef void (Stats::*FnPtr)();
  void addfield(const char *, FnPtr, int);
  FnPtr *vfunc;                // list of ptrs to functions

  void compute_compute();        // functions that compute a single value
  void compute_variable();

  // functions that compute a single value
  // customize a new keyword by adding a method prototype

  void compute_step();
  void compute_elapsed();
  void compute_elaplong();
  void compute_dt();
  void compute_cpu();
  void compute_tpcpu();
  void compute_spcpu();
  void compute_wall();

};

}

#endif

