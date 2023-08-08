/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#ifndef MUSE_VARIABLE_H
#define MUSE_VARIABLE_H

#include "stdio.h"
#include "pointers.h"

namespace MUSE_NS {

class Variable : protected Pointers {
 public:
  Variable(class MUSE *);
  ~Variable();
  void set(int, char **);
  void set(char *, int, char **);
  int next(int, char **);
  int find(char *);

  int equal_style(int);
  int particle_style(int);
  int grid_style(int);
  int surf_style(int);
  int internal_style(int);

  char *retrieve(char *);
  double compute_equal(int);
  double compute_equal(char *);
  void compute_particle(int, double *, int, int);
  void compute_grid(int, double *, int, int);
  void compute_surf(int, double *, int, int) {}  // not yet supported
  void internal_set(int, double);

  int int_between_brackets(char *&, int);
  double evaluate_boolean(char *);

 private:
  int me;
  int nvar;                // # of defined variables
  int maxvar;              // max # of variables following lists can hold
  char **names;            // name of each variable
  int *style;              // style of each variable
  int *num;                // # of values for each variable
  int *which;              // next available value for each variable
  int *pad;                // 1 = pad loop/uloop variables with 0s, 0 = no pad
  class VarReader **reader;   // variable that reads from file
  char ***data;            // str value of each variable's values
  double *dvalue;          // single numeric value for internal variables

  int *eval_in_progress;   // flag if evaluation of variable is in progress

  class RanPark *randomequal;     // RNG for equal-style vars
  class RanPark *randomparticle;  // RNG for particle-style vars

  int precedence[17];      // precedence level of math operators
                           // set length to include up to OR in enum

  int treestyle;           // tree being used for particle or grid-style var

                           // local copies of compute vector_grid vectors
  int nvec_storage;        // # of vectors currently stored locally
  int maxvec_storage;      // max # of vectors in vec_storage
  double **vec_storage;    // list of vector copies
  int *maxlen_storage;     // allocated length of each vector

  struct Tree {            // parse tree for particle-style variables
    double value;          // single scalar
    double *array;         // per-atom or per-type list of doubles
    char *carray;          // ptr into data struct with nstride = sizeof(struct)
    int type;              // operation, see enum{} in variable.cpp
    int nstride;           // stride between atoms if array is a 2d array
    int selfalloc;         // 1 if array is allocated here, else 0
    int ivalue1,ivalue2;   // extra values for needed for gmask,rmask,grmask
    Tree *left,*middle,*right;    // ptrs further down tree
  };

  void remove(int);
  void grow();
  void copy(int, char **, char **);
  double evaluate(char *, Tree **);
  double collapse_tree(Tree *);
  double eval_tree(Tree *, int);
  void free_tree(Tree *);
  int find_matching_paren(char *, int, char *&);
  int math_function(char *, char *, Tree **, Tree **, int &, double *, int &);
  int special_function(char *, char *, Tree **, Tree **,
		       int &, double *, int &);
  int is_particle_vector(char *);
  void particle_vector(char *, Tree **, Tree **, int &);
  int is_constant(char *);
  double constant(char *);
  char *find_next_comma(char *);
  void print_tree(Tree *, int);
  double *add_storage(double *);
};

class VarReader : protected Pointers {
 public:
  VarReader(class MUSE *, char *, char *, int);
  ~VarReader();
  int read_scalar(char *);

 private:
  int me,style;
  FILE *fp;
};

}

#endif

