/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "mpi.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "stats.h"
#include "MUSEsystem.h"

#include "modify.h"

#include "compute.h"
#include "input.h"
#include "variable.h"
#include "output.h"
#include "timer.h"
#include "memory.h"
#include "error.h"

using namespace MUSE_NS;

// customize a new keyword by adding to this list:

// step,elapsed,elaplong,dt,cpu,tpcpu,spcpu,wall,
// np,ntouch,ncomm,nbound,nexit,nscoll,nscheck,ncoll,nattempt,nreact,nsreact,
// npave,ntouchave,ncommave,nboundave,nexitave,nscollave,nscheckave,
// ncollave,nattemptave,nreactave,nsreactave,
// ngrid,nsplit,maxlevel,
// vol,lx,ly,lz,xlo,xhi,ylo,yhi,zlo,zhi

enum{INT,FLOAT};
enum{SCALAR,VECTOR,ARRAY};

#define INVOKED_SCALAR 1
#define INVOKED_VECTOR 2
#define INVOKED_ARRAY 4

#define MAXLINE 8192               // make this 4x longer than Input::MAXLINE
#define DELTA 8

/* ---------------------------------------------------------------------- */

Stats::Stats(MUSE *muse) : Pointers(muse)
{
  MPI_Comm_rank(world,&me);

  MPI_Barrier(world);
  wall0 = MPI_Wtime();

  line = new char[MAXLINE];

  keyword = NULL;
  vfunc = NULL;
  vtype = NULL;

  field2index = NULL;
  argindex1 = NULL;
  argindex2 = NULL;

  // default args

  char **arg = new char*[3];
  arg[0] = (char *) "step";
  arg[1] = (char *) "cpu";
  arg[2] = (char *) "cpu";//FIXME

  nfield = 3;
  allocate();
  set_fields(3,arg);

  delete [] arg;

  // stats_modify defaults

  flushflag = 0;

  // format strings
  format_float_def = (char *) "%12.8g";
  format_int_def = (char *) "%8d";
  format_line_user = NULL;
  format_float_user = NULL;
  format_int_user = NULL;
}

/* ---------------------------------------------------------------------- */

Stats::~Stats()
{
  delete [] line;
  deallocate();

  // format strings

  delete [] format_line_user;
  delete [] format_float_user;
  delete [] format_int_user;
}

/* ---------------------------------------------------------------------- */

void Stats::init()
{
  int i,n;

  // set format string for each field
  // add trailing '/n' to last value

  char *format_line = NULL;
  if (format_line_user) {
    int n = strlen(format_line_user) + 1;
    format_line = new char[n];
    strcpy(format_line,format_line_user);
  }

  char *ptr,*format_line_ptr;
  for (i = 0; i < nfield; i++) {
    format[i][0] = '\0';

    if (format_line) {
      if (i == 0) format_line_ptr = strtok(format_line," \0");
      else format_line_ptr = strtok(NULL," \0");
    }

    if (format_column_user[i]) ptr = format_column_user[i];
    else if (vtype[i] == FLOAT) {
      if (format_float_user) ptr = format_float_user;
      else if (format_line_user) ptr = format_line_ptr;
      else ptr = format_float_def;
    } else if (vtype[i] == INT) {
      if (format_int_user) ptr = format_int_user;
      else if (format_line_user) ptr = format_line_ptr;
      else ptr = format_int_def;
    }

    n = strlen(format[i]);
    sprintf(&format[i][n],"%s ",ptr);
  }
  strcat(format[nfield-1],"\n");

  delete [] format_line;

  int m;
  // find current ptr for each Compute ID

  for (int i = 0; i < ncompute; i++) {
    m = modify->find_compute(id_compute[i]);
    if (m < 0) error->all(FLERR,"Could not find stats compute ID");
    computes[i] = modify->compute[m];
  }

  // find current ptr for each Variable ID

  for (int i = 0; i < nvariable; i++) {
    m = input->variable->find(id_variable[i]);
    if (m < 0) error->all(FLERR,"Could not find stats variable name");
    variables[i] = m;
  }
}

/* ---------------------------------------------------------------------- */

void Stats::header()
{
  int loc = 0;
  for (int i = 0; i < nfield; i++)
    loc += sprintf(&line[loc],"%s ",keyword[i]);
  sprintf(&line[loc],"\n");

  if (me == 0) {
    if (screen) fprintf(screen,"%s",line);
    if (logfile) fprintf(logfile,"%s",line);
  }
}

/* ---------------------------------------------------------------------- */

void Stats::compute(int flag)
{
  int i;

  firststep = flag;

  // invoke Compute methods needed for stats keywords

  for (i = 0; i < ncompute; i++)
    if (compute_which[i] == SCALAR) {
      if (!(computes[i]->invoked_flag & INVOKED_SCALAR)) {
        computes[i]->compute_scalar();
        computes[i]->invoked_flag |= INVOKED_SCALAR;
      }
    } else if (compute_which[i] == VECTOR) {
      if (!(computes[i]->invoked_flag & INVOKED_VECTOR)) {
        computes[i]->compute_vector();
        computes[i]->invoked_flag |= INVOKED_VECTOR;
      }
    } else if (compute_which[i] == ARRAY) {
      if (!(computes[i]->invoked_flag & INVOKED_ARRAY)) {
        computes[i]->compute_array();
        computes[i]->invoked_flag |= INVOKED_ARRAY;
      }
    }

  // add each stat value to line with its specific format

  int loc = 0;
  for (ifield = 0; ifield < nfield; ifield++) {
    (this->*vfunc[ifield])();
    if (vtype[ifield] == FLOAT)
      loc += sprintf(&line[loc],format[ifield],dvalue);
    else if (vtype[ifield] == INT)
      loc += sprintf(&line[loc],format[ifield],ivalue);
  }

  // print line to screen and logfile

  if (me == 0) {
    if (screen) fprintf(screen,"%s",line);
    if (logfile) {
      fprintf(logfile,"%s",line);
      if (flushflag) fflush(logfile);
    }
  }
}

/* ----------------------------------------------------------------------
   modify stats parameters
------------------------------------------------------------------------- */

void Stats::modify_params(int narg, char **arg)
{
  if (narg == 0) error->all(FLERR,"Illegal stats_modify command");

  int iarg = 0;
  while (iarg < narg) {
    if (strcmp(arg[iarg],"every") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal stats_modify command");
      if (strstr(arg[iarg+1],"v_") == arg[iarg+1]) {
        delete [] output->var_stats;
        int n = strlen(&arg[iarg+1][2]) + 1;
        output->var_stats = new char[n];
        strcpy(output->var_stats,&arg[iarg+1][2]);
      } else error->all(FLERR,"Illegal stats_modify command");
      output->stats_every = 0;
      iarg += 2;

    } else if (strcmp(arg[iarg],"flush") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal stats_modify command");
      if (strcmp(arg[iarg+1],"no") == 0) flushflag = 0;
      else if (strcmp(arg[iarg+1],"yes") == 0) flushflag = 1;
      else error->all(FLERR,"Illegal stats_modify command");
      iarg += 2;

    } else if (strcmp(arg[iarg],"format") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal stats_modify command");

      if (strcmp(arg[iarg+1],"none") == 0) {
        delete [] format_line_user;
        delete [] format_int_user;
        delete [] format_float_user;
        format_line_user = NULL;
        format_int_user = NULL;
        format_float_user = NULL;
        for (int i = 0; i < nfield; i++) {
          delete [] format_column_user[i];
          format_column_user[i] = NULL;
        }
        iarg += 2;
        continue;
      }

      if (iarg+3 > narg) error->all(FLERR,"Illegal stats_modify command");

      if (strcmp(arg[iarg+1],"line") == 0) {
        delete [] format_line_user;
        int n = strlen(arg[iarg+2]) + 1;
        format_line_user = new char[n];
        strcpy(format_line_user,arg[iarg+2]);
      } else if (strcmp(arg[iarg+1],"int") == 0) {
        if (format_int_user) delete [] format_int_user;
        int n = strlen(arg[iarg+2]) + 1;
        format_int_user = new char[n];
        strcpy(format_int_user,arg[iarg+2]);//zh:有没有问题？？ stas_modify format %d
      } else if (strcmp(arg[iarg+1],"float") == 0) {
        if (format_float_user) delete [] format_float_user;
        int n = strlen(arg[iarg+2]) + 1;
        format_float_user = new char[n];
        strcpy(format_float_user,arg[iarg+2]);
      } else {
        int i = input->inumeric(FLERR,arg[iarg+1]) - 1;
        if (i < 0 || i >= nfield)
          error->all(FLERR,"Illegal stats_modify command");
        if (format_column_user[i]) delete [] format_column_user[i];
        int n = strlen(arg[iarg+2]) + 1;
        format_column_user[i] = new char[n];
        strcpy(format_column_user[i],arg[iarg+2]);
      }
      iarg += 3;

    } else error->all(FLERR,"Illegal stats_modify command");
  }
}

/* ----------------------------------------------------------------------
   allocate all per-field memory
------------------------------------------------------------------------- */

void Stats::allocate()
{
  int n = nfield;

  keyword = new char*[n];
  for (int i = 0; i < n; i++) keyword[i] = new char[32];
  vfunc = new FnPtr[n];
  vtype = new int[n];

  format = new char*[n];
  for (int i = 0; i < n; i++) format[i] = new char[32];
  format_column_user = new char*[n];
  for (int i = 0; i < n; i++) format_column_user[i] = NULL;

  field2index = new int[n];
  argindex1 = new int[n];
  argindex2 = new int[n];

  // memory for computes, variables

  ncompute = 0;
  id_compute = new char*[n];
  compute_which = new int[n];
  computes = new Compute*[n];

  nvariable = 0;
  id_variable = new char*[n];
  variables = new int[n];
}

/* ----------------------------------------------------------------------
   deallocate all per-field memory
------------------------------------------------------------------------- */

void Stats::deallocate()
{
  int n = nfield;

  for (int i = 0; i < n; i++) delete [] keyword[i];
  delete [] keyword;
  delete [] vfunc;
  delete [] vtype;

  for (int i = 0; i < n; i++) delete [] format[i];
  delete [] format;
  for (int i = 0; i < n; i++) delete [] format_column_user[i];
  delete [] format_column_user;

  delete [] field2index;
  delete [] argindex1;
  delete [] argindex2;

  for (int i = 0; i < ncompute; i++) delete [] id_compute[i];
  delete [] id_compute;
  delete [] compute_which;
  delete [] computes;

  for (int i = 0; i < nvariable; i++) delete [] id_variable[i];
  delete [] id_variable;
  delete [] variables;
}

/* ----------------------------------------------------------------------
   set fields of stats output from args
   called by constructor with default fields
   called by Input with stats_style command args
------------------------------------------------------------------------- */

void Stats::set_fields(int narg, char **arg)
{
  deallocate();

  // expand args if any have wildcard character "*"

  int expand = 0;
  char **earg;
  int nargnew = input->expand_args(narg,arg,0,earg);

  if (earg != arg) expand = 1;
  arg = earg;

  nfield = nargnew;
  allocate();
  nfield = 0;

  // customize a new keyword by adding to if statement

  for (int i = 0; i < nargnew; i++) {
    if (strcmp(arg[i],"step") == 0) {
      addfield("Step",&Stats::compute_step,INT);
    } else if (strcmp(arg[i],"elapsed") == 0) {
      addfield("Elapsed",&Stats::compute_elapsed, INT);
    } else if (strcmp(arg[i],"elaplong") == 0) {
      addfield("Elapsed",&Stats::compute_elaplong, INT);
    } else if (strcmp(arg[i],"dt") == 0) {
      addfield("Dt",&Stats::compute_dt,FLOAT);
    } else if (strcmp(arg[i],"cpu") == 0) {
      addfield("CPU",&Stats::compute_cpu,FLOAT);
    } else if (strcmp(arg[i],"tpcpu") == 0) {
      addfield("T/CPU",&Stats::compute_tpcpu,FLOAT);
    } else if (strcmp(arg[i],"spcpu") == 0) {
      addfield("S/CPU",&Stats::compute_spcpu,FLOAT);
    } else if (strcmp(arg[i],"wall") == 0) {
      addfield("WALL",&Stats::compute_wall,FLOAT);

    // compute value = c_ID,  variable value = v_ID
    // count trailing [] and store int arguments
    // copy = at most 8 chars of ID to pass to addfield

    } else if ((strncmp(arg[i],"c_",2) == 0) ||
               (strncmp(arg[i],"v_",2) == 0)) {

      int n = strlen(arg[i]);
      char *id = new char[n];
      strcpy(id,&arg[i][2]);

      // parse zero or one or two trailing brackets from ID
      // argindex1,argindex2 = int inside each bracket pair, 0 if no bracket

      char *ptr = strchr(id,'[');
      if (ptr == NULL) argindex1[nfield] = 0;
      else {
        *ptr = '\0';
        argindex1[nfield] = input->variable->int_between_brackets(ptr,0);
        ptr++;
        if (*ptr == '[') {
          argindex2[nfield] = input->variable->int_between_brackets(ptr,0);
          ptr++;
        } else argindex2[nfield] = 0;
      }

      if (arg[i][0] == 'c') {
        n = modify->find_compute(id);
        if (n < 0) error->all(FLERR,"Could not find stats compute ID");
        if (argindex1[nfield] == 0 && modify->compute[n]->scalar_flag == 0)
          error->all(FLERR,"Stats compute does not compute scalar");
        if (argindex1[nfield] > 0 && argindex2[nfield] == 0) {
          if (modify->compute[n]->vector_flag == 0)
            error->all(FLERR,"Stats compute does not compute vector");
          if (argindex1[nfield] > modify->compute[n]->size_vector)
            error->all(FLERR,"Stats compute vector is accessed out-of-range");
        }
        if (argindex1[nfield] > 0 && argindex2[nfield] > 0) {
          if (modify->compute[n]->array_flag == 0)
            error->all(FLERR,"Stats compute does not compute array");
          if (argindex1[nfield] > modify->compute[n]->size_array_rows ||
              argindex2[nfield] > modify->compute[n]->size_array_cols)
            error->all(FLERR,"Stats compute array is accessed out-of-range");
        }

        if (argindex1[nfield] == 0)
          field2index[nfield] = add_compute(id,SCALAR);
        else if (argindex2[nfield] == 0)
          field2index[nfield] = add_compute(id,VECTOR);
        else
          field2index[nfield] = add_compute(id,ARRAY);
        addfield(arg[i],&Stats::compute_compute,FLOAT);

      } else if (arg[i][0] == 'v') {
        n = input->variable->find(id);
        if (n < 0) error->all(FLERR,"Could not find stats variable name");
        if (input->variable->equal_style(n) == 0)
          error->all(FLERR,"Stats variable is not equal-style variable");
        if (argindex1[nfield])
          error->all(FLERR,"Stats variable cannot be indexed");

        field2index[nfield] = add_variable(id);
        addfield(arg[i],&Stats::compute_variable,FLOAT);
      }

      delete [] id;

    }
    else {
        char str[128];
        sprintf(str, "Invalid keyword %s in stats_style command", arg[i]);
        error->all(FLERR, str);
    }
  }

  // if wildcard expansion occurred, free earg memory from expand_args()

  if (expand) {
    for (int i = 0; i < nargnew; i++) delete [] earg[i];
    memory->sfree(earg);
  }
}

/* ----------------------------------------------------------------------
   add field to list of quantities to print
------------------------------------------------------------------------- */

void Stats::addfield(const char *key, FnPtr func, int typeflag)
{
  strcpy(keyword[nfield],key);
  vfunc[nfield] = func;
  vtype[nfield] = typeflag;
  nfield++;
}

/* ----------------------------------------------------------------------
   add compute ID to list of Compute objects to call
   return location of where this Compute is in list
   if already in list with same which, do not add, just return index
------------------------------------------------------------------------- */

int Stats::add_compute(const char *id, int which)
{
  int icompute;
  for (icompute = 0; icompute < ncompute; icompute++)
    if ((strcmp(id,id_compute[icompute]) == 0) &&
        which == compute_which[icompute]) break;
  if (icompute < ncompute) return icompute;

  int n = strlen(id) + 1;
  id_compute[ncompute] = new char[n];
  strcpy(id_compute[ncompute],id);
  compute_which[ncompute] = which;
  ncompute++;
  return ncompute-1;
}


/* ----------------------------------------------------------------------
   add variable ID to list of Variables to evaluate
------------------------------------------------------------------------- */

int Stats::add_variable(const char *id)
{
  int n = strlen(id) + 1;
  id_variable[nvariable] = new char[n];
  strcpy(id_variable[nvariable],id);
  nvariable++;
  return nvariable-1;
}

/* ----------------------------------------------------------------------
   compute a single stats value, word is any stats keyword
   called when a variable is evaluated by Variable class
   return value as double in answer
   return 0 if str is recoginzed keyword, 1 if unrecognized
   customize a new keyword by adding to if statement
------------------------------------------------------------------------- */

int Stats::evaluate_keyword(char *word, double *answer)
{
  // invoke a lo-level stats routine to compute the variable value

  if (strcmp(word,"step") == 0) {
    compute_step();
    dvalue = ivalue;

  } else if (strcmp(word,"elapsed") == 0) {

    compute_elapsed();
    dvalue = ivalue;

  } else if (strcmp(word,"elaplong") == 0) {

    compute_elaplong();
    dvalue = ivalue;

  } else if (strcmp(word,"dt") == 0) {
    compute_dt();

  } else if (strcmp(word,"cpu") == 0) {

    compute_cpu();

  } else if (strcmp(word,"tpcpu") == 0) {

    compute_tpcpu();

  } else if (strcmp(word,"spcpu") == 0) {

    compute_spcpu();

  } else if (strcmp(word,"wall") == 0) {
    compute_wall();
  } 
  else return 1;

  *answer = dvalue;
  return 0;
}


void Stats::compute_compute()
{
  int m = field2index[ifield];
  Compute *compute = computes[m];

  if (compute_which[m] == SCALAR)
    dvalue = compute->scalar;
  else if (compute_which[m] == VECTOR)
    dvalue = compute->vector[argindex1[ifield]-1];
  else
    dvalue = compute->array[argindex1[ifield]-1][argindex2[ifield]-1];
}

/* ---------------------------------------------------------------------- */

void Stats::compute_variable()
{
  dvalue = input->variable->compute_equal(variables[field2index[ifield]]);
}

/* ----------------------------------------------------------------------
   one method for every keyword stats can output
   called by compute() or evaluate_keyword()
   compute will have already been called
   set ivalue/dvalue/bivalue if value is int/double
   customize a new keyword by adding a method
------------------------------------------------------------------------- */

void Stats::compute_step()
{
  ivalue = muse->system->ntimestep;
}

/* ---------------------------------------------------------------------- */

void Stats::compute_elapsed()
{
  ivalue = muse->system->ntimestep - muse->system->firststep;
}

/* ---------------------------------------------------------------------- */

void Stats::compute_elaplong()
{
  ivalue = muse->system->ntimestep - muse->system->beginstep;
}

/* ---------------------------------------------------------------------- */

void Stats::compute_dt()
{
  dvalue = muse->system->dt;
}

/* ---------------------------------------------------------------------- */

void Stats::compute_cpu()
{
  if (firststep == 0) dvalue = 0.0;
  else dvalue = timer->elapsed(TIME_LOOP);
}

/* ---------------------------------------------------------------------- */

void Stats::compute_tpcpu()
{
  double new_cpu;
  double new_time = muse->system->ntimestep * muse->system->dt;

  if (firststep == 0) {
    new_cpu = 0.0;
    dvalue = 0.0;
  } else {
    new_cpu = timer->elapsed(TIME_LOOP);
    double cpu_diff = new_cpu - last_tpcpu;
    double time_diff = new_time - last_time;
    if (time_diff > 0.0 && cpu_diff > 0.0) dvalue = time_diff/cpu_diff;
    else dvalue = 0.0;
  }

  last_time = new_time;
  last_tpcpu = new_cpu;
}

/* ---------------------------------------------------------------------- */

void Stats::compute_spcpu()
{
  double new_cpu;
  int new_step = muse->system->ntimestep;

  if (firststep == 0) {
    new_cpu = 0.0;
    dvalue = 0.0;
  } else {
    new_cpu = timer->elapsed(TIME_LOOP);
    double cpu_diff = new_cpu - last_spcpu;
    int step_diff = new_step - last_step;
    if (cpu_diff > 0.0) dvalue = step_diff/cpu_diff;
    else dvalue = 0.0;
  }

  last_step = new_step;
  last_spcpu = new_cpu;
}

/* ---------------------------------------------------------------------- */

void Stats::compute_wall()
{
  dvalue = MPI_Wtime() - wall0;
}
