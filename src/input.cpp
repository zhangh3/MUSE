/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "mpi.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "MUSEunistd.h"
#include "sys/stat.h"
#include "input.h"
#include "math_extra.h"
#include "error.h"
#include "memory.h"
#include "style_command.h"
#include "variable.h"

using namespace MUSE_NS;

#define DELTALINE 256
#define DELTA 4

/* ---------------------------------------------------------------------- */


Input::Input(MUSE *muse, int argc, char **argv) : Pointers(muse)
{
  MPI_Comm_rank(world,&me);

  maxline = maxcopy = maxwork = 0;
  line = copy = work = NULL;
  narg = maxarg = 0;
  arg = NULL;

  echo_screen = 0;
  echo_log = 1;

  label_active = 0;
  labelstr = NULL;
  jump_skip = 0;
  ifthenelse_flag = 0;

  if (me == 0) {
    nfile = maxfile = 1;
    infiles = (FILE **) memory->smalloc(sizeof(FILE *),"input:infiles");
    infiles[0] = infile;
  } else infiles = NULL;
  variable = new Variable(muse);

  int iarg = 0;
  while (iarg < argc) {
      if (strcmp(argv[iarg], "-var") == 0 || strcmp(argv[iarg], "-v") == 0) {
          int jarg = iarg + 3;
          while (jarg < argc && argv[jarg][0] != '-') jarg++;
          variable->set(argv[iarg + 1], jarg - iarg - 2, &argv[iarg + 2]);
          iarg = jarg;
      }
      else if (strcmp(argv[iarg], "-echo") == 0 ||
          strcmp(argv[iarg], "-e") == 0) {
          narg = 1;
          char** tmp = arg;        // trick echo() into using argv instead of arg
          arg = &argv[iarg + 1];
          echo();
          arg = tmp;
          iarg += 2;
      }
      else iarg++;
  }
}

/* ---------------------------------------------------------------------- */

Input::~Input()
{
  // don't free command and arg strings
  // they just point to other allocated memory

  memory->sfree(line);
  memory->sfree(copy);
  memory->sfree(work);
  if (labelstr) delete [] labelstr;
  memory->sfree(arg);
  memory->sfree(infiles);
  delete variable;
}

/* ----------------------------------------------------------------------
   process all input from infile
   infile = stdin or file if command-line arg "-in" was used
------------------------------------------------------------------------- */

void Input::file()
{
  int m,n;

  while (1) {

    // read a line from input script
    // n = length of line including str terminator, 0 if end of file
    // if line ends in continuation char '&', concatenate next line

    if (me == 0) {
      m = 0;
      while (1) {
        if (maxline-m < 2) reallocate(line,maxline,0);
        if (fgets(&line[m],maxline-m,infile) == NULL) {
          if (m) n = strlen(line) + 1;
          else n = 0;
          break;
        }
        m = strlen(line);
        if (line[m-1] != '\n') continue;

        m--;
        while (m >= 0 && isspace(line[m])) m--;
        if (m < 0 || line[m] != '&') {
          line[m+1] = '\0';
          n = m+2;
          break;
        }
      }
    }

    // bcast the line
    // if n = 0, end-of-file
    // error if label_active is set, since label wasn't encountered
    // if original input file, code is done
    // else go back to previous input file

    MPI_Bcast(&n,1,MPI_INT,0,world);
    if (n == 0) {
      if (label_active) error->all(FLERR,"Label wasn't found in input script");
      if (me == 0) {
        if (infile != stdin) {
          fclose(infile);
          infile = NULL;
        }
        nfile--;
      }
      MPI_Bcast(&nfile,1,MPI_INT,0,world);
      if (nfile == 0) break;
      if (me == 0) infile = infiles[nfile-1];
      continue;
    }

    if (n > maxline) reallocate(line,maxline,n);
    MPI_Bcast(line,n,MPI_CHAR,0,world);

    // echo the command unless scanning for label

    if (me == 0 && label_active == 0) {
      if (echo_screen && screen) fprintf(screen,"%s\n",line);
      if (echo_log && logfile) fprintf(logfile,"%s\n",line);
    }

    // parse the line
    // if no command, skip to next line in input script

    parse();
    if (command == NULL) continue;

    // if scanning for label, skip command unless it's a label command

    if (label_active && strcmp(command,"label") != 0) continue;

    // execute the command

    if (execute_command()) {
      char *str = new char[maxline+32];
      sprintf(str,"Unknown command: %s",line);
      error->all(FLERR,str);
    }
  }
}

/* ----------------------------------------------------------------------
   process all input from filename
   called from library interface
------------------------------------------------------------------------- */

void Input::file(const char *filename)
{
  // error if another nested file still open, should not be possible
  // open new filename and set infile, infiles[0], nfile
  // call to file() will close filename and decrement nfile

  if (me == 0) {
    if (nfile > 1)
      error->one(FLERR,"Invalid use of library file() function");

    if (infile && infile != stdin) fclose(infile);
    infile = fopen(filename,"r");
    if (infile == NULL) {
      char str[128];
      sprintf(str,"Cannot open input script %s",filename);
      error->one(FLERR,str);
    }
    infiles[0] = infile;
    nfile = 1;
  }

  file();
}

/* ----------------------------------------------------------------------
   copy command in single to line, parse and execute it
   return command name to caller
------------------------------------------------------------------------- */

char *Input::one(const char *single)
{
  int n = strlen(single) + 1;
  if (n > maxline) reallocate(line,maxline,n);
  strcpy(line,single);

  // echo the command unless scanning for label

  if (me == 0 && label_active == 0) {
    if (echo_screen && screen) fprintf(screen,"%s\n",line);
    if (echo_log && logfile) fprintf(logfile,"%s\n",line);
  }

  // parse the line
  // if no command, just return NULL

  parse();
  if (command == NULL) return NULL;

  // if scanning for label, skip command unless it's a label command

  if (label_active && strcmp(command,"label") != 0) return NULL;

  // execute the command and return its name

  if (execute_command()) {
    char *str = new char[maxline+32];
    sprintf(str,"Unknown command: %s",line);
    error->all(FLERR,str);
  }

  return command;
}

/* ----------------------------------------------------------------------
   parse copy of command line by inserting string terminators
   strip comment = all chars from # on
   replace all $ via variable substitution
   command = first word
   narg = # of args
   arg[] = individual args
   treat text between single/double quotes as one arg
------------------------------------------------------------------------- */

void Input::parse()
{
  // duplicate line into copy string to break into words

  int n = strlen(line) + 1;
  if (n > maxcopy) reallocate(copy,maxcopy,n);
  strcpy(copy,line);

  // strip any # comment by replacing it with 0
  // do not strip # inside single/double quotes

  char quote = '\0';
  char *ptr = copy;
  while (*ptr) {
    if (*ptr == '#' && !quote) {
      *ptr = '\0';
      break;
    }
    if (*ptr == quote) quote = '\0';
    else if (*ptr == '"' || *ptr == '\'') quote = *ptr;
    ptr++;
  }

  // perform $ variable substitution (print changes)
  // except if searching for a label since earlier variable may not be defined

  if (!label_active) substitute(copy,work,maxcopy,maxwork,1);

  // command = 1st arg in copy string

  char *next;
  command = nextword(copy,&next);
  if (command == NULL) return;

  // point arg[] at each subsequent arg in copy string
  // nextword() inserts string terminators into copy string to delimit args
  // nextword() treats text between single/double quotes as one arg

  narg = 0;
  ptr = next;
  while (ptr) {
    if (narg == maxarg) {
      maxarg += DELTA;
      arg = (char **) memory->srealloc(arg,maxarg*sizeof(char *),"input:arg");
    }
    arg[narg] = nextword(ptr,&next);
    if (!arg[narg]) break;
    narg++;
    ptr = next;
  }
}

/* ----------------------------------------------------------------------
   find next word in str
   insert 0 at end of word
   ignore leading whitespace
   treat text between single/double quotes as one arg
   matching quote must be followed by whitespace char if not end of string
   strip quotes from returned word
   return ptr to start of word
   return next = ptr after word or NULL if word ended with 0
   return NULL if no word in string
------------------------------------------------------------------------- */

char *Input::nextword(char *str, char **next)
{
  char *start,*stop;

  start = &str[strspn(str," \t\n\v\f\r")];
  if (*start == '\0') return NULL;

  if (*start == '"' || *start == '\'') {
    stop = strchr(&start[1],*start);
    if (!stop) error->all(FLERR,"Unbalanced quotes in input line");
    if (stop[1] && !isspace(stop[1]))
      error->all(FLERR,"Input line quote not followed by whitespace");
    start++;
  } else stop = &start[strcspn(start," \t\n\v\f\r")];

  if (*stop == '\0') *next = NULL;
  else *next = stop+1;
  *stop = '\0';
  return start;
}


void Input::substitute(char*& str, char*& str2, int& max, int& max2, int flag)
{
    // use str2 as scratch space to expand str, then copy back to str
    // reallocate str and str2 as necessary
    // do not replace $ inside single/double quotes
    // var = pts at variable name, ended by NULL
    //   if $ is followed by '{', trailing '}' becomes NULL
    //   else $x becomes x followed by NULL
    // beyond = points to text following variable

    int i, n, paren_count;
    char immediate[256];
    char* var, * value, * beyond;
    char quote = '\0';
    char* ptr = str;

    n = strlen(str) + 1;
    if (n > max2) reallocate(str2, max2, n);
    *str2 = '\0';
    char* ptr2 = str2;

    while (*ptr) {
        // variable substitution

        if (*ptr == '$' && !quote) {

            // value = ptr to expanded variable
            // variable name between curly braces, e.g. ${a}

            if (*(ptr + 1) == '{') {
                var = ptr + 2;
                i = 0;

                while (var[i] != '\0' && var[i] != '}') i++;

                if (var[i] == '\0') error->one(FLERR, "Invalid variable name");
                var[i] = '\0';
                beyond = ptr + strlen(var) + 3;
                value = variable->retrieve(var);

                // immediate variable between parenthesis, e.g. $(1/2)

            }
            else if (*(ptr + 1) == '(') {
                var = ptr + 2;
                paren_count = 0;
                i = 0;

                while (var[i] != '\0' && !(var[i] == ')' && paren_count == 0)) {
                    switch (var[i]) {
                    case '(': paren_count++; break;
                    case ')': paren_count--; break;
                    default:;
                    }
                    i++;
                }

                if (var[i] == '\0') error->one(FLERR, "Invalid immediate variable");
                var[i] = '\0';
                beyond = ptr + strlen(var) + 3;
                sprintf(immediate, "%.20g", variable->compute_equal(var));
                value = immediate;

                // single character variable name, e.g. $a

            }
            else {
                var = ptr;
                var[0] = var[1];
                var[1] = '\0';
                beyond = ptr + 2;
                value = variable->retrieve(var);
            }

            if (value == NULL) error->one(FLERR, "Substitution for illegal variable");

            // check if storage in str2 needs to be expanded
            // re-initialize ptr and ptr2 to the point beyond the variable.

            n = strlen(str2) + strlen(value) + strlen(beyond) + 1;
            if (n > max2) reallocate(str2, max2, n);
            strcat(str2, value);
            ptr2 = str2 + strlen(str2);
            ptr = beyond;

            // output substitution progress if requested

            if (flag && me == 0 && label_active == 0) {
                if (echo_screen && screen) fprintf(screen, "%s%s\n", str2, beyond);
                if (echo_log && logfile) fprintf(logfile, "%s%s\n", str2, beyond);
            }

            continue;
        }

        if (*ptr == quote) quote = '\0';
        else if (*ptr == '"' || *ptr == '\'') quote = *ptr;

        // copy current character into str2

        *ptr2++ = *ptr++;
        *ptr2 = '\0';
    }

    // set length of input str to length of work str2
    // copy work string back to input str

    if (max2 > max) reallocate(str, max, max2);
    strcpy(str, str2);
}


/* ----------------------------------------------------------------------
   rellocate a string
   if n > 0: set max >= n in increments of DELTALINE
   if n = 0: just increment max by DELTALINE
------------------------------------------------------------------------- */

void Input::reallocate(char *&str, int &max, int n)
{
  if (n) {
    while (n > max) max += DELTALINE;
  } else max += DELTALINE;

  str = (char *) memory->srealloc(str,max*sizeof(char),"input:str");
}


int Input::execute_command()
{
    int flag = 1;

    if (!strcmp(command, "echo")) echo();
    else if (!strcmp(command, "shell")) shell();
    else if (!strcmp(command, "print")) print();
    else if (!strcmp(command, "label")) label();
    else if (!strcmp(command, "variable")) variable_command();
    else if (!strcmp(command, "if")) ifthenelse();
    else if (!strcmp(command, "next")) next_command();
    else if (!strcmp(command, "jump")) jump();

    else flag = 0;

    // return if command was listed above

    if (flag) return 0;

    // check if command is added via style.h


    if (0) return 0;      // dummy line to enable else-if macro expansion
#define COMMAND_CLASS
#define CommandStyle(key,Class)         \
  else if (strcmp(command,#key) == 0) { \
    Class cmd_obj(muse);                \
    cmd_obj.command(narg,arg);          \
    flag = 1;                           \
  }
#include "style_command.h"
#undef CommandStyle
#undef COMMAND_CLASS

    if (flag) return 0;

    // unrecognized command

    return -1;
}



/* ---------------------------------------------------------------------- */

void Input::echo()
{
  if (narg != 1) error->all(FLERR,"Illegal echo command");

  if (strcmp(arg[0],"none") == 0) {
    echo_screen = 0;
    echo_log = 0;
  } else if (strcmp(arg[0],"screen") == 0) {
    echo_screen = 1;
    echo_log = 0;
  } else if (strcmp(arg[0],"log") == 0) {
    echo_screen = 0;
    echo_log = 1;
  } else if (strcmp(arg[0],"both") == 0) {
    echo_screen = 1;
    echo_log = 1;
  } else error->all(FLERR,"Illegal echo command");
}


/* ---------------------------------------------------------------------- */

void Input::ifthenelse()
{
    if (narg < 3) error->all(FLERR, "Illegal if command");

    // substitute for variables in Boolean expression for "if"
    // in case expression was enclosed in quotes
    // must substitute on copy of arg else will step on subsequent args

    int n = strlen(arg[0]) + 1;
    if (n > maxline) reallocate(line, maxline, n);
    strcpy(line, arg[0]);
    substitute(line, work, maxline, maxwork, 0);

    // evaluate Boolean expression for "if"

    double btest = variable->evaluate_boolean(line);

    // bound "then" commands

    if (strcmp(arg[1], "then") != 0) error->all(FLERR, "Illegal if command");

    int first = 2;
    int iarg = first;
    while (iarg < narg &&
        (strcmp(arg[iarg], "elif") != 0 && strcmp(arg[iarg], "else") != 0))
        iarg++;
    int last = iarg - 1;

    // execute "then" commands
    // make copies of all arg string commands
    // required because re-parsing a command via one() will wipe out args

    if (btest != 0.0) {
        int ncommands = last - first + 1;
        if (ncommands <= 0) error->all(FLERR, "Illegal if command");

        char** commands = new char* [ncommands];
        ncommands = 0;
        for (int i = first; i <= last; i++) {
            int n = strlen(arg[i]) + 1;
            if (n == 1) error->all(FLERR, "Illegal if command");
            commands[ncommands] = new char[n];
            strcpy(commands[ncommands], arg[i]);
            ncommands++;
        }

        ifthenelse_flag = 1;
        for (int i = 0; i < ncommands; i++) one(commands[i]);
        ifthenelse_flag = 0;

        for (int i = 0; i < ncommands; i++) delete[] commands[i];
        delete[] commands;

        return;
    }

    // done if no "elif" or "else"

    if (iarg == narg) return;

    // check "elif" or "else" until find commands to execute
    // substitute for variables and evaluate Boolean expression for "elif"
    // must substitute on copy of arg else will step on subsequent args
    // bound and execute "elif" or "else" commands

    while (1) {
        if (iarg + 2 > narg) error->all(FLERR, "Illegal if command");
        if (strcmp(arg[iarg], "elif") == 0) {
            n = strlen(arg[iarg + 1]) + 1;
            if (n > maxline) reallocate(line, maxline, n);
            strcpy(line, arg[iarg + 1]);
            substitute(line, work, maxline, maxwork, 0);
            btest = variable->evaluate_boolean(line);
            first = iarg + 2;
        }
        else {
            btest = 1.0;
            first = iarg + 1;
        }

        iarg = first;
        while (iarg < narg &&
            (strcmp(arg[iarg], "elif") != 0 && strcmp(arg[iarg], "else") != 0))
            iarg++;
        last = iarg - 1;

        if (btest == 0.0) continue;

        int ncommands = last - first + 1;
        if (ncommands <= 0) error->all(FLERR, "Illegal if command");

        char** commands = new char* [ncommands];
        ncommands = 0;
        for (int i = first; i <= last; i++) {
            int n = strlen(arg[i]) + 1;
            if (n == 1) error->all(FLERR, "Illegal if command");
            commands[ncommands] = new char[n];
            strcpy(commands[ncommands], arg[i]);
            ncommands++;
        }

        // execute the list of commands

        ifthenelse_flag = 1;
        for (int i = 0; i < ncommands; i++) one(commands[i]);
        ifthenelse_flag = 0;

        // clean up

        for (int i = 0; i < ncommands; i++) delete[] commands[i];
        delete[] commands;

        return;
    }
}


/* ---------------------------------------------------------------------- */

void Input::include()
{
  if (narg != 1) error->all(FLERR,"Illegal include command");

  // do not allow include inside an if command
  // NOTE: this check will fail if a 2nd if command was inside the if command
  //       and came before the include

  if (ifthenelse_flag)
    error->all(FLERR,"Cannot use include command within an if command");

  if (me == 0) {
    if (nfile == maxfile) {
      maxfile++;
      infiles = (FILE **)
        memory->srealloc(infiles,maxfile*sizeof(FILE *),"input:infiles");
    }
    infile = fopen(arg[0],"r");
    if (infile == NULL) {
      char str[128];
      sprintf(str,"Cannot open input script %s",arg[0]);
      error->one(FLERR,str);
    }
    infiles[nfile++] = infile;
  }
}

/* ---------------------------------------------------------------------- */

void Input::jump()
{
  if (narg < 1 || narg > 2) error->all(FLERR,"Illegal jump command");

  if (jump_skip) {
    jump_skip = 0;
    return;
  }

  if (me == 0) {
    if (strcmp(arg[0],"SELF") == 0) rewind(infile);
    else {
      if (infile && infile != stdin) fclose(infile);
      infile = fopen(arg[0],"r");
      if (infile == NULL) {
        char str[128];
        sprintf(str,"Cannot open input script %s",arg[0]);
        error->one(FLERR,str);
      }
      infiles[nfile-1] = infile;
    }
  }

  if (narg == 2) {
    label_active = 1;
    if (labelstr) delete [] labelstr;
    int n = strlen(arg[1]) + 1;
    labelstr = new char[n];
    strcpy(labelstr,arg[1]);
  }
}

/* ---------------------------------------------------------------------- */

void Input::label()
{
  if (narg != 1) error->all(FLERR,"Illegal label command");
  if (label_active && strcmp(labelstr,arg[0]) == 0) label_active = 0;
}

/* ---------------------------------------------------------------------- */


void Input::next_command()
{
    if (variable->next(narg, arg)) jump_skip = 1;
}

/* ---------------------------------------------------------------------- */


/* ---------------------------------------------------------------------- */

void Input::print()
{
  if (narg < 1) error->all(FLERR,"Illegal print command");

  // copy 1st arg back into line (copy is being used)
  // check maxline since arg[0] could have been exanded by variables
  // substitute for $ variables (no printing) and print arg

  int n = strlen(arg[0]) + 1;
  if (n > maxline) reallocate(line,maxline,n);
  strcpy(line,arg[0]);
  substitute(line,work,maxline,maxwork,0);

  // parse optional args

  FILE *fp = NULL;
  int screenflag = 1;

  int iarg = 1;
  while (iarg < narg) {
    if (strcmp(arg[iarg],"file") == 0 || strcmp(arg[iarg],"append") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal print command");
      if (me == 0) {
        if (strcmp(arg[iarg],"file") == 0) fp = fopen(arg[iarg+1],"w");
        else fp = fopen(arg[iarg+1],"a");
        if (fp == NULL) {
          char str[128];
          sprintf(str,"Cannot open print file %s",arg[iarg+1]);
          error->one(FLERR,str);
        }
      }
      iarg += 2;
    } else if (strcmp(arg[iarg],"screen") == 0) {
      if (iarg+2 > narg) error->all(FLERR,"Illegal print command");
      if (strcmp(arg[iarg+1],"yes") == 0) screenflag = 1;
      else if (strcmp(arg[iarg+1],"no") == 0) screenflag = 0;
      else error->all(FLERR,"Illegal print command");
      iarg += 2;
    } else error->all(FLERR,"Illegal print command");
  }

  if (me == 0) {
    if (screenflag && screen) fprintf(screen,"%s\n",line);
    if (screenflag && logfile) fprintf(logfile,"%s\n",line);
    if (fp) {
      fprintf(fp,"%s\n",line);
      fclose(fp);
    }
  }
}

/* ---------------------------------------------------------------------- */

void Input::quit()
{
  if (narg) error->all(FLERR,"Illegal quit command");
  error->done();
}

/* ---------------------------------------------------------------------- */

void Input::shell()
{
  if (narg < 1) error->all(FLERR,"Illegal shell command");

  if (strcmp(arg[0],"cd") == 0) {
    if (narg != 2) error->all(FLERR,"Illegal shell cd command");
    chdir(arg[1]);

  } else if (strcmp(arg[0],"mkdir") == 0) {
    if (narg < 2) error->all(FLERR,"Illegal shell mkdir command");
    if (me == 0)
      for (int i = 1; i < narg; i++) {
#if defined(_WIN32)
        _mkdir(arg[i]);
#else
        mkdir(arg[i], S_IRWXU | S_IRGRP | S_IXGRP);
#endif
      }

  } else if (strcmp(arg[0],"mv") == 0) {
    if (narg != 3) error->all(FLERR,"Illegal shell mv command");
    if (me == 0) rename(arg[1],arg[2]);

  } else if (strcmp(arg[0],"rm") == 0) {
    if (narg < 2) error->all(FLERR,"Illegal shell rm command");
    if (me == 0)
      for (int i = 1; i < narg; i++) unlink(arg[i]);

  } else if (strcmp(arg[0],"rmdir") == 0) {
    if (narg < 2) error->all(FLERR,"Illegal shell rmdir command");
    if (me == 0)
      for (int i = 1; i < narg; i++) rmdir(arg[i]);

  } else if (strcmp(arg[0],"putenv") == 0) {
    if (narg < 2) error->all(FLERR,"Illegal shell putenv command");
    for (int i = 1; i < narg; i++) {
      char *ptr = strdup(arg[i]);
#ifdef _WIN32
      if (ptr != NULL) _putenv(ptr);
#else
      if (ptr != NULL) putenv(ptr);
#endif
    }

  // use work string to concat args back into one string separated by spaces
  // invoke string in shell via system()

  } else {
    int n = 0;
    for (int i = 0; i < narg; i++) n += strlen(arg[i]) + 1;
    if (n > maxwork) reallocate(work,maxwork,n);

    strcpy(work,arg[0]);
    for (int i = 1; i < narg; i++) {
      strcat(work," ");
      strcat(work,arg[i]);
    }

    if (me == 0) system(work);
  }
}

/* ---------------------------------------------------------------------- */
void Input::variable_command()
{
    variable->set(narg, arg);
}

/* ----------------------------------------------------------------------
   read a floating point value from a string
   generate an error if not a legitimate floating point value
   called by various commands to check validity of their arguments
------------------------------------------------------------------------- */

double Input::numeric(const char* file, int line, const char* str)
{
    if (!str)
        error->all(file, line, "Expected floating point parameter "
            "in input script or data file");
    int n = strlen(str);
    if (n == 0)
        error->all(file, line, "Expected floating point parameter "
            "in input script or data file");

    for (int i = 0; i < n; i++) {
        if (isdigit(str[i])) continue;
        if (str[i] == '-' || str[i] == '+' || str[i] == '.') continue;
        if (str[i] == 'e' || str[i] == 'E') continue;
        error->all(file, line, "Expected floating point parameter "
            "in input script or data file");
    }

    return atof(str);
}

/* ----------------------------------------------------------------------
   read an integer value from a string
   generate an error if not a legitimate integer value
   called by various commands to check validity of their arguments
------------------------------------------------------------------------- */

int Input::inumeric(const char* file, int line, char* str)
{
    if (!str)
        error->all(file, line,
            "Expected integer parameter in input script or data file");
    int n = strlen(str);
    if (n == 0)
        error->all(file, line,
            "Expected integer parameter in input script or data file");

    for (int i = 0; i < n; i++) {
        if (isdigit(str[i]) || str[i] == '-' || str[i] == '+') continue;
        error->all(file, line,
            "Expected integer parameter in input script or data file");
    }

    return atoi(str);
}


/* ----------------------------------------------------------------------
   compute bounds implied by numeric str with a possible wildcard asterik
   1 = lower bound, nmax = upper bound
   5 possibilities:
     (1) i = i to i, (2) * = nmin to nmax,
     (3) i* = i to nmax, (4) *j = nmin to j, (5) i*j = i to j
   return nlo,nhi
------------------------------------------------------------------------- */

void Input::bounds(char *str, int nmax, int &nlo, int &nhi, int nmin)
{
  char *ptr = strchr(str,'*');

  if (ptr == NULL) {
    nlo = nhi = atoi(str);
  } else if (strlen(str) == 1) {
    nlo = nmin;
    nhi = nmax;
  } else if (ptr == str) {
    nlo = nmin;
    nhi = atoi(ptr+1);
  } else if (strlen(ptr+1) == 0) {
    nlo = atoi(str);
    nhi = nmax;
  } else {
    nlo = atoi(str);
    nhi = atoi(ptr+1);
  }

  if (nlo < nmin || nhi > nmax || nlo > nhi)
    error->all(FLERR,"Numeric index is out of bounds");
}

/* ----------------------------------------------------------------------
   count and return words in a single line
   make copy of line before using strtok so as not to change line
   trim anything from '#' onward
------------------------------------------------------------------------- */

int Input::count_words(char *line)
{
  int n = strlen(line) + 1;
  char *copy = (char *) memory->smalloc(n*sizeof(char),"copy");
  strcpy(copy,line);

  char *ptr;
  if ((ptr = strchr(copy,'#'))) *ptr = '\0';

  if (strtok(copy," \t\n\r\f") == NULL) {
    memory->sfree(copy);
    return 0;
  }
  n = 1;
  while (strtok(NULL," \t\n\r\f")) n++;

  memory->sfree(copy);
  return n;
}


