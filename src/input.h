/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */


#ifndef MUSE_INPUT_H
#define MUSE_INPUT_H

#include "stdio.h"
#include "pointers.h"

namespace MUSE_NS {

    class Input : protected Pointers {
    public:
        int narg;                    // # of command args
        char** arg;                  // parsed args for command
        class Variable* variable;    // defined variables

        Input(class MUSE*, int, char**);
        ~Input();
        void file();                   // process all input
        void file(const char*);       // process an input script
        char* one(const char*);       // process a single command
        void substitute(char*&, char*&, int&, int&, int);
        // substitute for variables in a string
        int expand_args(int, char**, int, char**&);  // expand args due to wildcard

        double numeric(const char*, int, const char*);    // arg checking
        int inumeric(const char*, int, char*);
        int bnumeric(const char*, int, char*);
        void bounds(char*, int, int&, int&, int nmin = 1);
        int count_words(char*);

    private:
        int me;                      // proc ID
        char* command;               // ptr to current command
        int maxarg;                  // max # of args in arg
        char* line, * copy, * work;      // input line & copy and work string
        int maxline, maxcopy, maxwork; // max lengths of char strings
        int echo_screen;             // 0 = no, 1 = yes
        int echo_log;                // 0 = no, 1 = yes
        int nfile, maxfile;           // current # and max # of open input files
        int label_active;            // 0 = no label, 1 = looking for label
        char* labelstr;              // label string being looked for
        int jump_skip;               // 1 if skipping next jump, 0 otherwise
        int ifthenelse_flag;         // 1 if executing commands inside an if-then-else

        FILE** infiles;              // list of open input files

        void parse();                          // parse an input text line
        char* nextword(char*, char**);       // find next word in string with quotes
        void reallocate(char*&, int&, int);  // reallocate a char string
        int execute_command();                 // execute a single command

        void clear();                // input script commands
        void echo();
        void ifthenelse();
        void include();
        void jump();
        void label();
        void log();
        void next_command();
        void partition();
        void print();
        void quit();
        void shell();
        void variable_command();

        // SPARTA commands

        void boundary();
        void bound_modify();
        void collide_command();
        void collide_modify();
        void compute();
        void dimension();
        void dump();
        void dump_modify();
        void fix();
        void global();
        void group();
        void mixture();
        void package();
        void react_command();
        void react_modify();
        void region();
        void reset_timestep();
        void restart();
        void seed();
        void species();
        void stats();
        void stats_modify();
        void stats_style();
        void surf_collide();
        void surf_modify();
        void surf_react();
        void timestep();
        void uncompute();
        void undump();
        void unfix();
        void units();
        void weight();
    };

}

#endif

/* ERROR/WARNING messages:

*/
