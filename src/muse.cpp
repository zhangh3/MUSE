/* ----------------------------------------------------------------------
   MUSE - MUltibody System dynamics Engine
   https://github.com/zhangh3/MUSE

   Zhang He, zhanghecalt@163.com
   Science and Technology on Space Physics Laboratory, Beijing

   This software is distributed under the GNU General Public License.
   Copyright (c) 2023 Zhang He. All rights reserved.
------------------------------------------------------------------------- */

#include "mpi.h"
#include "string.h"
#include "muse.h"
#include "memory.h"
#include "input.h"
#include "version.h"
#include "ensemble.h"
#include "error.h"
#include "MUSEsystem.h"
#include "joint.h"
#include "body.h"
#include "timer.h"
#include "modify.h"
#include "output.h"


#define DELTA 5


using namespace MUSE_NS;
MUSE::MUSE(int narg, char **arg, MPI_Comm communicator)
{
	MPI_Comm_rank(world, &me);

	screen = stdout;
	logfile = NULL;
	infile = NULL;
	output = NULL;

	int inflag = 0;


	nBodies  = maxBodies  = 0;
	nJoints  = maxJoints  = 0;

	body   = NULL;
	joint  = NULL;
	system = NULL;

  memory = new Memory(this);
  error = new Error(this);
  ensemble = new Ensemble(this,communicator);
  system = new System(this);

  ///////creat
  modify = new Modify(this);
  output = new Output(this);
  timer = new Timer(this);


  if (me == 0) {
	  if (screen) fprintf(screen, "================================================\n");
	  if (screen) fprintf(screen, "   __       __  __    __   ______   ________  \n");
	  if (screen) fprintf(screen, "  |  \\     /  \\|  \\  |  \\ /      \\ |        \\ \n");
	  if (screen) fprintf(screen, "  | ::\\   /  ::| ::  | ::|  ::::::\\| :::::::: \n");
	  if (screen) fprintf(screen, "  | :::\\ /  :::| ::  | ::| ::___\\::| ::__     \n");
	  if (screen) fprintf(screen, "  | ::::\\  ::::| ::  | :: \\::    \\ | ::  \\    \n");
	  if (screen) fprintf(screen, "  | ::\\:: :: ::| ::  | :: _\\::::::\\| :::::    \n");
	  if (screen) fprintf(screen, "  | :: \\:::| ::| ::__/ ::|  \\__| ::| ::_____  \n");
	  if (screen) fprintf(screen, "  | ::  \\: | :: \\::    :: \\::    ::| ::     \\ \n");
	  if (screen) fprintf(screen, "   \\::      \\::  \\::::::   \\::::::  \\:::::::: \n");
	  if (screen) fprintf(screen, "\n================================================\n");
	  if (screen) fprintf(screen, "VERSION: (%s)\n\n", MUSE_VERSION);
	  if (logfile) fprintf(logfile, "VERSION: (%s)\n\n", MUSE_VERSION);
  }

  int iarg = 1;
  while (iarg < narg) {
	  if (strcmp(arg[iarg], "-in") == 0 ||
		  strcmp(arg[iarg], "-i") == 0) {
		  if (iarg + 2 > narg)
			  error->all(FLERR, "Invalid command-line argument");
		  inflag = iarg + 1;
		  iarg += 2;
	  }
	  else error->all(FLERR, "Invalid command-line argument");
  }

  if (me == 0) {
	  if (inflag == 0) infile = stdin;
	  else infile = fopen(arg[inflag], "r");
	  if (infile == NULL) {
		  char str[128];
		  sprintf(str, "Cannot open input script %s", arg[inflag]);
		  error->one(FLERR, str);
	  }
  }


  input = new Input(this, narg, arg);



}

MUSE::~MUSE()
{
	delete ensemble;
	delete error;
	delete memory;
	delete input;
	delete timer;
	delete modify;
	delete system;
	//delete comm;
	delete output;

	for (int i = 0; i < nBodies; i++) delete body[i];
	memory->sfree(body);

	for (int i = 0; i < nJoints; i++) delete joint[i];
	memory->sfree(joint);

	if (me == 0) {
		if (screen && screen != stdout) fclose(screen);
		//infile 已在file()内关闭但是为了避免自定义main（）中不调用file（），增加此行判断
		if (infile && infile != stdin) fclose(infile); 
		if (logfile) fclose(logfile);
	}
	
}

void MUSE_NS::MUSE::init()
{
	//comm->init();//FIXME
	modify->init();               
	output->init();
	timer->init();
}

int MUSE::add_Body(char *name)
{
  int ibody;

  for (ibody = 0; ibody < nBodies; ibody++)
	  if (strcmp(name,body[ibody]->name) == 0) break;

  if (ibody < nBodies) {
      error->all(FLERR,"Bodies with same name!");
  } else {
    if (nBodies == maxBodies) {
      maxBodies += DELTA;
      body = (Body **) memory->srealloc(body,maxBodies*sizeof(Body *),"muse:body");
    }
  }
  body[ibody] = new Body(this);
  body[ibody]->set_Name(name);
  body[ibody]->IDinMuse = ibody;



  nBodies++;
  return ibody;
}

int MUSE::add_Joint(char* name)
{
	int ijoint;

	for (ijoint = 0; ijoint < nJoints; ijoint++)
		if (strcmp(name, joint[ijoint]->name) == 0) break;

	if (ijoint < nJoints) {
		error->all(FLERR, "Joints with same name!");
	}
	else {
		if (nJoints == maxJoints) {
			maxJoints += DELTA;
			joint = (Joint**)memory->srealloc(joint, maxJoints * sizeof(Joint*), "muse:joint");
		}
	}
	joint[ijoint] = new Joint(this);
	joint[ijoint]->set_Name(name);
	joint[ijoint]->IDinMuse = ijoint;
	nJoints++;
	return ijoint;
}
