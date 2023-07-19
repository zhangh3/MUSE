# MUSE multiple-machine Makefile

SHELL = /bin/bash

# Definitions

ROOT =	MUSE
EXE =	$(ROOT)_$@
ARLIB =  libmuse_$@.a
SHLIB =	 libmuse_$@.so
ARLINK = libmuse.a
SHLINK = libmuse.so

OBJDIR =   Obj_$@
OBJSHDIR = Obj_shared_$@

SRC =	$(wildcard *.cpp)
INC =	$(wildcard *.h)
OBJ = 	$(SRC:.cpp=.o)

SRCLIB = $(filter-out main.cpp,$(SRC))
OBJLIB = $(filter-out main.o,$(OBJ))

# Command-line options for mode: exe (default), shexe, lib, shlib

mode = exe
objdir = $(OBJDIR)

ifeq ($(mode),shexe)
objdir = $(OBJSHDIR)
endif

ifeq ($(mode),lib)
objdir = $(OBJDIR)
endif

ifeq ($(mode),shlib)
objdir = $(OBJSHDIR)
endif

# Package variables

PACKAGE = fft kokkos

PACKALL = $(PACKAGE)

PACKAGEUC = $(shell echo $(PACKAGE) | tr a-z A-Z)

YESDIR = $(shell echo $(@:yes-%=%) | tr a-z A-Z)
NODIR  = $(shell echo $(@:no-%=%) | tr a-z A-Z)

# List of all targets

help:
	@echo ''
	@echo 'make clean-all           delete all object files'
	@echo 'make clean-machine       delete object files for one machine'
	@echo 'make mpi-stubs           build dummy MPI library in STUBS'
	@echo ''
	@echo 'make machine             build MUSE for machine'
	@echo 'make mode=lib machine    build MUSE as static lib for machine'
	@echo 'make mode=shlib machine  build MUSE as shared lib for machine'
	@echo 'make mode=shexe machine  build MUSE as shared exe for machine'
	@echo ''
	@echo 'machine is one of these from src/MAKE:'
	@echo ''
	@files="`ls MAKE/Makefile.*`"; \
	  for file in $$files; do head -1 $$file; done
	@echo ''

# Build MUSE in one of 4 modes
# exe =   exe with static compile in Obj_machine (default)
# shexe = exe with shared compile in Obj_shared_machine
# lib =   static lib in Obj_machine
# shlib = shared lib in Obj_shared_machine

.DEFAULT:
	@if [ $@ = "serial" -a ! -f STUBS/libmpi_stubs.a ]; \
	  then $(MAKE) mpi-stubs; fi
	@test -f MAKE/Makefile.$@
	@if [ ! -d $(objdir) ]; then mkdir $(objdir); fi
	@cp MAKE/Makefile.$@ $(objdir)/Makefile
	@if [ -f MAKE/Makefile.$@ ]; \
	  then cp MAKE/Makefile.$@ $(objdir)/Makefile; fi
ifeq ($(mode),exe)
	@cd $(objdir); \
	$(MAKE) $(MFLAGS) "OBJ = $(OBJ)" "INC = $(INC)" "SHFLAGS =" \
	  "EXE = ../$(EXE)" ../$(EXE)
endif
ifeq ($(mode),shexe)
	@cd $(objdir); \
	$(MAKE) $(MFLAGS) "OBJ = $(OBJ)" "INC = $(INC)" \
	  "EXE = ../$(EXE)" ../$(EXE)
endif
ifeq ($(mode),lib)
	@cd $(objdir); \
	$(MAKE) $(MFLAGS) "OBJ = $(OBJLIB)" "INC = $(INC)" "SHFLAGS =" \
	  "EXE = ../$(ARLIB)" lib
	@rm -f $(ARLINK)
	@ln -s $(ARLIB) $(ARLINK)
endif
ifeq ($(mode),shlib)
	@cd $(objdir); \
	$(MAKE) $(MFLAGS) "OBJ = $(OBJLIB)" "INC = $(INC)" \
	  "EXE = ../$(SHLIB)" shlib
	@rm -f $(SHLINK)
	@ln -s $(SHLIB) $(SHLINK)
endif

# Remove machine-specific object files

clean:
	@echo 'make clean-all           delete all object files'
	@echo 'make clean-machine       delete object files for one machine'

clean-all:
	rm -rf Obj_*
clean-%:
	rm -rf Obj_$(@:clean-%=%) Obj_shared_$(@:clean-%=%)

# Make MPI STUBS library

mpi-stubs:
	@cd STUBS; $(MAKE) clean; $(MAKE)

