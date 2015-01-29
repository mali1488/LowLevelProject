Low Level Parallel Programming VT 2015
======================================

I. Crowd Simulation Project
---------------------------

Provides the source code for crowd simulation and a demo application.

Directories:
  libpedsim   -- the crowd simulation library
  demo        -- a demo application using the simulation library


II. Requirements
----------------
 Qt 4             -- tested with Qt 4.8
 g++         


III. Building
-------------
(You may need to add the library -lcudart at the LIBS line in demo/src/src.pr if you have nvidia toolkit installed)

Run:
  make

If the binary runs into problem finding libpedsim, type the following into your terminal: export LD_LIBRARY_PATH=.

IV. Running Demo Application
----------------------------
Run:
  cd demo
  ./demo






  
  
  



