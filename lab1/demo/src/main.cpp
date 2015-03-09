//#include "ped_agent.h"
#include "ped_model.h"
#include "MainWindow.h"
#include "ParseScenario.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QApplication>
#include <QTimer>
#include <thread>

#include <unistd.h>

#include "Timer.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstring>

#include <iostream>
#include <fstream>

//#define SIMULATION_STEPS 1000000
#define SIMULATION_STEPS 5

int main(int argc, char*argv[]) { 
  Ped::Model model;
  bool timing_mode = 0;
  bool collision = false;
  int i = 1;
  bool enableHeatmap = false;
  QString scenefile = "scenario.xml";
  Ped::IMPLEMENTATION choice = Ped::SEQ;
  int number_of_threads = std::thread::hardware_concurrency();
  // Argument handling
  while(i < argc)
    {
      if(strcmp(argv[i], "--seq") == 0){
	choice = Ped::SEQ;
	cout << "executing sequential\n";
      }
      if(strcmp(argv[i], "--omp") == 0){
	choice = Ped::OMP;
	cout << "executing omp\n";
      }
      if(strcmp(argv[i], "--pthread") == 0){
	choice = Ped::PTHREAD;
	cout << "executing pthread\n";
      }
      if(strcmp(argv[i], "--vector") == 0){
	choice = Ped::VECTOR;
	cout << "vector" <<endl;
      }     
      if(strcmp(argv[i], "--test") == 0){
	choice = Ped::TEST;
	cout << "test" <<endl;
      }
      if(strcmp(argv[i], "--opencl") == 0){
	choice = Ped::OPENCL;
	cout << "opencl" <<endl;
      }
      if(strcmp(argv[i], "--collision") == 0){
	collision = true;
	cout << "collision on" <<endl;
      }

      if(strcmp(argv[i], "--heatmap") == 0){
	enableHeatmap = true;
	cout << "heatmap enabled" <<endl;
      }

      if(strncmp(argv[i], "--threads=", 10) == 0){
	number_of_threads = atoi(&argv[i][10]);
	cout << "Executing simulation with " << number_of_threads << " threads. \n";
      }
      
      if(strcmp(argv[i], "--timing-mode") == 0){
	cout << "Timing mode on\n";
	timing_mode = true;
      }

      i+=1;
    }

  if(choice == Ped::PTHREAD && collision) {
    choice = Ped::COLLISIONPTHREAD;
  }

  if(choice == Ped::SEQ && collision) {
    choice = Ped::COLLISIONSEQ;
  }

  ParseScenario parser(scenefile);
  model.setup(parser.getAgents(),choice,number_of_threads, enableHeatmap);

  QApplication app(argc, argv);
  
  MainWindow mainwindow(model, enableHeatmap);
  
  // TODO: default 100
  const int delay_ms = 16;
  Timer *timer;
#define TICK_LIMIT 1000
#define AS_FAST_AS_POSSIBLE 0
  if(timing_mode)
    {
      std::cout << "Timing with TICK_LIMIT = " << TICK_LIMIT << "\n";
      timer = new Timer(model,mainwindow,AS_FAST_AS_POSSIBLE, enableHeatmap);
      timer->setTickLimit(TICK_LIMIT);
    }
  else
    {
      timer = new Timer(model,mainwindow,delay_ms, enableHeatmap);
      mainwindow.show();

    }
  cout << "Demo setup complete, running ..." << endl;
  int retval = 0;
  std::chrono::time_point<std::chrono::system_clock> start,stop;
  start = std::chrono::system_clock::now();

  // If timing mode, just go as fast as possible without delays or graphical updates
  if(timing_mode)
  {
    timer->busyTick();
  }
  else
  {
    timer->qtTimerTick();
    retval = app.exec();
  }

  stop = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = stop-start;
  cout << "Time: " << elapsed_seconds.count() << " seconds." << endl;

  cout << "Done" << endl;

  if(choice == Ped::OPENCL) {
    printf("\nExecution time of opencl in seconds = %0.3f seconds\n", (model.total_opencl_time / 100000000.0) );
  }

  ofstream file;
  file.open("../data.txt", std::ios::app);

  file << elapsed_seconds.count() << ", ";

  delete (timer);
  return retval;
}
