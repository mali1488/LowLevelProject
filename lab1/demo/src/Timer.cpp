#include "Timer.h"
#include <iostream>
#include <QApplication>
#include <thread>
using namespace std;
Timer::Timer(Ped::Model &model_,MainWindow &window_, int time_) : model(model_),window(window_),time(time_),maxTicks(-1)
{
 

}



void Timer::setTickLimit(int limit)
{
  this->maxTicks = limit;
}

void Timer::tick()
{
  model.tick();
  window.paint();
  if(maxTicks-- == 0)
    {
      QApplication::quit();
    }
}


void Timer::qtTimerTick()
{
  auto movetimer = new QTimer(); 
  QObject::connect(movetimer, SIGNAL(timeout()), this, SLOT(tick()));
  movetimer->start(time);
}


void Timer::busyTick()
{
  std::thread t([](Timer * t)
	   {
	     for(int i = 0; i < t->maxTicks; i++)
	     {
	       t->model.tick();
	     }
	   },this);
  t.join();
}
