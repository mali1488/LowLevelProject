

#ifndef _timer_h_
#define _timer_h_

#include <QTimer>
#include "ped_model.h"
#include "MainWindow.h"
// Driver for updating the world
class Timer : public QObject{
  Q_OBJECT
  

public:
  Timer(Ped::Model &model,MainWindow &window, int time);
  Timer(Ped::Model &model,MainWindow &window, int time, bool heatmapFlag);
  Timer() = delete;
  void setTickLimit(int limit);
  void qtTimerTick();
  void busyTick();
public slots:
  void tick();

private:
  Ped::Model &model;
  MainWindow &window;
  const int time;
  int maxTicks;
  bool heatmapFlag = false;
};









#endif
