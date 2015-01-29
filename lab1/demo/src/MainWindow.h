#ifndef _mainwindow_h_
#define _mainwindow_h_

#include <QMainWindow>
#include <QGraphicsScene>

#include <vector>

#include "ped_model.h"
#include "ped_agent.h"
#include "ViewAgent.h"
class QGraphicsView;


class MainWindow : public QMainWindow {
  //Q_OBJECT
 public:
  MainWindow() = delete;
  MainWindow(const Ped::Model &model);
  void paint();
  static int cellToPixel(int val);
  static const int cellsizePixel = 5; // TODO: make configurable (zoom?)
 private:
//void paintAgent(const Ped::Tagent &agent);
  QGraphicsView *graphicsView;
  QGraphicsScene * scene;
  const Ped::Model &model;
std::vector<ViewAgent*> viewAgents;
};


#endif
