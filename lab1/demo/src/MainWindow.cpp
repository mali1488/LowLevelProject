#include "MainWindow.h"

#include <QtGui>
#include <QBrush>

#include <iostream>
MainWindow::MainWindow(const Ped::Model &model) : model(model)
{
  
  // The Window 
  graphicsView = new QGraphicsView();

  setCentralWidget(graphicsView);

  // A surface for managing a large number of 2D graphical items
  scene = new QGraphicsScene(QRect(0,0,800,600),this);
  
  // Connect
  graphicsView->setScene(scene);


  // Paint on surface
  scene->setBackgroundBrush(Qt::black);
  //graphicsscene->setItemIndexMethod(QGraphicsScene::NoIndex);
  

  for (int x=0; x<=800; x+=cellsizePixel)
  {
    scene->addLine(x,0,x,800, QPen(Qt::gray));
  }

// Now add the horizontal lines, paint them green
  for (int y=0; y<=800; y+=cellsizePixel)
  {
    scene->addLine(0,y,800,y, QPen(Qt::gray));
  }

  // Create viewAgents with references to the position of the model counterparts
  auto &agents = model.getAgents();
  for(auto agent : agents)
    {
      viewAgents.push_back(new ViewAgent(agent,scene));
    }

  paint();
  graphicsView->show(); // Redundant? 
}

 
void MainWindow::paint() {
  //std::cout << "painting" << endl;

  
  for(auto a : viewAgents)
    {
      a->paint();
    }
}

  
int MainWindow::cellToPixel(int val)
{
  //cout << "conv" << val << " " << val*cellsizePixel << endl;
  return val*cellsizePixel;
}
