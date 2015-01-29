//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#include "ParseScenario.h"
#include <string>

/// object constructor
/// \date    2011-01-03
ParseScenario::ParseScenario(QString filename) : QObject(0)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    return;
  }

  while (!file.atEnd())
  {
    QByteArray line = file.readLine();
    processXmlLine(line);
  }
}

vector<Ped::Tagent*> ParseScenario::getAgents() const
{
  return agents;
}

/// Called for each line in the file
void ParseScenario::processXmlLine(QByteArray dataLine)
{
  xmlReader.addData(dataLine);

  while (!xmlReader.atEnd())
  {
    xmlReader.readNext();
    if (xmlReader.isStartElement())
    {
      handleXmlStartElement();
    }
    else if (xmlReader.isEndElement())
    {
      handleXmlEndElement();
    }
  }
}

void ParseScenario::handleXmlStartElement()
{
  if (xmlReader.name() == "waypoint")
  {
    handleWaypoint();	
  }
  else if (xmlReader.name() == "agent")
  {
    handleAgent();
  }
  else if (xmlReader.name() == "addwaypoint")
  {
    handleAddWaypoint();
  }
  else
  {
    // nop, unknown, ignore
  }
}

void ParseScenario::handleXmlEndElement()
{
  if (xmlReader.name() == "agent") {
    Ped::Tagent *a;
    foreach (a, tempAgents)
    {
      agents.push_back(a);
    }
  }
}

void ParseScenario::handleWaypoint()
{
  QString id = readString("id");
  double x = readDouble("x");
  double y = readDouble("y");
  double r = readDouble("r");
  
  Ped::Twaypoint *w = new Ped::Twaypoint(x, y, r);
  waypoints[id] = w;
}

void ParseScenario::handleAgent()
{
  double x = readDouble("x");
  double y = readDouble("y");
  int n = readDouble("n");
  double dx = readDouble("dx");
  double dy = readDouble("dy");

  tempAgents.clear();
  for (int i = 0; i < n; ++i)
  {
    int xPos = x + qrand()/(RAND_MAX/dx) -dx/2;
    int yPos = y + qrand()/(RAND_MAX/dy) -dy/2;
    Ped::Tagent *a = new Ped::Tagent(xPos, yPos);
    tempAgents.push_back(a);
  }
}

void ParseScenario::handleAddWaypoint()
{
  QString id = readString("id");
  Ped::Tagent *a;
  foreach (a, tempAgents)
  {
    a->addWaypoint(waypoints[id]);
  }
}

double ParseScenario::readDouble(const QString &tag)
{
  return readString(tag).toDouble();
}

QString ParseScenario::readString(const QString &tag)
{
  return xmlReader.attributes().value(tag).toString();
}
