//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>

/*------------Constructors----------*/
Ped::Tagent::Tagent(int posX, int posY) {
  Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
  Ped::Tagent::init((int) round(posX), (int) round(posY));
}

void Ped::Tagent::init(int posX, int posY) {
  position.x = posX;
  position.y = posY;
  destination = NULL;
  lastDestination = NULL;
}
/*----------Where to go methods-----------------*/

void Ped::Tagent::whereToGo() {
  computeForces();
}

void Ped::Tagent::computeForces() {
  waypointForce = computeWaypointForce();
}

void Ped::Tagent::go() {
  Tvector moveForce = waypointForce;
  
  ////////////
  /// THIS IS NEW
  ///////////////////////////////////////////////

  desiredPosition.x = round(position.x + moveForce.x);
  desiredPosition.y = round(position.y + moveForce.y);

  ////////////
  /// NEW END
  ///////////////////////////////////////////////
}


Ped::Tvector Ped::Tagent::computeWaypointForce() {
  destination = getNextDestination();
  Tvector waypointDirection = computeDirection();
  Tvector force = waypointDirection.normalized();

  return force;
}

void Ped::Tagent::setNextDestination() {
  destination = getNextDestination();
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
  Ped::Twaypoint* nextDestination = NULL;
  if (destination != NULL) {
    nextDestination = destination; // agent hasn't arrived yet
  }
  else if (!waypoints.empty()) {
    nextDestination = getNextWaypoint();
  }
  return nextDestination;
}

Ped::Tvector Ped::Tagent::computeDirection() {
  if (destination == NULL) {
    std::cout << "TESTING\n";
    // Don't move, if nowhere to go
    return Ped::Tvector(0, 0, 0);
  }

  Tvector direction;
  bool reachesDestination = false; // if agent reaches destination in n
  if (lastDestination == NULL) {
    //std::cout << "TESTTEST\n";
    Twaypoint tempDestination(destination->getx(), destination->gety(), destination->getr());
    tempDestination.settype(Ped::Twaypoint::TYPE_POINT);
    direction = tempDestination.getForce(position.x, position.y, 0, 0, &reachesDestination);
  }
  else {
    direction = destination->getForce(position.x, position.y,
				      lastDestination->getx(),
				      lastDestination->gety(),
				      &reachesDestination);
  }

  if (reachesDestination == true) {
    // Circular waypoint chasing
    waypoints.push_back(destination);
    lastDestination = destination;
    destination = NULL;
  }

  return direction;
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
  waypoints.push_back(wp);
}

bool Ped::Tagent::removeWaypoint(const Twaypoint* wp) {
  if (destination == wp)
    destination = NULL;
  if (lastDestination == wp)
    lastDestination = NULL;
  
  bool removed = false;
  for (int i = waypoints.size(); i > 0; --i) {
    Twaypoint* currentWaypoint = waypoints.front();
    waypoints.pop_front();
    if (currentWaypoint != wp) {
      waypoints.push_back(currentWaypoint);
      removed = true;
    }
  }
  
  return removed;
}

Ped::Twaypoint* Ped::Tagent::getNextWaypoint() {
  Twaypoint* waypoint = waypoints.front();
  waypoints.pop_front();
  return waypoint;
}

void Ped::Tagent::clearWaypoints() {
  destination = NULL;
  lastDestination = NULL;
  
  for (int i = waypoints.size(); i > 0; --i) {
    waypoints.pop_front();
  }
}

