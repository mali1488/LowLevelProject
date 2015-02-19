//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#ifndef _ped_agent_h_
#define _ped_agent_h_ 1


// g++ does not understand cuda stuff. This makes it ignore them. (use this if you want)
#ifndef __CUDACC__
#define __device__ 
#define __host__
#endif

#include "ped_vector.h"
#include <vector>
#include <deque>

using namespace std;

namespace Ped {
  class Twaypoint;

  class Tagent {
  public:
    Tagent(int posX, int posY);
    Tagent(double posX, double posY);

    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////
    
    // Returns the desired positons coordinates
    int getDesiredX() const {return desiredPosition.x;}
    int getDesiredY() const {return desiredPosition.y;}

    // Sets the agent's position
    void setX(int x) {position.x = x;}
    void setY(int y) {position.y = y;}

    ////////////
    /// END NEW 
    ///////////////////////////////////////////////

    // Computes forces that determine the next position
    void whereToGo();

    // Update the position according to computed forces
    void go();

    const Tvector& getPosition() const { return position; }
    double getX() const { return position.x; };
    double getY() const { return position.y; };

    Twaypoint* getDestination() const { return destination; };
    Twaypoint* getLastDestination() const { return lastDestination; };
    Tvector getWaypointForce() const { return waypointForce; };

    void setWaypointForce(Tvector wp) { waypointForce = wp; };

    void setDestination(Twaypoint* wp) { destination = wp; };
    void setLastDestination(Twaypoint* wp) { lastDestination = wp; };

    void addWaypoint(Twaypoint* wp);
    bool removeWaypoint(const Twaypoint* wp);
    void clearWaypoints();
    void setWaypointBehavior(int mode) { waypointBehavior = mode; };

    deque<Twaypoint*> getWaypoints() { return waypoints;};

    bool reachedDestination() { return (destination == NULL); };
    
    // TODO Should be private
    Tvector position;    

    // TODO Should be private
    // The force towards the current destination
    Tvector waypointForce;

    void setNextDestination();

  private:
    Tagent() {};
    
    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////

    // The desired next position
    Tvector desiredPosition;

    ////////////
    /// END NEW
    ///////////////////////////////////////////////

    Twaypoint* destination;
    Twaypoint* lastDestination;

    deque<Twaypoint*> waypoints;
    int waypointBehavior;

   // Computes the forces that determine the next position
    void computeForces();
    
    // Interntal init function 
    void init(int posX, int posY);

    // Computing waypoint force and directions
    Tvector computeWaypointForce();
    Tvector computeDirection();
    Twaypoint* getNextDestination();
    Twaypoint* getNextWaypoint();
  };
}

#endif
