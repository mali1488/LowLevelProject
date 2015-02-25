//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
#include "ped_tree.h"

#include <cassert>
#include <cstddef>
#include <algorithm>
#include <pthread.h>

#include <iostream>

using namespace std;

pthread_mutexattr_t Attr;

#define ZONE_CONSTANT 10


/// Description: set intial values
/// \author  chgloor
/// \date    2012-01-28


Ped::Ttree::Ttree(Ped::Ttree *root,std::map< Ped::Tagent*, Ped::Ttree*> *treehash, int pdepth, int pmaxDepth, double px, double py, double pw, double ph) {
  // more initializations here. not really necessary to put them into the initializator list, too.
  this->root = root != NULL ? root: this;
  this->treehash = treehash;
  // this should be moved somewhere else
  pthread_mutexattr_init(&Attr);
  pthread_mutexattr_settype(&Attr, PTHREAD_MUTEX_RECURSIVE);
  isleaf = true;
  this->agents = new struct lockedAgents;
  this->agents->agentCAS = false;
  pthread_mutex_init(&(this->agents->lock), &Attr);    

  x = px;
  y = py;
  w = pw;
  h = ph;
  depth = pdepth;
  maxDepth = pmaxDepth;
  tree1 = NULL;
  tree2 = NULL;
  tree3 = NULL;
  tree4 = NULL;
};


/// Destructor. Deleted this node and all its children. If there are any agents left, they are removed first (not deleted).
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::~Ttree() {
  clear();
}


void Ped::Ttree::clear() {
  if(isleaf) {
    agents->agentSet.clear();
  }
  else {
    tree1->clear();
    delete tree1;
    tree2->clear();
    delete tree2;
    tree3->clear();
    delete tree3;
    tree4->clear();
    delete tree4;
    isleaf = true;
  }
}

bool treeCmp( Ped::Tagent *a,  Ped::Tagent *b) {
  return a->getX() == b->getX() && a->getY() == b->getY();
}

/// Adds an agent to the tree. Searches the right node and adds the agent there.
/// If there are too many agents at that node allready, a new child is created.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a The agent to add
void Ped::Ttree::addAgent( Ped::Tagent *a) {
  if (isleaf) {
    agents->agentSet.insert(a);
    //model->setResponsibleTree(this, a);
    (*treehash)[a] = this;
  }
  else {
    if ((a->getX() >= x+w/2) && (a->getY() >= y+h/2)){
      tree3->addAgent(a); // 3
      }
      else if ((a->getX() <= x+w/2) && (a->getY() <= y+h/2)) { // TODO: should not be else if (original code does not use else if here)?
	tree1->addAgent(a); // 1
      }
      else if ((a->getX() >= x+w/2) && (a->getY() <= y+h/2)) {
	tree2->addAgent(a); // 2
      }
      else if ((a->getX() <= x+w/2) && (a->getY() >= y+h/2)) {
	tree4->addAgent(a); // 4
      }
  }

  if (agents->agentSet.size() > 8 && depth < maxDepth) {
    isleaf = false;
    addChildren();
    while (!agents->agentSet.empty()) {
       Ped::Tagent *a = (*agents->agentSet.begin());
      if ((a->getX() >= x+w/2) && (a->getY() >= y+h/2)) { 
	tree3->addAgent(a); // 3
      }
      else if ((a->getX() <= x+w/2) && (a->getY() <= y+h/2)) { // TODO: should not be else if (original code does not use else if here)?
	tree1->addAgent(a); // 1
      }
      else if ((a->getX() >= x+w/2) && (a->getY() <= y+h/2)) {
	tree2->addAgent(a); // 2
      }
      else if ((a->getX() <= x+w/2) && (a->getY() >= y+h/2)) {
	tree4->addAgent(a); // 4
      }
      agents->agentSet.erase(a);
    }
  }
}


/// A little helper that adds child nodes to this node
/// \author  chgloor
/// \date    2012-01-28
void Ped::Ttree::addChildren() {
    tree1 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x, y, w/2, h/2);
    tree2 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x+w/2, y, w/2, h/2);
    tree3 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x+w/2, y+h/2, w/2, h/2);
    tree4 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x, y+h/2, w/2, h/2);
}


Ped::Ttree* Ped::Ttree::getChildByPosition(double xIn, double yIn) {
  if((xIn <= x+w/2) && (yIn <= y+h/2))
    return tree1;
  if((xIn >= x+w/2) && (yIn <= y+h/2))
    return tree2;
  if((xIn >= x+w/2) && (yIn >= y+h/2))
    return tree3;
  if((xIn <= x+w/2) && (yIn >= y+h/2))
    return tree4;

  // this should never happen
  return NULL;
}


/// Updates the tree structure if an agent moves. Removes the agent and places it again, if outside boundary.
/// If an this happens, this is O(log n), but O(1) otherwise.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a the agent to update
void Ped::Ttree::moveAgent( Ped::Tagent *a) {
  if ((a->getX() < x) || (a->getX() > (x+w)) || (a->getY() < y) || (a->getY() > (y+h))) {
    agents->agentSet.erase(a);    
    root->addAgent(a);
  }
}

bool Ped::Ttree::dangerZone(Ped::Tagent *a, std::vector<Ped::Ttree*> *trees) {
  int r = 1;// TODO: kanske kan använda ostrikt olikhet om r = 2
    for (std::vector<Ped::Ttree*>::iterator i = trees->begin(); i != trees->end(); ++i) {
        Ped::Ttree *t = (*i);
	/* Allow agents to stand on right-most and bottom-most border */
        if (((t->x + t->w) >= (a->getX()+r)) && ((t->x) < a->getX() - r) && ((t->y + t->h) >= (a->getY() + r)) && ((t->y) < (a->getY() - r))) { // TODO: if segfault, control corner cases! (Might need to change r?)
            return false;
        }
    }
    return true;
}

bool Ped::Ttree::moveAgent(Ped::Tagent *a, std::vector<Ped::Ttree*> *trees, std::pair<int,int> *pos) {
  int r = 1;// TODO: kanske kan använda ostrikt olikhet om r = 2
    for (std::vector<Ped::Ttree*>::iterator i = trees->begin(); i != trees->end(); ++i) {
        Ped::Ttree *t = (*i);
	/* Allow agents to stand on right-most and bottom-most border */
        if (((t->x + t->w) >= (pos->first+r)) && ((t->x) < pos->first - r) && ((t->y + t->h) >= (pos->second + r)) && ((t->y) < (pos->second - r))) { // TODO: if segfault, control corner cases! (Might need to change r?)
            //std::cout << "TRUE!\n";
            return true;
        }
    }
    return false;
}



std::vector<Ped::Ttree*> Ped::Ttree::getNeighbor() {
    std::vector<Ped::Ttree*> ret;
    int pos;
    //Ped::Ttree* max;
    if(root->tree1 == this) pos = 1;
    if(root->tree2 == this) pos = 2;
    if(root->tree3 == this) pos = 3;
    if(root->tree4 == this) pos = 4;
    if (pos == 1 || pos == 4) {
        ret.push_back(root->tree2);
        ret.push_back(root->tree3);
        //max = root->tree2->getAgents().size() > root->tree3->getAgents().size() ? root->tree2 : root->tree3;
    } else {
        ret.push_back(root->tree1);
        ret.push_back(root->tree4);
        //max = root->tree1->getAgents().size() > root->tree4->getAgents().size() ? root->tree1 : root->tree4;
    }
    return ret;
}



bool Ped::Ttree::dangerControl( Ped::Tagent *a, int dist) {
  if ((a->getX() - dist < x) || (a->getX() + dist > (x+w)) || (a->getY() - dist < y) || (a->getY() + dist > (y+h))) {
    return true;
  } else {
    return false;
  }
}

bool Ped::Ttree::removeAgent( Ped::Tagent *a) {
  if(isleaf) {
    size_t removedCount = agents->agentSet.erase(a);
    return (removedCount > 0);
  }
  else {
    return getChildByPosition(a->getX(), a->getY())->removeAgent(a);
  }
}


/// Checks if this tree node has not enough agents in it to justify more child nodes. It does this by checking all
/// child nodes, too, recursively. If there are not enough children, it moves all the agents into this node, and deletes the child nodes.
/// \author  chgloor
/// \date    2012-01-28
/// \return  the number of agents in this and all child nodes.
int Ped::Ttree::cut() {
  if (isleaf) {
  int size = agents->agentSet.size();
  return size;
  }
	
  int count = 0;
  count += tree1->cut();
  count += tree2->cut();
  count += tree3->cut();
  count += tree4->cut();
  if (count < 5) {
    assert(tree1->isleaf == true);
    assert(tree2->isleaf == true);
    assert(tree3->isleaf == true);
    assert(tree4->isleaf == true);

    agents->agentSet.insert(tree1->agents->agentSet.begin(), tree1->agents->agentSet.end());
    agents->agentSet.insert(tree2->agents->agentSet.begin(), tree2->agents->agentSet.end());
    agents->agentSet.insert(tree3->agents->agentSet.begin(), tree3->agents->agentSet.end());
    agents->agentSet.insert(tree4->agents->agentSet.begin(), tree4->agents->agentSet.end());

    for (set< Ped::Tagent*>::iterator it = agents->agentSet.begin(); it != agents->agentSet.end(); ++it) {
       Tagent *a = (*it);
      (*treehash)[a] = this;
    }
    delete tree1;
    delete tree2;
    delete tree3;
    delete tree4;
  }
  return count;
}

void Ped::Ttree::getLeaves(std::vector<Ped::Ttree*> *ta) {
  if (isleaf) {
    ta->push_back(this);
    return;
  }
  tree1->getLeaves(ta);
  tree2->getLeaves(ta);
  tree3->getLeaves(ta);
  tree4->getLeaves(ta);
}

/// Returns the set of agents that is stored within this tree node
/// \author  chgloor
/// \date    2012-01-28
/// \return  The set of agents
/// \todo This might be not very efficient, since all childs are checked, too. And then the set (set of pointer, but still) is being copied around.
set< Ped::Tagent*> Ped::Ttree::getAgents()  {
  if (isleaf)
    return agents->agentSet; 
  set< Ped::Tagent*> ta;
  set< Ped::Tagent*> t1 = tree1->getAgents();
  set< Ped::Tagent*> t2 = tree2->getAgents();
  set< Ped::Tagent*> t3 = tree3->getAgents();
  set< Ped::Tagent*> t4 = tree4->getAgents();
  ta.insert(t1.begin(), t1.end());
  ta.insert(t2.begin(), t2.end());
  ta.insert(t3.begin(), t3.end());
  ta.insert(t4.begin(), t4.end());
  //std::cout << "ta.size: " << ta.size() << "\n";
  //std::cout << "ta in getAgents: " << *(ta.begin()) << "\n";
  return ta;
}

void Ped::Ttree::getAgents(list< Ped::Tagent*>& outputList)  {
  if(isleaf) {
    for (set< Ped::Tagent*>::iterator it = agents->agentSet.begin(); it != agents->agentSet.end(); ++it) {
       Ped::Tagent* currentAgent = (*it);
      outputList.push_back(currentAgent);
    }
  }
  else {
    tree1->getAgents(outputList);
    tree2->getAgents(outputList);
    tree3->getAgents(outputList);
    tree4->getAgents(outputList);
  }
}


/// Checks if a point x/y is within the space handled by the tree node, or within a given radius r
/// \author  chgloor
/// \date    2012-01-29
/// \return  true if the point is within the space
/// \param   px The x co-ordinate of the point
/// \param   py The y co-ordinate of the point
/// \param   pr The radius
// ORIGINAL VERSION:
bool Ped::Ttree::intersects(double px, double py, double pr)  {
  if (((px+pr) >= x) && ((px-pr) <= (x+w)) && ((py+pr) >= y) && ((py-pr) <= (y+h)))
    return true; // x+-r/y+-r is inside
  else
    return false;
    } 

// Experimental version:
/*
bool Ped::Ttree::intersects(double px, double py, double pr)  {
  if (((px-pr) >= x) && ((px+pr) <= (x+w)) && ((py-pr) >= y) && ((py+pr) <= (y+h)))
    return true; // x+-r/y+-r is inside
  else
    return false;
    } */

void Ped::Ttree::toString(){
  std::cout << "x :"<< x << ",y :" << y << ",h :"<< h << ", w:" << w << ", number agents: " << getAgents().size() << ", depth:" << depth << "\n";
}
