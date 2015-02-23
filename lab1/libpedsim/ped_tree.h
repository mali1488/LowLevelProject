//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#ifndef _ped_tree_h_
#define _ped_tree_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <set>
#include <list>
#include <pthread.h>

#include "ped_model.h"
#include "ped_agent.h"

#include <semaphore.h>

using namespace std;


namespace Ped {
    class Tagent;
    class Model;

    class LIBEXPORT Ttree {
      friend class Ped::Model;

    public:
      typedef struct parameters {
	Model* model;
	std::vector<Ped::Ttree*> *workLoad;
	//Ped::Ttree *tree;
	sem_t semaphore;
	sem_t mainSem;
	int idx;
      } *ThreadParams;

      Ttree(Ped::Ttree *root,std::map<const Ped::Tagent*, Ped::Ttree*> *treehash, int depth, int maxDepth, double x, double y, double w, double h);
      Ttree(Ped::Ttree *root,std::map<const Ped::Tagent*, Ped::Ttree*> *treehash, int depth, int maxDepth, double x, double y, double w, double h, std::map<const Ped::Ttree*, ThreadParams> *threadhash);
        virtual ~Ttree();

        virtual void clear();
        void getLeaves(std::vector<Ped::Ttree*> *ta);

        virtual void addAgent(const Ped::Tagent *a);
        virtual void moveAgent(const Ped::Tagent *a);
        //virtual bool moveAgent(Ped::Tagent *a, std::vector<Ped::Ttree*> *tree, std::pair<int,int> *pos);
        virtual bool removeAgent(const Ped::Tagent *a);

        virtual set<const Ped::Tagent*> getAgents() const;
        virtual void getAgents(list<const Ped::Tagent*>& outputList) const;

        virtual bool intersects(double px, double py, double pr) const;

        double getx() const { return x; };
        double gety() const { return y; };
        double getw() const { return w; };
        double geth() const { return h; };

        int getdepth() const { return depth; };

	void toString();
	
	bool moveCheck(const Ped::Tagent *a, int x, int y);
	bool dangerControl(const Ped::Tagent *a, int dist);

        typedef struct lockedAgents {
	  pthread_mutex_t lock; // = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
	  bool agentCAS;
	  set<const Ped::Tagent*> agentSet;
        } *LockedAgents;

    protected:
        virtual void addChildren();
        Ttree* getChildByPosition(double x, double y);
	int cut();
    protected:
	std::map<const Ped::Tagent*, Ped::Ttree*> *treehash;
	std::map<const Ped::Ttree*, ThreadParams> *threadhash;
        //set<const Ped::Tagent*> agents;	// set and not vector, since we need to delete elements from the middle very often
                                        // set and not list, since deletion is based on pointer (search O(log n) instead of O(n)).
        LockedAgents agents;

        bool isleaf;
        double x;
        double y;
        double w;
        double h;
        int depth;
	int maxDepth;

        Ttree *tree1;
        Ttree *tree2;
        Ttree *tree3;
        Ttree *tree4;
	Ttree *root;
    };
}

#endif
