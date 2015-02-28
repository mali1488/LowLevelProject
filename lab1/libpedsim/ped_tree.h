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

      Ttree(Ped::Ttree *root,std::map< Ped::Tagent*, Ped::Ttree*> *treehash, int depth, int maxDepth, double x, double y, double w, double h);
        virtual ~Ttree();
        short owner;

        virtual void clear();
        void getLeaves(std::vector<Ped::Ttree*> *ta);

        virtual void addAgent( Ped::Tagent *a);
        virtual void moveAgent( Ped::Tagent *a);
	virtual bool dangerZone(Ped::Tagent *a, std::vector<Ped::Ttree*> *tree);
        virtual bool removeAgent( Ped::Tagent *a);
        std::vector<Ped::Ttree*> getNeighbor();

        virtual set< Ped::Tagent*> getAgents() ;
        virtual void getAgents(list< Ped::Tagent*>& outputList) ;

        virtual bool intersects(double px, double py, double pr) ;

        double getx()  { return x; };
        double gety()  { return y; };
        double getw()  { return w; };
        double geth()  { return h; };

        int getdepth()  { return depth; };

	void toString();
	
	bool moveCheck( Ped::Tagent *a, int x, int y);
	bool dangerControl( Ped::Tagent *a, int dist);

        typedef struct lockedAgents {
	  pthread_mutex_t lock; // = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
	  bool agentCAS;
	  set< Ped::Tagent*> agentSet;
        } *LockedAgents;

    protected:
        virtual void addChildren();
        Ttree* getChildByPosition(double x, double y);
	int cut();
    protected:
	std::map< Ped::Tagent*, Ped::Ttree*> *treehash;
        //set< Ped::Tagent*> agents;	// set and not vector, since we need to delete elements from the middle very often
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
