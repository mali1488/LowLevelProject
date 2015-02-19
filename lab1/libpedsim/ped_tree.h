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

#include "ped_model.h"
#include "ped_agent.h"

using namespace std;

namespace Ped {
    class Tagent;
    class Model;

    class LIBEXPORT Ttree {
      friend class Ped::Model;

    public:
      Ttree(Ped::Ttree *root,std::map<const Ped::Tagent*, Ped::Ttree*> *treehash, int depth, int maxDepth, double x, double y, double w, double h);
        virtual ~Ttree();

        virtual void clear();

        virtual void addAgent(const Ped::Tagent *a);
        virtual void moveAgent(const Ped::Tagent *a);
        virtual bool removeAgent(const Ped::Tagent *a);

        virtual set<const Ped::Tagent*> getAgents() const;
        virtual void getAgents(list<const Ped::Tagent*>& outputList) const;

        virtual bool intersects(double px, double py, double pr) const;

        double getx() const { return x; };
        double gety() const { return y; };
        double getw() const { return w; };
        double geth() const { return h; };

        int getdepth() const { return depth; };

    protected:
        virtual void addChildren();
        Ttree* getChildByPosition(double x, double y);
	int cut();
    protected:
	std::map<const Ped::Tagent*, Ped::Ttree*> *treehash;
        set<const Ped::Tagent*> agents;	// set and not vector, since we need to delete elements from the middle very often
                                        // set and not list, since deletion is based on pointer (search O(log n) instead of O(n)).

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
