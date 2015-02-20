#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include <map>
#include "ped_tree.h"
#include "ped_agent.h"
#include <xmmintrin.h>
#include <time.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include <set>
#include <pthread.h>
#include <semaphore.h>

namespace Ped{
  class Tagent;
  class Ttree;
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ, TEST,OPENCL};
  

  class Model
  {
  public:
    double total_opencl_time;
    void setup(std::vector<Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads);
    void tick();
    const std::vector<Tagent*> getAgents() const;

    static void* threaded_tick(void* data);
    static void* threaded_tickMain(void* data);

    void whereToGoVec(std::vector<Tagent*> agents);
    void goVec(int i);
    void normalize(__m128 *SSEx, __m128 *SSEy, __m128 *SSEz, __m128 *SSEwx, __m128 *SSEwy, __m128 *SSEwz, __m128 *SSEr, float *reachedArr, int i);
    void calc_diff(__m128 *SSEx, __m128 *SSEy, __m128 *SSEz, __m128 SSEwx, __m128 SSEwy, __m128 SSEwz);
    void updateAgents(int i);

    // Vectorization variables
    float* px;
    float* py;
    float* pz;

    float* wx;
    float* wy;
    float* wz;

    float* agents_positions;

    float* rArr;
    float* reachedArr;

    // openCL variables
    
    cl_device_id device_id;
    cl_context context;
    cl_command_queue command_queue;
    cl_mem memobjx;
    cl_mem memobjy;
    cl_mem memobjwx;
    cl_mem memobjwy;
    cl_mem memobjrArr;
    cl_mem memobjReachedArr;
    cl_program program;
    cl_kernel kernel;
    cl_platform_id platform_id;
    cl_uint ret_num_devices; 
    cl_uint ret_num_platforms;
    cl_int ret;
 
    FILE *fp;
    char *source_str;
    size_t source_size;

    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////

    // Updates the treehash, which maps each agent to the current tree node that contains it
    void setResponsibleTree(Ped::Ttree *tree, const Ped::Tagent *agent);

    // Adds an agent to the tree structure
    void placeAgent(const Ped::Tagent *a);

    // Cleans up the tree and restructures it. Worth calling every now and then.
    void cleanup();
    ~Model();
    ////////////
    /// END NEW
    ///////////////////////////////////////////////
  private:
    IMPLEMENTATION implementation;
    std::vector<Tagent*> agents;
    int number_of_threads;
    
    struct parameters {
      Ped::Ttree* myTree;
      int numAgents;
      int treeID;
      Model* model;
    };
    struct parameters* Params;
    
    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////
    void doSafeMovement( Ped::Tagent *agent);
    // The maximum quadtree depth
    static const int treeDepth = 10;    

    // Keeps track of the positions of each agent
    Ped::Ttree *tree;

    // Maps the agent to the tree node containing it. Convenience data structure
    // in order to update the tree in case the agent moves.
    std::map<const Ped::Tagent*, Ped::Ttree*> *treehash;

    // Returns the set of neighboring agents for the specified position
    set<const Ped::Tagent*> getNeighbors(int x, int y, int dist) const;
    void getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int d) const;

    set<pthread_t> threadSet;

    sem_t tickSem;

    ////////////
    /// END NEW
    ///////////////////////////////////////////////
  };

}
#endif
