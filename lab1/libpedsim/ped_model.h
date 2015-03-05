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
#include <dispatch/dispatch.h>
#else
#include <CL/cl.h>
#include <semaphore.h>
#endif

#include <set>
#include <pthread.h>


namespace Ped{
  class Tagent;
  class Ttree;
  
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ, TEST, OPENCL, COLLISIONSEQ, COLLISIONPTHREAD, HEATMAP};

  class Model
  {
  public:
    double total_opencl_time;
    void setup(std::vector<Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads, bool heatmapFlag);
    void tick();
    const std::vector<Tagent*> getAgents() const;

    static void* threaded_tick(void* data);
    static void* threaded_tick_collision(void* data);

    ////////////
    /// NEW
    ///////////////////////////////////////////////
    int const * const * getHeatmap() const;
    int getHeatmapSize() const;
    ////////////
    /// END NEW
    ///////////////////////////////////////////////

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

    cl_mem memobjHeatmap;
    cl_mem memobjScaleHeatmap;
    cl_mem memobjBlurHeatmap;

    cl_program program;

    cl_kernel createHeatmapkernel;
    cl_kernel fadeHeatmapkernel;
    cl_kernel scalekernel;
    cl_kernel kernel;
    cl_kernel blurkernel;

    cl_platform_id platform_id;
    cl_uint ret_num_devices; 
    cl_uint ret_num_platforms;
    cl_int ret;

    cl_event blurEvent;
    cl_event fadeEvent;
    cl_event scaleEvent;
    cl_event createheatmapEvent;
    FILE *fp;
    char *source_str;
    size_t source_size;

    double total_time_blur;
    double total_time_scale;
    double total_time_createheatmap;
    double total_time_fade;
    cl_ulong time_start;
    cl_ulong time_end;

    time_t total_write_time;
    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////

    // Updates the treehash, which maps each agent to the current tree node that contains it
    void setResponsibleTree(Ped::Ttree *tree,  Ped::Tagent *agent);

    // Adds an agent to the tree structure
    void placeAgent( Ped::Tagent *a);
    void naiveBalance();

    int *agentCounter;
    void setAgentCounter(int idx, int value);

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
    pthread_t *threads;
    int tickcounter;

    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////
    //#define WIDTH 800
    #define HEIGHT 600
    #define WIDTH 832
    //#define HEIGHT 624

    #define SIZE 1024
    #define CELLSIZE 5
    #define SCALED_SIZE SIZE*CELLSIZE

    #define SCALED_WIDTH WIDTH*CELLSIZE
    #define SCALED_HEIGHT HEIGHT*CELLSIZE

    int ** heatmap;
    int * heatMapContogious;
    int * scaledHeatMapContogious;
    int * rowSize;
    int * xDesired;
    int * yDesired;
    int * blurHeatMapContigious;
    // The scaled heatmap that fits to the view
    int ** scaled_heatmap;
    int * scaledRowSize;

    // The final heatmap: blurred and scaled to fit the view
    int ** blurred_heatmap;
    
    void setupHeatmapSeq();
    void updateHeatmapSeq();

    void setupHeatmapPar();
    void updateHeatmapPar();
    ////////////
    /// END NEW
    ///////////////////////////////////////////////

    struct parameters {
      Model* model;
      std::vector<Ped::Ttree*> *workLoad;
      std::list<Ped::Tagent*> *leavers;
      //Ped::Ttree *tree;
      #ifdef __APPLE__
         dispatch_semaphore_t semaphore;
         dispatch_semaphore_t mainSem;
      #else
         sem_t semaphore;
         sem_t mainSem;
      #endif
      int idx;
    };
    struct parameters** Params;

    struct oldparams {
      int start;
      int end;
      std::vector<Ped::Tagent*> agents;
    };
    struct oldparams** Oldparams;

    #ifdef __APPLE__
       dispatch_semaphore_t testSem;
    #else
       sem_t testSem;
    #endif

    
    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////
    void doSafeMovement( Ped::Tagent *agent);
    void doSafeMovementThreaded( Ped::Tagent *agent, std::list<Ped::Tagent*> *leavers, std::vector<Ped::Ttree*> *trees, Ped::Ttree *currentTree);
    // The maximum quadtree depth
    static const int treeDepth = 10;    

    // Keeps track of the positions of each agent
    Ped::Ttree *tree;

    // Maps the agent to the tree node containing it. Convenience data structure
    // in order to update the tree in case the agent moves.
    std::map< Ped::Tagent*, Ped::Ttree*> *treehash;

    // Returns the set of neighboring agents for the specified position
    set< Ped::Tagent*> getNeighbors(int x, int y, int dist) ;
    void getNeighbors(list< Ped::Tagent*>& neighborList, int x, int y, int d) ;
    set< Ped::Tagent*> getNeighbors(int x, int y, int dist, Ped::Ttree *startTree) ;
    void getNeighbors(list< Ped::Tagent*>& neighborList, int x, int y, int d, Ped::Ttree *startTree) ;

    set<pthread_t> threadSet;

    ////////////
    /// END NEW
    ///////////////////////////////////////////////
  };
  
  

}
#endif
