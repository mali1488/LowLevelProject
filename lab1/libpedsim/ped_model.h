#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"
#include <xmmintrin.h>
#include <time.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif


namespace Ped{
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ, TEST,OPENCL};
  double total_time;

  class Model
  {
  public:
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

    float* lenArr;
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
    
  private:
    
    IMPLEMENTATION implementation;
    std::vector<Tagent*> agents;
    int number_of_threads;
    
    struct parameters {
      int start;
      int end;
      std::vector<Ped::Tagent*> agents;
    };
    struct parameters** params;
  };

  
}
#endif
