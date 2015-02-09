#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"

namespace Ped{
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ, TEST};

  class Model
  {
  public:
    void setup(std::vector<Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads);
    void tick();
    const std::vector<Tagent*> getAgents() const;
    static void* threaded_tick(void* data);
    static void* threaded_tickMain(void* data);

    float* px;
    float* py;
    float* pz;

    float* wx;
    float* wy;
    float* wz;

    float* wfx;
    float* wfy;
    float* wfz;

    float* agents_positions;
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
