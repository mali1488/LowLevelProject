#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"

namespace Ped{
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads);
    void tick();
    const std::vector<Tagent*> getAgents() const;
    static void* threaded_tick(void* data);
  private:
    
    IMPLEMENTATION implementation;
    std::vector<Tagent*> agents;
    int number_of_threads;
    
  };
}
#endif
