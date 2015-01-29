#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <iostream>
#include <pthread.h>
#include <stdio.h>

struct parameters {
  int start;
  int end;
};

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION choice)
{
  agents = agentsInScenario;
  implementation = choice;
  cout << choice;
}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

static void* Ped::Model::threaded_tick(void* parameters){
  struct parameters* params = (struct parameters*) parameters;
  
  int start = params->start;
  int end = params->end;
  std::vector<Tagent*> agents =  Ped::Model::getAgents();
  cout << start;
  cout << "heheheheheh\n";
  for(int i = start; start < end; i++) {
    agents[i]->whereToGo();
    agents[i]->go();
  }
  pthread_exit(NULL);
}

void Ped::Model::tick()
{
  std::vector<Tagent*> agents = Ped::Model::getAgents();
  int length = agents.size();
  switch(this->implementation) {
  case SEQ:
    for(int i = 0; i < length; i++) {
      agents[i]->whereToGo();
      agents[i]->go();
    }
    break;
  case OMP:
#pragma omp parallel
    {
#pragma omp for
      for(int i = 0; i < length; i++) {
      agents[i]->whereToGo();
      agents[i]->go();
    }
    }
    break;
    case PTHREAD:
      int number_of_threads = 2;
      pthread_t threads[number_of_threads];
      int d_agents = length / number_of_threads;
      struct parameters p[number_of_threads];
      cout << "innan for\n";
      for(int i = 0; i < number_of_threads; i++) {
         p[i].start= i*d_agents;
         p[i].end = (i + 1)*d_agents - 1;
         if(pthread_create(&threads[1],NULL,this.threaded_tick,(void*)&p[i])) {
            perror("thread crash\n");
         }
    }
      pthread_exit(NULL);
    }
}
