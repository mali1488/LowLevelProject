#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <iostream>
#include <pthread.h>
#include <stdio.h>

struct parameters {
  int start;
  int end;
  std::vector<Ped::Tagent*> agents;
};

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads)
{
  agents = agentsInScenario;
  implementation = choice;
  number_of_threads = numThreads;
}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

void* Ped::Model::threaded_tick(void* parameters){
  struct parameters* params = (struct parameters*) parameters;
  
  int start = params->start;
  int end = params->end;
  std::vector<Ped::Tagent*> agents = params->agents;
  for(int i = start; i <= end; i++) {
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
    { // TODO: I have no idea why this is needed (Yet! http://stackoverflow.com/questions/5685471/error-jump-to-case-label)
    for(int i = 0; i < length; i++) {
      agents[i]->whereToGo();
      agents[i]->go();
    }
    break;
    }
  case OMP:
    {
#pragma omp parallel num_threads(this->number_of_threads)
    {
#pragma omp for 
      for(int i = 0; i < length; i++) {
	agents[i]->whereToGo();
	agents[i]->go();
      }
    }
    break;
    }
  case PTHREAD:
    {
    int number_of_threads = this->number_of_threads;
    pthread_t threads[number_of_threads];
    int d_agents = length / number_of_threads;
    struct parameters p[number_of_threads];
    for(int i = 0; i < number_of_threads; i++){
      p[i] = *(new struct parameters());
    }
    
    for(int i = 0; i < number_of_threads; i++) {
      p[i].start= i*d_agents;
      p[i].end = (i + 1)*d_agents - 1;
      p[i].agents = agents;
      if(pthread_create(&threads[i],NULL,&(threaded_tick),(void*)&p[i])) {
	perror("thread crash\n");
      }
    }
    
    for(int i = 0; i < number_of_threads; i++){
      pthread_join(threads[i], NULL);
    }
    //pthread_join(threads[0],NULL);
    //pthread_join(threads[1],NULL);
    break;
    }
  default:
    {
    //pthread_exit(NULL);
    }
  }
}
