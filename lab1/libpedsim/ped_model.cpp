#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <iostream>
#include <pthread.h>
#include <stdio.h>

struct parameters;

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads)
{
  agents = agentsInScenario;
  implementation = choice;
  number_of_threads = numThreads;
  
  
  if(choice == PTHREAD){
    std::vector<Tagent*> agents = Ped::Model::getAgents();
    int length = agents.size();
    int delta = length / this->number_of_threads;
    int d_agents = length / number_of_threads; 
    this->params = new struct parameters* [number_of_threads];

    for(int i = 0; i < number_of_threads; i++){
      this->params[i] = new struct parameters();
    }
    
    for(int i = 0; i < number_of_threads; i++) {
      this->params[i]->start= i*d_agents;
      this->params[i]->end = (i + 1)*d_agents - 1;
      this->params[i]->agents = agents;
    }

    this->params[number_of_threads-1]->end = length-1;
  }
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

/* Garage hack */
void* Ped::Model::threaded_tickMain(void* parameters){
  struct parameters* params = (struct parameters*) parameters;
  
  int start = params->start;
  int end = params->end;
  std::vector<Ped::Tagent*> agents = params->agents;
  for(int i = start; i <= end; i++) {
    agents[i]->whereToGo();
    agents[i]->go();
  }
}

void Ped::Model::tick()
{
  std::vector<Tagent*> agents = Ped::Model::getAgents();
  int length = agents.size();
  int delta = length / this->number_of_threads;
  
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
    
    for(int i = 0; i < number_of_threads-1; i++) {
      if(pthread_create(&threads[i],NULL,&(threaded_tick),(void*)this->params[i])) {
	perror("thread crash\n");
      }
    }

    threaded_tickMain((void *) this->params[number_of_threads-1]);

    for(int i = 0; i < number_of_threads-1; i++){
      pthread_join(threads[i], NULL);
    }

    break;
    }
  default:
    {
    //pthread_exit(NULL);
    }
  }
}
