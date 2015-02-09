#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <math.h>

#include <string.h>
#include <stdlib.h>

#include <xmmintrin.h>

struct parameters;

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads)
{
  agents = agentsInScenario;
  implementation = choice;
  number_of_threads = numThreads;
  std::vector<Tagent*> agents = Ped::Model::getAgents();  
  int length = agents.size();  
  if(choice == PTHREAD){
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
  if(choice == VECTOR || choice == TEST) {
    /*
      px = new double[length];
      py = new double[length];
      wx = new double[length];
      wy = new double[length];
    */

    px = (float *) malloc(sizeof(float) * length);
    py = (float *) malloc(sizeof(float) * length);
    wx = (float *) malloc(sizeof(float) * length);
    wy = (float *) malloc(sizeof(float) * length);

    for(int i = 0; i<length; i++) {
      px[i] = agents[i]->position.x;
      py[i] = agents[i]->position.y;
      wx[i] = agents[i]->waypointForce.x;
      wx[i] = agents[i]->waypointForce.y;
    } 
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
  case VECTOR: 
    {
      //#pragma omp simd
      for (int i = 0; i < length; i++) {
	agents[i]->whereToGo();
      }
      //      #pragma omp simd
      /*
      float* x_arr = (float *) malloc(sizeof(float) * length);
      float* y_arr = (float *) malloc(sizeof(float) * length);
      
      float* wx_arr = (float *) malloc(sizeof(float) * length);
      float* wy_arr = (float *) malloc(sizeof(float) * length);

      memcpy(x_arr, px, sizeof(float)*length);
      memcpy(y_arr, py, sizeof(float)*length);
      
      memcpy(wx_arr, wx, sizeof(float)*length);
      memcpy(wy_arr, wy, sizeof(float)*length);
      */
      for(int i = 0; i < length; i += 4){
	__m128 SSEwx = _mm_load_ps(&wx[i]);
	__m128 vTemp = _mm_load_ps(&px[i]);
	__m128 v = _mm_add_ps(vTemp,SSEwx);
	_mm_store_ps(&px[i],v);

	__m128 SSEwy = _mm_load_ps(&wy[i]);
	vTemp = _mm_load_ps(&py[i]);
	v = _mm_add_ps(vTemp,SSEwy);
	_mm_store_ps(&py[i],v);	
      }
      break;
    }
  case TEST:
    {
    for (int i = 0; i < length; ++i) {
      agents[i]->whereToGo();
    }

    for (int i = 0; i < length; i++) {
      px[i] = px[i] + wx[i];
      py[i] = py[i] + wy[i];
    }
    break;
  }
  default:
    {
      //pthread_exit(NULL);
    }
  }
}
