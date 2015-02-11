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
#include <smmintrin.h>

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
    px = (float *) malloc(sizeof(float) * length);
    py = (float *) malloc(sizeof(float) * length);
    pz = (float *) malloc(sizeof(float) * length);
    
    wx = (float *) malloc(sizeof(float) * length);
    wy = (float *) malloc(sizeof(float) * length);
    wz = (float *) calloc(length, sizeof(float));

    lenArr = (float *) malloc(sizeof(float) * length);
    
    wfx = (float *) malloc(sizeof(float) * length);
    wfy = (float *) malloc(sizeof(float) * length);
    wfz = (float *) malloc(sizeof(float) * length);
    
    for(int i = 0; i<length; i++) {
      px[i] = agents[i]->position.x;
      py[i] = agents[i]->position.y;
      pz[i] = agents[i]->position.z;
      //wx[i] = agents[i]->destination.x;
      //wy[i] = agents[i]->destination.y;
      //wz[i] = agents[i]->destination.z;
      wfx[i] = agents[i]->waypointForce.x;
      wfy[i] = agents[i]->waypointForce.y;
      wfz[i] = agents[i]->waypointForce.z;
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
      __m128 SSEx;
      __m128 SSEy;
      __m128 SSEz;

      __m128 SSEwx;
      __m128 SSEwy;
      __m128 SSEwz;

      __m128 temp;

      /* <wheretogo()>*/
      for(int i = 0; i < length; i++){
	agents[i]->setNextDestination();
	Twaypoint* tempDest = agents[i]->getDestination();
	Twaypoint* tempLastDest = agents[i]->getLastDestination();

	if(tempDest != NULL) {
	  wx[i] = tempDest->getx();
	  wy[i] = tempDest->gety();
	}

	/* Shrinking behaviour not caused by this */
	if (tempLastDest == NULL) {
	  bool reachesDestination = false;
	  //std::cout << "TESTTEST\n";
	  Twaypoint tempDestination(tempDest->getx(), tempDest->gety(), tempDest->getr());
	  tempDestination.settype(Ped::Twaypoint::TYPE_POINT);
	  Tvector direction = tempDestination.getForce(agents[i]->position.x, agents[i]->position.y, 0, 0, &reachesDestination);
	  agents[i]->setWaypointForce(direction);
	  
	  }


      }

      for (int i = 0; i < length; i += 4) {
	// SSEx will contain four first floats starting at px[i] ...
	SSEx = _mm_load_ps(&px[i]);
	SSEy = _mm_load_ps(&py[i]);
	SSEz = _mm_load_ps(&pz[i]);

	SSEwx = _mm_load_ps(&wx[i]);
	SSEwy = _mm_load_ps(&wy[i]);
	SSEwz = _mm_load_ps(&wz[i]);

/*
	SSEx =_mm_round_ps(SSEx, _MM_FROUND_TO_NEAREST_INT);
	SSEy = _mm_round_ps(SSEy, _MM_FROUND_TO_NEAREST_INT);
	SSEz =_mm_round_ps(SSEz, _MM_FROUND_TO_NEAREST_INT);
	SSEwx =_mm_round_ps(SSEwx, _MM_FROUND_TO_NEAREST_INT);
	SSEwy =_mm_round_ps(SSEwy, _MM_FROUND_TO_NEAREST_INT);
	SSEwz =_mm_round_ps(SSEwz, _MM_FROUND_TO_NEAREST_INT);
*/

	// diff (line 79 ped_waypoint)
	SSEx = _mm_sub_ps(SSEwx,SSEx);
	SSEy = _mm_sub_ps(SSEwy,SSEy);
	SSEz = _mm_sub_ps(SSEwz,SSEz);

	/*
	SSEx =_mm_round_ps(SSEx, _MM_FROUND_TO_NEAREST_INT);
	SSEy =_mm_round_ps(SSEy, _MM_FROUND_TO_NEAREST_INT);
	SSEz =_mm_round_ps(SSEz, _MM_FROUND_TO_NEAREST_INT);
	*/

	//TODO: some kind of reached-check here? See getForce.

	// normalization starting
	// x*x + y*y
	temp = _mm_add_ps(_mm_mul_ps(SSEx,SSEx),_mm_mul_ps(SSEy,SSEy));
	// temp + z*z
	temp = _mm_add_ps(temp,_mm_mul_ps(SSEz,SSEz));
	// sqrt(temp)
	temp = _mm_sqrt_ps(temp);

	temp = _mm_round_ps(temp, _MM_FROUND_TO_NEAREST_INT);

	_mm_store_ps(&lenArr[i], temp);
	
	// SSEx / temp (normalization, line 51 ped_vector)
	SSEwx = _mm_div_ps(SSEx,temp);
	SSEwy = _mm_div_ps(SSEy,temp);
	SSEwz = _mm_div_ps(SSEz,temp);
	// TODO check for zero division
	
	SSEwx =_mm_round_ps(SSEwx, _MM_FROUND_TO_NEAREST_INT);
	SSEwy=_mm_round_ps(SSEwy, _MM_FROUND_TO_NEAREST_INT);
        SSEwz=_mm_round_ps(SSEwz, _MM_FROUND_TO_NEAREST_INT);


	// Should be normalized here (corresponding to line 89 @ waypoint)

	// Store result back into array
	_mm_store_ps(&wx[i], SSEwx);
	_mm_store_ps(&wy[i], SSEwy);  
	_mm_store_ps(&wz[i], SSEwz);
	
      }
      /* </wheretogo()>*/
      
      /* <go()> */
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
      /* </go()> */

      

      // Update the agents with the new values
      for(int i = 0; i < length; i++){
	agents[i]->position.x = round(px[i]);
	agents[i]->position.y = round(py[i]);

	agents[i]->setWaypointForce(Ped::Tvector(wx[i], wy[i], wz[i]));

	//std::cout << "scherman x: " << wx[i] << " scherman y: " << wy[i] << "\n";

	Twaypoint* tempDest = agents[i]->getDestination();
	Twaypoint* tempLastDest = agents[i]->getLastDestination();

	if(lenArr[i] == 0) { //TODO: remove?
	  //std::cout << "ZERO!\n";
	  agents[i]->setWaypointForce(Ped::Tvector());
	}

	if(tempDest != NULL){
	  if(lenArr[i] < tempDest->getr()) { // TODO: weird behaviour
	    // Circular waypoint chasing
	    //std::cout << "STUDSA!\n";
	    deque<Twaypoint*> waypoints = agents[i]->getWaypoints();
	    agents[i]->addWaypoint(tempDest);
	    agents[i]->setLastDestination(tempDest);
	    agents[i]->setDestination(NULL);
	  }
	}

	/* TODO: should this be added?
	Twaypoint* tempDest = agents[i]->getDestination();
	tempDest->setx((float) wx[i]);
	tempDest->sety((float) wy[i]); */
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
