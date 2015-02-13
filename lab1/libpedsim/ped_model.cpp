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

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

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
  if(choice == VECTOR || choice == TEST || choice == OPENCL) {
    px = (float *) malloc(sizeof(float) * length);
    py = (float *) malloc(sizeof(float) * length);
    pz = (float *) malloc(sizeof(float) * length);
    
    wx = (float *) malloc(sizeof(float) * length);
    wy = (float *) malloc(sizeof(float) * length);
    wz = (float *) calloc(length, sizeof(float));

    lenArr = (float *) malloc(sizeof(float) * length);
    
    for(int i = 0; i<length; i++) {
      px[i] = agents[i]->position.x;
      py[i] = agents[i]->position.y;
      pz[i] = agents[i]->position.z;
      //std::cout << "x: " << px[i] << ",y: " << py[i] << "\n";
    } 

  }
#define MAX_SOURCE_SIZE (0x100000)
  if(choice == OPENCL) {
    // ret is an error return value
    // Load the source code containing the kernel
    // and see if it succeded
    char fileName[] = "../libpedsim/whereToGo.cl";
    //printf("file to open = %s\n",fileName);
    fp = fopen(fileName,"r");
    //cout << "fp = " << fp << "\n";
    //    if(!fp) {
    if(fp == NULL) {
      fprintf(stderr,"Failed to load kernel, fp == NULL\n");
      exit(1);
    }
    // allocate string and copy file fp data to
    // source_str
    source_str = (char*)malloc(MAX_SOURCE_SIZE);
    source_size = fread(source_str,1, MAX_SOURCE_SIZE, fp);
    fclose(fp);

    // get platform and device info
    ret = clGetPlatformIDs(1, &platform_id, &ret_num_platforms);
    if(ret != CL_SUCCESS) {
      fprintf(stderr,"Failed to get platformID\n");
      exit(1);
    }
    ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);
    if(ret != CL_SUCCESS) {
      fprintf(stderr,"Failed to get deviceID\n");
      exit(1);
    } 
    // Create OpenCL context 
    context = clCreateContext(NULL, 1, &device_id, NULL, NULL, &ret);
    if(context == NULL) {
      fprintf(stderr,"failed to create context\n");
      cout << context << "\n";
      exit(1);
    }
    
    
    // create commando queue
    command_queue = clCreateCommandQueue(context,device_id,0,&ret);
    if (command_queue == NULL) {
      fprintf(stderr,"Failed to create command_queue\n");
      exit(1);
    }
    // Create memorybuffer for the GPU
    size_t memoryToAllocate = sizeof(float)*length;
    memobjx = clCreateBuffer(context, CL_MEM_READ_WRITE,memoryToAllocate, NULL, &ret);
    memobjy = clCreateBuffer(context, CL_MEM_READ_WRITE,memoryToAllocate, NULL, &ret);
    memobjwx = clCreateBuffer(context, CL_MEM_READ_WRITE,memoryToAllocate, NULL, &ret);
    memobjwy = clCreateBuffer(context, CL_MEM_READ_WRITE,memoryToAllocate, NULL, &ret);
    memobjlenarr = clCreateBuffer(context, CL_MEM_READ_WRITE,memoryToAllocate, NULL, &ret);
    // Creates a program object for a context, and loads the source
    // code specified by the text strings(source_str) in the strings array into 
    // the program object (program). 
  program = clCreateProgramWithSource(context, 1, (const char **)&source_str,
				      (const size_t *)&source_size, &ret);
  if(program == NULL) {
    fprintf(stderr,"Failed to create program\n");
    exit(1);
  }
  
  // Build Kernel Program. Compile the program into an executabe binary
  ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);
  if(ret != CL_SUCCESS) {
    fprintf(stderr,"Failed to build program\n");
    exit(1);
  }
  // should this be HERE???
  // Anyways we load the program to the kernel and loads the argument to the kernels
  kernel = clCreateKernel(program, "whereToGo", &ret);
  if(kernel==NULL){
    fprintf(stderr,"Failed to create kernel\n");
    exit(1);
  }
  if(clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&memobjx) != CL_SUCCESS|| 
     clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&memobjy) != CL_SUCCESS||
     clSetKernelArg(kernel, 2, sizeof(cl_mem), (void *)&memobjwx)!= CL_SUCCESS||
     clSetKernelArg(kernel, 3, sizeof(cl_mem), (void *)&memobjwy)!= CL_SUCCESS||
     clSetKernelArg(kernel, 4, sizeof(cl_mem), (void *)&memobjlenarr)!= CL_SUCCESS) {
    fprintf(stderr,"Failed to set kernel parameters\n");
    exit(1);
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
  return NULL;
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
      whereToGoVec(agents);

      for (int i = 0; i < length; i += 4) {
	// SSEx will contain four first floats starting at px[i] ...
	SSEx = _mm_load_ps(&px[i]);
	SSEy = _mm_load_ps(&py[i]);
	SSEz = _mm_load_ps(&pz[i]);

	SSEwx = _mm_load_ps(&wx[i]);
	SSEwy = _mm_load_ps(&wy[i]);
	SSEwz = _mm_load_ps(&wz[i]);

	calc_diff(&SSEx, &SSEy, &SSEz, SSEwx, SSEwy, SSEwz);
	normalize(&SSEx, &SSEy, &SSEz, &SSEwx, &SSEwy, &SSEwz, lenArr, i);

	// Store result back into array
	_mm_store_ps(&wx[i], SSEwx);
	_mm_store_ps(&wy[i], SSEwy);  
	_mm_store_ps(&wz[i], SSEwz);
      }
      
      /* <go()> */
      for(int i = 0; i < length; i += 4) {
	goVec(i);
      }
      /* </go()> */

      // Update the agents with the new values
      for(int i = 0; i < length; i++) {
	updateAgents(i);
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
  case OPENCL:
    {
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

      if( clEnqueueWriteBuffer(command_queue,memobjx,CL_TRUE,0,sizeof(float)*length,px,0,NULL,NULL) != CL_SUCCESS ||
	  clEnqueueWriteBuffer(command_queue,memobjy,CL_TRUE,0,sizeof(float)*length,py,0,NULL,NULL) != CL_SUCCESS ||
	  clEnqueueWriteBuffer(command_queue,memobjwx,CL_TRUE,0,sizeof(float)*length,wx,0,NULL,NULL) != CL_SUCCESS ||
	  clEnqueueWriteBuffer(command_queue,memobjwy,CL_TRUE,0,sizeof(float)*length,wy,0,NULL,NULL) != CL_SUCCESS ||
	  clEnqueueWriteBuffer(command_queue,memobjlenarr,CL_TRUE,0,sizeof(float)*length,lenArr,0,NULL,NULL) != CL_SUCCESS ) {
	fprintf(stderr, "failed to enqueue parameters to kernel, in tick\n");
	
	cout << "ret = " << ret << "\n";
	exit(1);
      }

      // Execute OpenCL kernel as data parallel 
      size_t global_item_size = length;
      size_t local_item_size = 1;
      ret = clEnqueueNDRangeKernel(command_queue, kernel, 1,NULL, &global_item_size, &local_item_size, 0, NULL, NULL);
      if(ret != CL_SUCCESS) {
	//cout << "ret = " << ret << " :";
	fprintf(stderr,"Failed to load kernels in tick\n");
	exit(1);
      }

      ret = clEnqueueReadBuffer(command_queue,memobjx,CL_TRUE,0,sizeof(float)*length,px,0,NULL,NULL);
      ret = clEnqueueReadBuffer(command_queue,memobjy,CL_TRUE,0,sizeof(float)*length,py,0,NULL,NULL);
      ret = clEnqueueReadBuffer(command_queue,memobjwx,CL_TRUE,0,sizeof(float)*length,wx,0,NULL,NULL);
      ret = clEnqueueReadBuffer(command_queue,memobjwy,CL_TRUE,0,sizeof(float)*length,wy,0,NULL,NULL);
      ret = clEnqueueReadBuffer(command_queue,memobjlenarr,CL_TRUE,0,sizeof(float)*length,lenArr,0,NULL,NULL);
    
      /*
      clFlush(command_queue);
      clFinish(command_queue);
      */

      // printf("check values: %f, %f, %f, %f\n",px[0],px[1],px[2],px[3]);
      for(int i = 0; i < length; i += 4) {
	goVec(i);
      }
      
      

      for(int i = 0; i < length; i++) {
	updateAgents(i);
	//std::cout << "scherman x: " << px[i] << " scherman y: " << py[i] << "\n";
      }

      break;
    }
  default:
    {
      //pthread_exit(NULL);
    }
  }
}

void Ped::Model::whereToGoVec(std::vector<Tagent*> agents) {
  for(int i = 0; i < agents.size(); i++){
    agents[i]->setNextDestination();
    Twaypoint* tempDest = agents[i]->getDestination();
    Twaypoint* tempLastDest = agents[i]->getLastDestination();

    if(tempDest != NULL) {
      wx[i] = tempDest->getx();
      wy[i] = tempDest->gety();
    }
    if (tempLastDest == NULL) {
      bool reachesDestination = false;
      //std::cout << "TESTTEST\n";
      Twaypoint tempDestination(tempDest->getx(), tempDest->gety(), tempDest->getr());
      tempDestination.settype(Ped::Twaypoint::TYPE_POINT);
      Tvector direction = tempDestination.getForce(agents[i]->position.x, agents[i]->position.y, 0, 0, &reachesDestination);
      agents[i]->setWaypointForce(direction);

    }
  }
}

void Ped::Model::goVec(int i) {
  __m128 SSEwx = _mm_load_ps(&wx[i]);
  __m128 vTemp = _mm_load_ps(&px[i]);
  __m128 v = _mm_add_ps(vTemp,SSEwx);
  v = _mm_round_ps(v, _MM_FROUND_TO_NEAREST_INT);

  _mm_store_ps(&px[i],v);

  __m128 SSEwy = _mm_load_ps(&wy[i]);
  vTemp = _mm_load_ps(&py[i]);
  v = _mm_add_ps(vTemp,SSEwy);
  v = _mm_round_ps(v, _MM_FROUND_TO_NEAREST_INT);
  _mm_store_ps(&py[i],v);	
}

void Ped::Model::normalize(__m128 *SSEx, __m128 *SSEy, __m128 *SSEz, __m128 *SSEwx, __m128 *SSEwy, __m128 *SSEwz, float *lenArr, int i) {
  __m128 temp;
  // normalization starting
  // x*x + y*y
  temp = _mm_add_ps(_mm_mul_ps(*SSEx,*SSEx),_mm_mul_ps(*SSEy,*SSEy));
  // temp + z*z
  temp = _mm_add_ps(temp,_mm_mul_ps(*SSEz,*SSEz));
  // sqrt(temp)
  temp = _mm_sqrt_ps(temp);

  _mm_store_ps(&lenArr[i], temp);

  *SSEwx = _mm_div_ps(*SSEx,temp);
  *SSEwy = _mm_div_ps(*SSEy,temp);
  *SSEwz = _mm_div_ps(*SSEz,temp);
}

void Ped::Model::calc_diff(__m128 *SSEx, __m128 *SSEy, __m128 *SSEz, __m128 SSEwx, __m128 SSEwy, __m128 SSEwz) {
  *SSEx = _mm_sub_ps(SSEwx,*SSEx);
  *SSEy = _mm_sub_ps(SSEwy,*SSEy);
  *SSEz = _mm_sub_ps(SSEwz,*SSEz);
}

void Ped::Model::updateAgents(int i) {
  agents[i]->position.x = round(px[i]);
  agents[i]->position.y = round(py[i]);

  agents[i]->setWaypointForce(Ped::Tvector(wx[i], wy[i], wz[i]));

  
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
}
