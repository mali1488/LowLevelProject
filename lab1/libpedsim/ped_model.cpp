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

#include <stack>
#include <algorithm>

#include <unistd.h>
#include <semaphore.h>

#define MAX_SOURCE_SIZE (0x100000)

#define BALANCE_CONSTANT 2

struct parameters;

sem_t tickSem;


bool bounce = true;

// Comparator used to identify if two agents differ in their position
bool cmp(Ped::Tagent *a, Ped::Tagent *b) {
  return (a->getX() < b->getX()) || ((a->getX() == b->getX()) && (a->getY() < b->getY()));
}

void Ped::Model::calculateWorkLoad(int amountAgents) {
    // TODO: rounds down - problem?
    int avg = amountAgents / (this->number_of_threads);
    std::cout << "avg: " << avg << "\n";
    std::vector<Ped::Ttree*> *leaves = new vector<Ped::Ttree*>;
    this->tree->getLeaves(leaves);
    int thread = 0;
    int leafCounter = 0;
    
    std::vector<Ped::Ttree*>::iterator it;
    for (it = leaves->begin(); it != leaves->end(); ++it) {
        leafCounter += (*it)->agents->agentSet.size();
        if (agentCounter[thread % number_of_threads] > avg) {
            thread++;
        }
        Params[thread % number_of_threads]->workLoad->push_back(*it);
        // TODO: checking size without checking lock???
        agentCounter[thread % number_of_threads] += (*it)->agents->agentSet.size();
    }
    std::cout << "leaf size: " << leafCounter << "\n";
    std::cout << "agents: " << amountAgents << "\n";
    std::cout << "agents in tree: " << tree->getAgents().size() << "\n";
    if (leafCounter != amountAgents) {
        std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAH\n\n\n\n\n\n\n";
    }

}

void Ped::Model::naiveBalance() {
    int min_idx = 0;
    int max_idx = 0;
    for (int i = 1; i < number_of_threads; i++) {
        if (agentCounter[i] < agentCounter[min_idx]) {
            min_idx = i;
        }
        if (agentCounter[i] > agentCounter[max_idx]) {
            max_idx = i;
        }
    }
    //std::cout << "min idx: " << min_idx << "\nmax idx: " << max_idx << "\n";
    //if (agentCounter[min_idx] * BALANCE_CONSTANT > agentCounter[max_idx]) {
    if (agentCounter[max_idx] * BALANCE_CONSTANT > agentCounter[min_idx]) {
      Ped::Ttree *heaviest_tree = (*Params[max_idx]->workLoad)[0];
        int heaviest_tree_agents = heaviest_tree->getAgents().size();
        int idx = 0;

        for (int i = 1; i < Params[max_idx]->workLoad->size(); i++) {
	  Ped::Ttree *currentTree = (*Params[max_idx]->workLoad)[i];
            int currentAgents = currentTree->getAgents().size();
            if (currentAgents > heaviest_tree_agents) {
                heaviest_tree = currentTree;
                heaviest_tree_agents = currentAgents;
                idx = i;
            }
        }
        if (!heaviest_tree->isleaf) {
            Params[min_idx]->workLoad->push_back(heaviest_tree->tree1);
            Params[min_idx]->workLoad->push_back(heaviest_tree->tree2);
            Params[max_idx]->workLoad->erase(Params[max_idx]->workLoad->begin() + idx);
            Params[max_idx]->workLoad->push_back(heaviest_tree->tree3);
            Params[max_idx]->workLoad->push_back(heaviest_tree->tree4);
        }
    }
}

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION choice, int numThreads) {
  sem_init(&tickSem, 1, 0);
  total_opencl_time = 0;
  
  /*
  agents = agentsInScenario;
  std::vector<Tagent*> agents = Ped::Model::getAgents();  */

  implementation = choice;
  number_of_threads = numThreads;

  // Hack! do not allow agents to be on the same position. Remove duplicates from scenario.
  bool (*fn_pt)(Ped::Tagent*, Ped::Tagent*) = cmp;
  std::set<Ped::Tagent*, bool(*)(Ped::Tagent*, Ped::Tagent*)> agentsWithUniquePosition (fn_pt);
  std::copy(agentsInScenario.begin(), agentsInScenario.end(), std::inserter(agentsWithUniquePosition, agentsWithUniquePosition.begin()));
 
  agents = std::vector<Ped::Tagent*>(agentsWithUniquePosition.begin(), agentsWithUniquePosition.end());
  treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();
 
  // Create a new quadtree containing all agents
  tree = new Ttree(NULL,treehash, 0, treeDepth, 0, 0, 200, 150); //TODO: dimension?

  for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it){
    tree->addAgent(*it);
  }

  tree->tree1->toString();
  tree->tree2->toString();
  tree->tree3->toString();
  tree->tree4->toString();

  int length = agents.size();  
  if(choice == PTHREAD){
    this->Params = new struct parameters* [number_of_threads];
   
    this->agentCounter = new int[number_of_threads];
    pthread_t threads[number_of_threads];
    sem_init(&(this->testSem), 1, 1);

    for (int i = 0; i < number_of_threads; i++) {
        this->Params[i] = new struct parameters();
	this->Params[i]->workLoad = new vector<Ped::Ttree*>;
        this->Params[i]->model = this;
        this->agentCounter[i] = 0;
        this->Params[i]->idx = i;
        sem_init(&(this->Params[i]->semaphore), 1, 0);
	sem_init(&(this->Params[i]->mainSem), 1, 1);
        if(pthread_create(&threads[i],NULL,&(threaded_tick),(void*)this->Params[i])) {
            perror("Could not create thread!");
            exit(1);
        }
    }

    this->Params[0]->workLoad->push_back(tree->tree1);
    this->Params[1]->workLoad->push_back(tree->tree2);
    this->Params[2]->workLoad->push_back(tree->tree3);
    this->Params[3]->workLoad->push_back(tree->tree4);

    /*
    this->Params[0]->tree = tree->tree1;
    this->Params[1]->tree = tree->tree2;
    this->Params[2]->tree = tree->tree3;
    this->Params[3]->tree = tree->tree4; */

    //std::cout << "this->Params[0]->workLoad.size(): " << (*this->Params[0]->workLoad).size() << "\n";

  }
  if(choice == VECTOR || choice == TEST || choice == OPENCL) {
    px = (float *) malloc(sizeof(float) * length);
    py = (float *) malloc(sizeof(float) * length);
    pz = (float *) malloc(sizeof(float) * length);
   
    wx = (float *) malloc(sizeof(float) * length);
    wy = (float *) malloc(sizeof(float) * length);
    wz = (float *) calloc(length, sizeof(float));
   
    rArr = (float *) malloc(sizeof(float) * length);
    reachedArr = (float *) calloc(length, sizeof(float));
   
   
    for(int i = 0; i<length; i++) {
      px[i] = agents[i]->position.x;
      py[i] = agents[i]->position.y;
      pz[i] = agents[i]->position.z;
    }
   
  }

  if(choice == OPENCL) {
    // ret is an error return value
    // Load the source code containing the kernel
    // and see if it succeded
    char fileName[] = "../libpedsim/whereToGo.cl";
    fp = fopen(fileName,"r");
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

    ret_num_devices = CL_DEVICE_MAX_COMPUTE_UNITS;
    ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_GPU, 1, &device_id, &ret_num_devices);

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
    memobjwx = clCreateBuffer(context, CL_MEM_READ_ONLY,memoryToAllocate, NULL, &ret);
    memobjwy = clCreateBuffer(context, CL_MEM_READ_ONLY,memoryToAllocate, NULL, &ret);
    memobjrArr = clCreateBuffer(context, CL_MEM_READ_ONLY,memoryToAllocate, NULL, &ret);
    memobjReachedArr = clCreateBuffer(context, CL_MEM_READ_WRITE,memoryToAllocate, NULL, &ret);

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
      cout << ret << "\n";
      fprintf(stderr,"Failed to build program\n");
      exit(1);
    }

    // Load the program to the kernel and loads the argument to the kernels
    kernel = clCreateKernel(program, "whereToGo", &ret);
    if(kernel==NULL){
      fprintf(stderr,"Failed to create kernel\n");
      exit(1);
    }
    if(clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&memobjx) != CL_SUCCESS|| 
       clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&memobjy) != CL_SUCCESS||
       clSetKernelArg(kernel, 2, sizeof(cl_mem), (void *)&memobjwx)!= CL_SUCCESS||
       clSetKernelArg(kernel, 3, sizeof(cl_mem), (void *)&memobjwy)!= CL_SUCCESS||
       clSetKernelArg(kernel, 4, sizeof(cl_mem), (void *)&memobjrArr)!= CL_SUCCESS ||
       clSetKernelArg(kernel, 5, sizeof(cl_mem), (void *)&memobjReachedArr)!= CL_SUCCESS) {
      fprintf(stderr,"Failed to set kernel parameters\n");
      exit(1);
    }

    /* Write starting positions to device memory */
    clEnqueueWriteBuffer(command_queue,memobjx,CL_TRUE,0,sizeof(float)*length,px,0,NULL,NULL);
    clEnqueueWriteBuffer(command_queue,memobjy,CL_TRUE,0,sizeof(float)*length,py,0,NULL,NULL);
  }
}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

void* Ped::Model::threaded_tick(void* parameters){
  struct parameters* params = (struct parameters*) parameters;
  std::vector<Ped::Ttree*> *trees;
  trees = (params->workLoad);

  while(true) {
    sem_wait(&(params->semaphore));
    
    //std::cout << "thread id: " << params->idx << "\nworkload size: " << trees->size() << "\n";
    int agentsUpdated = 0;    
    for (std::vector<Ped::Ttree*>::iterator i = trees->begin(); i != trees->end(); ++i) {
      std::set<const Ped::Tagent*> agents = (*i)->getAgents();
      (*i)->toString();
        agentsUpdated += agents.size();
        for (std::set<const Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it) {
	  //std::cout << "--XYZ---idx " << params->idx << " executing!\n";
            Ped::Tagent* currentAgent = const_cast<Ped::Tagent*>(*it);
            currentAgent->whereToGo();
            currentAgent->go();                 // This rather becomes a "computeNextDesiredPosition"
            // Search for neighboring agents
	    sem_wait(&params->model->testSem);
            params->model->doSafeMovement(currentAgent);    
	    sem_post(&params->model->testSem);
        }
    }
    params->model->agentCounter[params->idx] = agentsUpdated;
    sem_post(&(params->mainSem));
    //std::cout << "thread id: " << params->idx << "\nupdated: " << agentsUpdated << "\n";
  }
}

void Ped::Model::tick()
{
  //std::vector<Tagent*> agents = Ped::Model::getAgents();
  int length = agents.size();
  int delta = length / this->number_of_threads;
  
  switch(this->implementation) {
  case SEQ:
    { 
      //calculateWorkLoad(length);
      for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it) {
	  Ped::Tagent *agent = (*it);
	  agent->whereToGo();
	  agent->go();                 // This rather becomes a "computeNextDesiredPosition"
	  doSafeMovement(agent);
	    
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
	  doSafeMovement(agents[i]);
	}
      }
      break;
    }
  case PTHREAD:
    {
      for(int i = 0; i < number_of_threads; i++) { // TODO: add lock for creating new threads (otherwise this check might break)
	sem_wait(&(this->Params[i]->mainSem));
        agentCounter[i] = 0;
        sem_post(&(this->Params[i]->semaphore));
      }

      // TODO: add something to make sure all threads have finished running before continuing
      /*
      for(int i = 0; i < number_of_threads; i++){
          pthread_join(threads[i], NULL);
      } 
      */

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
      __m128 SSEr;
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

	SSEr = _mm_load_ps(&rArr[i]);

	calc_diff(&SSEx, &SSEy, &SSEz, SSEwx, SSEwy, SSEwz);
	normalize(&SSEx, &SSEy, &SSEz, &SSEwx, &SSEwy, &SSEwz, &SSEr, reachedArr, i);


	// Store result back into array
	_mm_store_ps(&wx[i], SSEwx);
	_mm_store_ps(&wy[i], SSEwy);  
	_mm_store_ps(&wz[i], SSEwz);
	goVec(i);
	for (int k = 0; k < 4; k++) {
	  updateAgents(i+k);
	}
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
	  rArr[i] = tempDest->getr();
        }
 
        if (tempLastDest == NULL) {
          bool reachesDestination = false;
          Twaypoint tempDestination(tempDest->getx(), tempDest->gety(), tempDest->getr());
          tempDestination.settype(Ped::Twaypoint::TYPE_POINT);
          Tvector direction = tempDestination.getForce(agents[i]->position.x, agents[i]->position.y, 0, 0, &reachesDestination);
          agents[i]->setWaypointForce(direction);
	}
      }
      
      if(bounce) {
	clEnqueueWriteBuffer(command_queue,memobjwx,CL_FALSE,0,sizeof(float)*length,wx,0,NULL,NULL);
	clEnqueueWriteBuffer(command_queue,memobjwy,CL_FALSE,0,sizeof(float)*length,wy,0,NULL,NULL);
	clEnqueueWriteBuffer(command_queue,memobjrArr,CL_FALSE,0,sizeof(float)*length,rArr,0,NULL,NULL);

	/* Reset reachedArr */
	for(int i = 0; i < length; i++){
	  reachedArr[i] = 0;
	}

	clEnqueueWriteBuffer(command_queue,memobjReachedArr,CL_TRUE,0,sizeof(float)*length,reachedArr,0,NULL,NULL);
	bounce = false;
      }

      // Execute OpenCL kernel as data parallel 
      size_t global_item_size = length;
      size_t local_item_size = 100;
      ret = clEnqueueNDRangeKernel(command_queue, kernel, 1,NULL, &global_item_size, &local_item_size, 0, NULL, NULL);
      if(ret != CL_SUCCESS) {
	cout << "ret = " << ret << " :";
	fprintf(stderr,"Failed to load kernels in tick\n");
	exit(1);
      }
      
      /* Read results from device */
      ret = clEnqueueReadBuffer(command_queue,memobjx,CL_FALSE,0,sizeof(float)*length,px,0,NULL,NULL);
      ret = clEnqueueReadBuffer(command_queue,memobjy,CL_FALSE,0,sizeof(float)*length,py,0,NULL,NULL);
      ret = clEnqueueReadBuffer(command_queue,memobjReachedArr,CL_FALSE,0,sizeof(float)*length,reachedArr,0,NULL,NULL);

      clFinish(command_queue);

      for(int i = 0; i < length; i++) {
	updateAgents(i);
      }

      break;
    }
  default:
    {
      break;
    }
  }
}

void  Ped::Model::doSafeMovementTest( Ped::Tagent *agent, int idx)
{
  // Search for neighboring agents
  sem_wait(&(Params[idx]->model->testSem));
  set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);
  sem_post(&(Params[idx]->model->testSem));
    
  // Retrieve their positions
  std::vector<std::pair<int, int> > takenPositions;
  for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
    std::pair<int,int> position((*neighborIt)->getX(), (*neighborIt)->getY());
    takenPositions.push_back(position);
  }

  // Compute the three alternative positions that would bring the agent
  // closer to his desiredPosition, starting with the desiredPosition itself
  std::vector<std::pair<int, int> > prioritizedAlternatives;
  std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
  prioritizedAlternatives.push_back(pDesired);

  int diffX = pDesired.first - agent->getX();
  int diffY = pDesired.second - agent->getY();
  std::pair<int, int> p1, p2;
  if (diffX == 0 || diffY == 0)
    {
      // Agent wants to walk straight to North, South, West or East
      p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
      p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
    }
  else {
    // Agent wants to walk diagonally
    p1 = std::make_pair(pDesired.first, agent->getY());
    p2 = std::make_pair(agent->getX(), pDesired.second);
  }
  prioritizedAlternatives.push_back(p1);
  prioritizedAlternatives.push_back(p2);

  // Find the first empty alternative position
  for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

    // If the current position is not yet taken by any neighbor
    if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

      // Set the agent's position 
      agent->setX((*it).first);
      agent->setY((*it).second);

      // Update the quadtree
      (*treehash)[agent]->moveAgent(agent);
      break;
    }
  }
}

void  Ped::Model::doSafeMovement( Ped::Tagent *agent)
{
  // Search for neighboring agents
  set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);
    
  // Retrieve their positions
  std::vector<std::pair<int, int> > takenPositions;
  for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
    std::pair<int,int> position((*neighborIt)->getX(), (*neighborIt)->getY());
    takenPositions.push_back(position);
  }

  // Compute the three alternative positions that would bring the agent
  // closer to his desiredPosition, starting with the desiredPosition itself
  std::vector<std::pair<int, int> > prioritizedAlternatives;
  std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
  prioritizedAlternatives.push_back(pDesired);

  int diffX = pDesired.first - agent->getX();
  int diffY = pDesired.second - agent->getY();
  std::pair<int, int> p1, p2;
  if (diffX == 0 || diffY == 0)
    {
      // Agent wants to walk straight to North, South, West or East
      p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
      p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
    }
  else {
    // Agent wants to walk diagonally
    p1 = std::make_pair(pDesired.first, agent->getY());
    p2 = std::make_pair(agent->getX(), pDesired.second);
  }
  prioritizedAlternatives.push_back(p1);
  prioritizedAlternatives.push_back(p2);

  // Find the first empty alternative position
  for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

    // If the current position is not yet taken by any neighbor
    if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

      // Set the agent's position 
      agent->setX((*it).first);
      agent->setY((*it).second);

      // Update the quadtree
      (*treehash)[agent]->moveAgent(agent);
      break;
    }
  }
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const {
  // if there is no tree, return all agents
  if(tree == NULL) 
    return set<const Ped::Tagent*>(agents.begin(), agents.end());

  // create the output list
  list<const Ped::Tagent*> neighborList;
  getNeighbors(neighborList, x, y, dist);

  // copy the neighbors to a set
  return set<const Ped::Tagent*>(neighborList.begin(), neighborList.end());
}

/// \date    2012-01-29
/// \param   the list to populate
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
void Ped::Model::getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int dist) const {
  stack<Ped::Ttree*> treestack;
  

  treestack.push(tree);
  while(!treestack.empty()) {
    int counter = 0;
    Ped::Ttree *t = treestack.top();
    treestack.pop();
    if (t->isleaf) {
      t->getAgents(neighborList);
    }
    else {
      if (t->tree1->intersects(x, y, dist)) {
	counter += 1;
	treestack.push(t->tree1);
      }
      if (t->tree2->intersects(x, y, dist)) {
	counter += 1;
	treestack.push(t->tree2);
      }
      if (t->tree3->intersects(x, y, dist)) {
	counter += 1;
	treestack.push(t->tree3);
      }
      if (t->tree4->intersects(x, y, dist)) {
	counter += 1;
	treestack.push(t->tree4);
      }
      //std::cout << "counter: " << counter << "\n";
    }
  }
}

/// Populates the list of neighbors that can be found around x/y./// This triggers a cleanup of the tree structure. Unused leaf nodes are collected in order to
/// save memory. Ideally cleanup() is called every second, or about every 20 timestep.
/// \date    2012-01-28
void Ped::Model::cleanup() {
  if(tree != NULL)
    tree->cut();
}

void Ped::Model::whereToGoVec(std::vector<Tagent*> agents) {
  for(int i = 0; i < agents.size(); i++){
    agents[i]->setNextDestination();
    Twaypoint* tempDest = agents[i]->getDestination();
    Twaypoint* tempLastDest = agents[i]->getLastDestination();

    if(tempDest != NULL) {
      wx[i] = tempDest->getx();
      wy[i] = tempDest->gety();
      rArr[i] = tempDest->getr();
    }
    if (tempLastDest == NULL) {
      bool reachesDestination = false;
      Twaypoint tempDestination(tempDest->getx(), tempDest->gety(), tempDest->getr());
      tempDestination.settype(Ped::Twaypoint::TYPE_POINT);
      Tvector direction = tempDestination.getForce(agents[i]->position.x, agents[i]->position.y, 0, 0, &reachesDestination);
      agents[i]->setWaypointForce(direction);
    }
  }
}

void Ped::Model::goVec(int i) {
  __m128 SSEwxx = _mm_load_ps(&wx[i]);
  __m128 vTemp = _mm_load_ps(&px[i]);
  __m128 v = _mm_add_ps(vTemp,SSEwxx);
  v = _mm_round_ps(v, _MM_FROUND_TO_NEAREST_INT);
  _mm_store_ps(&px[i], v);

  __m128 SSEwyy = _mm_load_ps(&wy[i]);
  vTemp = _mm_load_ps(&py[i]);
  v = _mm_add_ps(vTemp,SSEwyy);
  v = _mm_round_ps(v, _MM_FROUND_TO_NEAREST_INT);
  _mm_store_ps(&py[i], v);
}

void Ped::Model::normalize(__m128 *SSEx, __m128 *SSEy, __m128 *SSEz, __m128 *SSEwx, __m128 *SSEwy, __m128 *SSEwz, __m128 *SSEr, float *reachedArr, int i) {
  __m128 temp;
  __m128 reachedTemp;

  // normalization starting
  // x*x + y*y
  temp = _mm_add_ps(_mm_mul_ps(*SSEx,*SSEx),_mm_mul_ps(*SSEy,*SSEy));
  // temp + z*z
  temp = _mm_add_ps(temp,_mm_mul_ps(*SSEz,*SSEz));
  // sqrt(temp)
  temp = _mm_sqrt_ps(temp);

  reachedTemp = _mm_cmplt_ps(temp, *SSEr);

  _mm_store_ps(&reachedArr[i], reachedTemp);

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

  agents[i]->position.x = px[i];
  agents[i]->position.y = py[i];

  agents[i]->setWaypointForce(Ped::Tvector(wx[i], wy[i], wz[i]));

  Twaypoint* tempDest = agents[i]->getDestination();
  Twaypoint* tempLastDest = agents[i]->getLastDestination();

  if(tempDest != NULL){
    if(reachedArr[i]) { /* Agent i reached waypoint */
      deque<Twaypoint*> waypoints = agents[i]->getWaypoints();
      agents[i]->addWaypoint(tempDest);
      agents[i]->setLastDestination(tempDest);
      agents[i]->setDestination(NULL);
      bounce = true;
    }
  }
}

Ped::Model::~Model()
{
  if(tree != NULL)
    {
      delete tree;
      tree = NULL;
    }
  if(treehash != NULL)
    {
      delete treehash;
      treehash = NULL;
    }
}
