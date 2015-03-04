__kernel void fadeHeatmap(__global int* heatmap, __global int* row_size) {
  int row = get_global_id(0);
  int column = get_global_id(1);
  heatmap[row*(*row_size) + column] *= 0.80;
}

__kernel void heatmap(__global int* heatmap, __global int* row_size,__global int* x,__global int* y) {
  int SIZE = 1024;
  int agent = get_global_id(0);
  if((x[agent] >= 0) && (x[agent] <= SIZE) && (y[agent] <= SIZE) && (y[agent] >= 0)) {
    atomic_add(&(heatmap[y[agent] * (*row_size) + x[agent]]), 40);
  }

  atomic_min(&(heatmap[y[agent] * (*row_size) + x[agent]]), 255);
}


/* Spawn a thread per cell i heatmap */
__kernel void scaleHeatmap(__global int* scaledHeatmap, __global int* heatmap, __global int *row_size) {
    int row = get_global_id(0);
    int column = get_global_id(1);
    int CELLSIZE = 5;
    scaledHeatmap[row * (*row_size) + column] = heatmap[(int)(row/5 *((*row_size)/5)) + (int)(column/5)];
}

/*
  const int w[5][5] = {
  {1,4,7,4,1},
  {4,16,26,16,4},
  {7,26,41,26,7},
  {4,16,26,16,4},
  {1,4,7,4,1}
  };
  __kernel void gaussian_blur(__global float* scaledHeatmap, __global int row_size) {
  int row = get_global_id(0);
  int column = get_global_id(1);
  int sum = 0;
  for (int k = -2; k < 3; k++) {
  for (int l = -2; l < 3; l++) {
  sum += w[2 + k][2 + l] * scaledHeatMap[(row * row_size + k) + (column + l)];
  }
  }
  sum = sum/WEIGHTSUM;
  scaledHeapmap[row * row_size + column] = 0x00FF0000 | sum<<24;
  }
*/
