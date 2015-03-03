__kernel void heatmap(__global int* heatmap, __global int* row_size,__global int* x,__global int* y) {
  int SIZE = 1024;
  int agent = get_global_id(0);
  if((x[agent] <= SIZE) || (y[agent] <= SIZE)) {
    printf("update heatmap[%d][%d] agen%d\n",x[agent],y[agent],agent);
    atom_add(&(heatmap[x[agent] * (*row_size) + y[agent]]), 40);
  }
}
  const int w[5][5] = {
    {1,4,7,4,1},
    {4,16,26,16,4},
    {7,26,41,26,7},
    {4,16,26,16,4},
    {1,4,7,4,1}
};

__kernel void createHeatmap(__global float* heatmap, __global int row_size) {
    int row = get_global_id(0);
    int column = get_global_id(1);
    if(x[agent] =< SIZE || y[agent] =< SIZE) {
      atom_xchg(heatmap[x[agent] * row_size + y[agent]], atomic_min(heatmap[x[agent] * row_size + y[agent]] + 40, 255));
    }
}

__kernel void fadeHeatmap(__global float* heatmap, __global int row_size) {
    int row = get_global_id(0);
    int column = get_global_id(1);
    heatmap[row*row_size + column] *= 0.80;
}

__kernel void scaleHeatmap(__global float* scaledHeatmap, __global float* heatmap, __global int row_size) {
    int row = get_global_id(0);
    int column = get_global_id(1);
    scaledHeatmap[row * CELLSIZE * row_size + column * CELLSIZE] = heatmap[row * row_size + column];
}

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
