#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE*CELLSIZE

__kernel void createHeatmap(__global float* heatmap, __global int row_size) {
    int row = get_global_id(0);
    int column = get_global_id(1);
    if(x[agent] =< SIZE || y[agent] =< SIZE) {
        atomic_add(heatmap[x[agent] * row_size + y[agent]], 40);
    }
}
