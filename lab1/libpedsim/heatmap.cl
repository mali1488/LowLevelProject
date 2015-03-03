__kernel void heatmap(__global int* heatmap, __global int* row_size,__global int* x,__global int* y) {
  int SIZE = 1024;
  int agent = get_global_id(0);
  if((x[agent] <= SIZE) || (y[agent] <= SIZE)) {
    printf("update heatmap[%d][%d] agen%d\n",x[agent],y[agent],agent);
    atom_add(&(heatmap[x[agent] * (*row_size) + y[agent]]), 40);
  }
}
