__kernel void whereToGo(__global float* x, __global float* y,__global float* wx, __global float* wy, __global float* len_arr) {
  int id = get_global_id(0);
  x[id] = wx[id] - x[id];
  y[id] = wy[id] - y[id];
  float temp = x[id]*x[id] + y[id] * y[id];
  temp = sqrt(temp);
  len_arr[id] = temp;
  wx[id] = x[id] / temp;
  wy[id] = y[id] / temp;

}
