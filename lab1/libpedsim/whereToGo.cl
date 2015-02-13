__kernel void whereToGo(__global float* x, __global float* y,__global float* wx, __global float* wy, __global float* len_arr) {
  int id = get_global_id(0);
  float xInt = wx[id] - x[id];
  float yInt = wy[id] - y[id];
  float temp = xInt*xInt + yInt*yInt;
  temp = sqrt(temp);
  len_arr[id] = temp;
  wx[id] = xInt / temp;
  wy[id] = yInt / temp;
}
