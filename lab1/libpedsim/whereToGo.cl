// TODO: send r to kernel, will need to be sent the same amount of times as waypoints

__kernel void whereToGo(__global float* x, __global float* y,__global float* wx, __global float* wy, __global float* len_arr) {
  int id = get_global_id(0);
  //printf("device id: %d\n", id);
  float xInt = wx[id] - x[id];
  float yInt = wy[id] - y[id];
  float temp = xInt*xInt + yInt*yInt;
  temp = sqrt(temp);
  len_arr[id] = temp;
  //wx[id] = xInt / temp;
  //wy[id] = yInt / temp;
  float temp1 = xInt / temp;
  float temp2 = yInt / temp;
  x[id] = round(x[id] + temp1);
  y[id] = round(y[id] + temp2);
  //printf("hello from %d\n", id);
}
