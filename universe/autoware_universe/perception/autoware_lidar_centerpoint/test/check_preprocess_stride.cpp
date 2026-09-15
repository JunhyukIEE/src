// Standalone regression check (from workspace root, after sourcing install/setup.bash):
// g++ -std=c++17 src/universe/autoware_universe/perception/autoware_lidar_centerpoint/test/check_preprocess_stride.cpp
//   -Iinstall/autoware_lidar_centerpoint/include -I/usr/local/cuda/include
//   -Linstall/autoware_lidar_centerpoint/lib -Wl,--no-as-needed
//   -lautoware_lidar_centerpoint_lib -lautoware_lidar_centerpoint_cuda_lib
//   -L/usr/local/cuda/lib64 -lcudart -o /tmp/check_centerpoint_preprocess
// /tmp/check_centerpoint_preprocess
#include "autoware/lidar_centerpoint/preprocess/preprocess_kernel.hpp"
#include <cuda_runtime.h>

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

using namespace autoware::lidar_centerpoint;

int main()
{
  cudaStream_t stream{};
  auto check = [](cudaError_t error) { assert(error == cudaSuccess); };
  check(cudaStreamCreate(&stream));
  CenterPointConfig config(1, 4, 8, 8, {-10, -10, -4, 10, 10, 6},
    {1, 1, 10}, 1, 9, {0.35}, 0.5, {0.0}, false);
  PreprocessCuda preprocess(config, stream);
  unsigned char * input{};
  float * transform{};
  float * output{};
  float * shuffled{};
  unsigned * indices{};
  check(cudaMalloc(&input, 64));
  check(cudaMalloc(&transform, 16 * sizeof(float)));
  check(cudaMalloc(&output, 8 * 4 * sizeof(float)));
  check(cudaMalloc(&shuffled, 8 * 4 * sizeof(float)));
  check(cudaMalloc(&indices, 8 * sizeof(unsigned)));
  const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  check(cudaMemcpy(transform, identity, sizeof(identity), cudaMemcpyHostToDevice));
  for (unsigned stride : {16u, 32u}) {
    std::vector<unsigned char> bytes(2 * stride, 0xff);
    InputPointType a{1, 2, 3, 4, 0, 0}, b{5, 6, 7, 8, 0, 0};
    std::memcpy(bytes.data(), &a, sizeof(a));
    std::memcpy(bytes.data() + stride, &b, sizeof(b));
    check(cudaMemcpy(input, bytes.data(), bytes.size(), cudaMemcpyHostToDevice));
    check(preprocess.generateSweepPoints_launch(
      reinterpret_cast<InputPointType *>(input), 2, stride, 0.25f, transform, output));
    check(cudaStreamSynchronize(stream));
    float host[8];
    check(cudaMemcpy(host, output, sizeof(host), cudaMemcpyDeviceToHost));
    assert(host[0] == 1 && host[2] == 3 && host[4] == 5 && host[6] == 7);
    assert(host[3] == 0.25f && host[7] == 0.25f);
    const unsigned permutation[8] = {7, 6, 5, 4, 3, 2, 1, 0};
    check(cudaMemcpy(indices, permutation, sizeof(permutation), cudaMemcpyHostToDevice));
    check(preprocess.shufflePoints_launch(output, indices, shuffled, 2, 8, 0));
    check(cudaStreamSynchronize(stream));
    float result[32];
    check(cudaMemcpy(result, shuffled, sizeof(result), cudaMemcpyDeviceToHost));
    for (unsigned i = 0; i < 6; ++i) assert(std::isinf(result[i * 4]));
    assert(result[24] == 5 && result[28] == 1);
  }
  check(cudaFree(input));
  check(cudaFree(transform));
  check(cudaFree(output));
  check(cudaFree(shuffled));
  check(cudaFree(indices));
  check(cudaStreamDestroy(stream));
  std::cout << "PASS: 16/32-byte stride and sparse shuffle\n";
}
