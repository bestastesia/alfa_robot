#include "timeopt/timeopt_path_kernel.cuh"

namespace prefix_numeric {
using namespace curobo::trajectory::bspline;
template<int Degree, BasisBackend Backend>
__device__ void evaluate(const float* c, float t, float* values) {
  auto context = create_context<Degree, Backend>(t, 0.75f);
  values[0] = context.compute_position(c);
  values[1] = context.compute_velocity(c);
  values[2] = context.compute_acceleration(c);
  values[3] = context.compute_jerk(c);
}
template<int Degree>
__device__ void dispatch(const float* c, float t, float* values, int backend) {
  if (backend == 0) evaluate<Degree, BasisBackend::POLYNOMIAL>(c,t,values);
  else if (backend == 1) evaluate<Degree, BasisBackend::FACTORED>(c,t,values);
  else evaluate<Degree, BasisBackend::MATRIX>(c,t,values);
}
__global__ void context_check(const float* controls, const float* t, float* output,
                             int samples, int cases, int degree, int backend) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= samples * cases) return;
  const float* c = controls + (index / samples) * 6;
  float values[4];
  if (degree == 3) dispatch<3>(c,t[index%samples],values,backend);
  else if (degree == 4) dispatch<4>(c,t[index%samples],values,backend);
  else dispatch<5>(c,t[index%samples],values,backend);
  for (int k=0;k<4;++k) output[k*samples*cases+index] = values[k];
}
__global__ void local_tail_check(const float* controls, const float* start, const float* goal,
                                const float* t, float* output, int samples) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= samples) return;
  float values[4];
  curobo::trajectory::timeopt::sample_joint(controls,start,goal,0,0,1,32,1,33,t[index],values);
  for (int k=0;k<4;++k) output[k*samples+index] = values[k];
}
}
