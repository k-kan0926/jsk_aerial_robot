#include "mpa_acados_controller/narx_mpc_solver.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>

namespace mpa_acados {

namespace {
inline double clip01(double x){ return std::max(0.0, std::min(1.0, x)); }
}

NarxMpcSolver::~NarxMpcSolver() {
  if (inited_) {
    // acados の free は acados_solver_X.c 内の関数で行う
    narx_disc_acados_free();
    narx_disc_acados_free_capsule();
  }
}

bool NarxMpcSolver::init(const std::string& models_root, const NarxParams& prm) {
  prm_ = prm;

  // （必要なら）モデル検索パスを使う場合は環境変数で渡す等。
  // ただし通常は c_generated_code をビルド対象に入れれば不要。

  // create capsule & create solver
  if (narx_disc_acados_create() != 0) {
    std::cerr << "[acados] create failed\n";
    return false;
  }
  inited_ = true;
  return true;
}

void NarxMpcSolver::set_parameters(double theta_ref, double dz0) {
  // stage 0..N-1
  for (int k = 0; k < narx_disc_N; ++k) {
    double p_vec[2] = {theta_ref, dz0};
    narx_disc_acados_update_params(k, p_vec, 2);
  }
  // terminal
  {
    double p_vec[2] = {theta_ref, dz0};
    narx_disc_acados_update_params(narx_disc_N, p_vec, 2);
  }
}

void NarxMpcSolver::set_x0(const std::vector<double>& x0) {
  if ((int)x0.size() != narx_disc_NX) {
    throw std::runtime_error("x0 length mismatch");
  }
  ocp_nlp_solver* solver = narx_disc_acados_get_solver();
  ocp_nlp_dims* dims = narx_disc_acados_get_nlp_dims();

  // x(0) = x0
  ocp_nlp_out* out = narx_disc_acados_get_nlp_out();
  ocp_nlp_in*  in  = narx_disc_acados_get_nlp_in();

  // set x0 constraint
  for (int i = 0; i < narx_disc_NX; ++i)
    in->constraints->idxbx[0][i] = i;
  in->constraints->nbx[0] = narx_disc_NX;
  std::memcpy(in->constraints->lbx[0], x0.data(), sizeof(double)*narx_disc_NX);
  std::memcpy(in->constraints->ubx[0], x0.data(), sizeof(double)*narx_disc_NX);

  // also set the out state guess to help convergence
  for (int i = 0; i <= narx_disc_N; ++i) {
    ocp_nlp_out_set(solver, i, "x", (void*)x0.data());
  }
}

void NarxMpcSolver::set_uniform_initial_u(double ps0, double pd0) {
  // acados 入力は u = [a, b] with a=(ps+pd)/(2*pmax), b=(ps-pd)/(2*pmax)
  double a0 = clip01((ps0 + pd0) / (2.0 * prm_.pmax));
  double b0 = clip01((ps0 - pd0) / (2.0 * prm_.pmax));
  ocp_nlp_solver* solver = narx_disc_acados_get_solver();
  double u0[2] = {a0, b0};
  for (int k = 0; k < narx_disc_N; ++k) {
    ocp_nlp_out_set(solver, k, "u", u0);
  }
}

int NarxMpcSolver::solve() {
  return narx_disc_acados_solve();
}

void NarxMpcSolver::get_first_control(double& ps, double& pd) const {
  ocp_nlp_solver* solver = narx_disc_acados_get_solver();
  double u0[2]; // [a,b]
  ocp_nlp_out_get(solver, 0, "u", u0);
  double a = clip01(u0[0]), b = clip01(u0[1]);
  ps = prm_.pmax * (a + b);
  pd = prm_.pmax * (a - b);
}

} // namespace mpa_acados
