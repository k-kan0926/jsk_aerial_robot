#include "mpa_acados_controller/narx_acados_solver.h"

#include <algorithm>
#include <cstring>

namespace mpa_acados_controller
{

NarxAcadosSolver::NarxAcadosSolver()
  : capsule_(nullptr)
  , pmax_(0.7)
{
}

NarxAcadosSolver::~NarxAcadosSolver()
{
  if (capsule_)
  {
    narx_disc_acados_free(capsule_);
    narx_disc_acados_free_capsule(capsule_);
    capsule_ = nullptr;
  }
}

bool NarxAcadosSolver::init(double pmax)
{
  pmax_ = pmax;

  capsule_ = narx_disc_acados_create_capsule();
  if (!capsule_) return false;

  if (narx_disc_acados_create(capsule_) != 0)
  {
    narx_disc_acados_free_capsule(capsule_);
    capsule_ = nullptr;
    return false;
  }
  return true;
}

void NarxAcadosSolver::setX0(const std::vector<double> &x0)
{
  if (!capsule_) return;

  ocp_nlp_config *nlp_config = narx_disc_acados_get_nlp_config(capsule_);
  ocp_nlp_dims   *nlp_dims   = narx_disc_acados_get_nlp_dims(capsule_);
  ocp_nlp_in     *nlp_in     = narx_disc_acados_get_nlp_in(capsule_);

  // ← ここを「関数」じゃなくて「マクロ」で取る
  const int nx = NARX_DISC_NX;
  const int N  = NARX_DISC_N;

  std::vector<double> x_init(nx, 0.0);
  const int copy_n = std::min<int>(nx, x0.size());
  if (copy_n > 0)
    std::memcpy(x_init.data(), x0.data(), sizeof(double) * copy_n);

  // stage 0
  ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, 0, "x", x_init.data());

  // 他ステージにも同じのを入れておくと初回が安定する
  for (int stage = 1; stage <= N; ++stage)
    ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, stage, "x", x_init.data());
}

void NarxAcadosSolver::setParams(double theta_ref, double dz0)
{
  if (!capsule_) return;

  // このモデルは np=2 のはず
  double p[2] = {theta_ref, dz0};
  const int N = NARX_DISC_N;

  for (int stage = 0; stage <= N; ++stage)
  {
    narx_disc_acados_update_params(capsule_, stage, p, 2);
  }
}

bool NarxAcadosSolver::solve()
{
  if (!capsule_) return false;
  int status = narx_disc_acados_solve(capsule_);
  return (status == 0);
}

bool NarxAcadosSolver::isOk() const
{
  return (capsule_ != nullptr);
}

bool NarxAcadosSolver::getU0(double &a, double &b) const
{
  if (!capsule_) return false;

  ocp_nlp_config *nlp_config = narx_disc_acados_get_nlp_config(capsule_);
  ocp_nlp_dims   *nlp_dims   = narx_disc_acados_get_nlp_dims(capsule_);
  ocp_nlp_out    *nlp_out    = narx_disc_acados_get_nlp_out(capsule_);

  double u0[ NARX_DISC_NU ] = {0.0};
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0);

  // うちのモデルは u=[a,b]
  a = std::min(1.0, std::max(0.0, u0[0]));
  b = std::min(1.0, std::max(0.0, u0[1]));
  return true;
}

bool NarxAcadosSolver::getControl(double &p1, double &p2) const
{
  double a, b;
  if (!getU0(a, b)) return false;

  const double ps = pmax_ * (a + b);
  const double pd = pmax_ * (a - b);
  p1 = 0.5 * (ps + pd);
  p2 = 0.5 * (ps - pd);
  return true;
}

int NarxAcadosSolver::nx() const
{
  return NARX_DISC_NX;
}

int NarxAcadosSolver::nu() const
{
  return NARX_DISC_NU;
}

int NarxAcadosSolver::N() const
{
  return NARX_DISC_N;
}

}  // namespace mpa_acados_controller
