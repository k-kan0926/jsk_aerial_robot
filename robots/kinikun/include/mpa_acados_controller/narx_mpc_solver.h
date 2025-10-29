#pragma once
#include <string>
#include <vector>
#include <memory>

// acados generated headers
extern "C" {
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

// ↓ 生成物のパスに合わせてインクルード
#include "acados_solver_narx_disc.h"
}

namespace mpa_acados {

struct NarxParams {
  // オフラインで把握済みのスカラー
  double dt;         // サンプル周期 [s]
  double pmax;       // [MPa]
  int    L;          // lags
  // z ~ a0 + a1*ps + a2*ps^2 の係数（静的高さ抑制の基準）
  double a0{0.0}, a1{0.0}, a2{0.0};
};

class NarxMpcSolver {
public:
  NarxMpcSolver() = default;
  ~NarxMpcSolver();

  // 生成Cのソルバを初期化。models_root は c_generated_code の親（例: models/narx_disc）
  bool init(const std::string& models_root, const NarxParams& prm);

  // パラメータ（theta_ref[rad], dz0[m]）をステージ＆終端に設定
  void set_parameters(double theta_ref, double dz0);

  // x0（6*L）を設定：s0=[theta, ps, pd, dps, dpd, dz] の縦積み（s0, s1, ..., s_{L-1})
  // s0 が「最新」。以降は古い順。
  void set_x0(const std::vector<double>& x0);

  // 初期入力推定（全ステージ同じ値で埋める）
  void set_uniform_initial_u(double ps0, double pd0);

  // 1回だけ解く（SQP-RTI）
  int solve();

  // k=0 の u = [a,b] を ps,pd に変換して返す
  void get_first_control(double& ps, double& pd) const;

  // OCP 地平線長を返す（acados 生成物に依存）
  int N() const { return narx_disc_N; }

private:
  NarxParams prm_;
  bool inited_{false};
};

} // namespace mpa_acados
