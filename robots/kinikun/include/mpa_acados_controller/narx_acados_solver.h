#pragma once

#include <vector>
extern "C" {
#include "c_generated_code/acados_solver_narx_disc.h"
}
#include "acados_solver_narx_disc.h"   // 生成コードのヘッダ

namespace mpa_acados_controller
{

class NarxAcadosSolver
{
public:
  NarxAcadosSolver();
  ~NarxAcadosSolver();

  // acadosのcapsuleをつくる
  bool init(double pmax);

  // ノード側から「初期状態バッファ」を流し込む
  void setX0(const std::vector<double> &x0);

  // 参照: (theta_ref, dz0) を全ステージへ
  void setParams(double theta_ref, double dz0);

  // MPCを1回解く
  bool solve();

  // ノードが欲しがっていた形
  bool isOk() const;

  // ノードが使っていた getU0(a,b) も用意しておく
  bool getU0(double &a, double &b) const;

  // こっちは最終的な p1,p2 が欲しいとき用
  bool getControl(double &p1, double &p2) const;

  int nx() const;
  int nu() const;
  int N()  const;

private:
  narx_disc_solver_capsule *capsule_;
  double pmax_;
};

}  // namespace mpa_acados_controller