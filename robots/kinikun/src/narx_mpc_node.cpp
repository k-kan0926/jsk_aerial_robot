#include "mpa_acados_controller/narx_mpc_solver.hpp"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <deque>

using mpa_acados::NarxMpcSolver;
using mpa_acados::NarxParams;

// 角度の一次遅れLPF
struct Lpf {
  double y{0.0}; bool init{false};
  double step(double x, double dt, double tau){
    if (!init){ y=x; init=true; return y; }
    double a = std::max(0.0, std::min(1.0, dt/std::max(1e-3, tau)));
    y += a*(x - y);
    return y;
  }
};

class NarxMpcNode {
public:
  NarxMpcNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
    // Parameters
    pnh_.param<std::string>("joint_topic", joint_topic_, "/kinikun1/joint_states");
    pnh_.param<std::string>("joint_name",  joint_name_,  "arm1_joint");
    pnh_.param<std::string>("target_topic", target_topic_, "/target_angle");
    pnh_.param<std::string>("pub_topic_p12", pub_topic_p12_, "/p1p2_cmd");
    pnh_.param<std::string>("pub_topic_counts", pub_topic_counts_, "/mpa_cmd");
    pnh_.param<bool>("also_pub_counts", also_pub_counts_, true);

    pnh_.param("ctrl_rate_hz", ctrl_rate_hz_, 100.0);
    pnh_.param("target_in_deg", target_in_deg_, true);

    pnh_.param("pmax", prm_.pmax, 0.7);
    pnh_.param("dt",   prm_.dt,   0.02);
    pnh_.param("L",    prm_.L,    8);
    pnh_.param("a0",   prm_.a0,   0.0);
    pnh_.param("a1",   prm_.a1,   0.0);
    pnh_.param("a2",   prm_.a2,   0.0);

    pnh_.param("tau_theta", tau_theta_, 0.08);
    pnh_.param("tau_cmd",   tau_cmd_,   0.10);
    pnh_.param("rate_sigma", rate_sigma_, 0.5);
    pnh_.param("rate_delta", rate_delta_, 0.5);

    pnh_.param("raw_counts_per_MPa", raw_counts_per_MPa_, 4096.0/0.9);

    std::string models_root;
    pnh_.param<std::string>("models_root", models_root, "models/narx_disc");

    // init solver
    if (!solver_.init(models_root, prm_)) {
      throw std::runtime_error("Failed to init acados solver");
    }

    // 初期状態バッファ（ps0=0.4, pd0=0）
    double ps0=0.40, pd0=0.0, theta0=0.0;
    make_x0(theta0, ps0, pd0, xbuf_);
    solver_.set_x0(xbuf_);
    solver_.set_uniform_initial_u(ps0, pd0);

    theta_ref_ = 0.0;  // [rad]
    theta_filt_init_ = false;

    // ROS I/O
    sub_joint_  = nh_.subscribe(joint_topic_, 10, &NarxMpcNode::cbJoint, this);
    sub_target_ = nh_.subscribe(target_topic_, 10, &NarxMpcNode::cbTarget, this);
    pub_p12_    = nh_.advertise<geometry_msgs::Vector3>(pub_topic_p12_, 10);
    if (also_pub_counts_)
      pub_counts_ = nh_.advertise<geometry_msgs::Vector3>(pub_topic_counts_, 10);

    // timer loop
    timer_ = nh_.createTimer(ros::Duration(1.0/std::max(1.0, ctrl_rate_hz_)),
                             &NarxMpcNode::onTimer, this);
    ROS_INFO("[narx_mpc_node] ready. rate=%.1fHz", ctrl_rate_hz_);
  }

private:
  // callbacks
  void cbJoint(const sensor_msgs::JointState& msg){
    for (size_t i=0;i<msg.name.size();++i){
      if (msg.name[i]==joint_name_){
        theta_meas_ = msg.position[i]; // [rad]
        theta_ok_ = true;
        return;
      }
    }
  }

  void cbTarget(const std_msgs::Float32& msg){
    double val = msg.data;
    theta_ref_ = target_in_deg_ ? (val*M_PI/180.0) : val;
    // replanは solver 1shot なので毎周期パラメータに反映するだけでOK
  }

  // timer
  void onTimer(const ros::TimerEvent& ev){
    if (!theta_ok_) return;
    const double dt = (ev.current_real - ev.last_real).toSec();
    // LPF
    double theta_f = lpf_theta_.step(theta_meas_, dt>0?dt:1.0/ctrl_rate_hz_, tau_theta_);

    // dz0: 静的 z(ps) の原点（例：初期姿勢を基準にする）
    // ここでは ps = xbuf_[1] を使って dz_now = a0 + a1*ps + a2*ps^2 とし、
    // その「基準」を最初に記録
    if (!dz0_init_){
      double ps = xbuf_[1];
      dz0_ = prm_.a0 + prm_.a1*ps + prm_.a2*ps*ps;
      dz0_init_ = true;
    }

    // パラメータ反映
    solver_.set_parameters(theta_ref_, dz0_);

    // x0 更新：先頭スライス s0 を最新に作り直し、バッファを前詰め
    shift_and_push_x(theta_f);

    // x0 を solver に渡す
    solver_.set_x0(xbuf_);

    // 初期入力推定を直近の ps,pd から与える（xbuf_ の s0 を使用）
    double ps_prev = xbuf_[1], pd_prev = xbuf_[2];
    solver_.set_uniform_initial_u(ps_prev, pd_prev);

    // 解く
    int status = solver_.solve();
    if (status) {
      ROS_WARN_THROTTLE(1.0, "[acados] solve status=%d", status);
    }

    // u0 取り出し → ps,pd → p1,p2
    double ps, pd; solver_.get_first_control(ps, pd);
    // rate-limit & 一次遅れでスムージング
    ps_cmd_ = rate_limit(exp_filter(ps_cmd_, ps, dt, tau_cmd_), ps_cmd_, rate_sigma_, dt);
    pd_cmd_ = rate_limit(exp_filter(pd_cmd_, pd, dt, tau_cmd_), pd_cmd_, rate_delta_, dt);

    // 可行領域 box clamp
    clamp_box(ps_cmd_, pd_cmd_, prm_.pmax);

    double p1 = 0.5*(ps_cmd_ + pd_cmd_);
    double p2 = 0.5*(ps_cmd_ - pd_cmd_);

    // publish
    geometry_msgs::Vector3 v; v.x=p1; v.y=p2; v.z=0.0;
    pub_p12_.publish(v);

    if (also_pub_counts_) {
      geometry_msgs::Vector3 c;
      c.x = std::round(p1 * raw_counts_per_MPa_);
      c.y = std::round(p2 * raw_counts_per_MPa_);
      c.z = 0.0;
      pub_counts_.publish(c);
    }
  }

  // helpers
  static void clamp_box(double& ps, double& pd, double pmax){
    // p1=0.5*(ps+pd), p2=0.5*(ps-pd) が [0,pmax]
    double p1 = 0.5*(ps+pd);
    double p2 = 0.5*(ps-pd);
    p1 = std::max(0.0, std::min(pmax, p1));
    p2 = std::max(0.0, std::min(pmax, p2));
    ps = p1 + p2;
    pd = p1 - p2;
  }

  static double rate_limit(double target, double current, double rate_max, double dt){
    double step = target - current;
    double lim = rate_max * std::max(dt, 1e-3);
    if (step >  lim) step =  lim;
    if (step < -lim) step = -lim;
    return current + step;
  }

  static double exp_filter(double current, double target, double dt, double tau){
    double a = std::max(0.0, std::min(1.0, dt/std::max(1e-3, tau)));
    return current + a*(target - current);
  }

  void make_x0(double theta0, double ps0, double pd0, std::vector<double>& x0){
    x0.assign(6*prm_.L, 0.0);
    double dps=0.0, dpd=0.0;
    double dz = 0.0; // s0 は相対値で保持
    // s0
    x0[0] = theta0; x0[1]=ps0; x0[2]=pd0; x0[3]=dps; x0[4]=dpd; x0[5]=dz;
    // s1..s_{L-1} は s0 をコピー
    for (int j=1;j<prm_.L;++j){
      int off = 6*j;
      for (int k=0;k<6;++k) x0[off+k] = x0[k];
    }
    ps_cmd_ = ps0; pd_cmd_ = pd0;
  }

  void shift_and_push_x(double theta_now){
    // 直近の ps, pd
    double ps_prev = xbuf_[1];
    double pd_prev = xbuf_[2];
    // 今周期の dps,dpd
    double dps = (ps_cmd_ - ps_prev)/std::max(prm_.dt, 1e-4);
    double dpd = (pd_cmd_ - pd_prev)/std::max(prm_.dt, 1e-4);
    // dz_now = (a0 + a1*ps_cmd + a2*ps_cmd^2) - dz0_
    double dz_now = (prm_.a0 + prm_.a1*ps_cmd_ + prm_.a2*ps_cmd_*ps_cmd_) - dz0_;

    // 新スライス s_next
    double s[6] = {theta_now, ps_cmd_, pd_cmd_, dps, dpd, dz_now};

    // バッファ前詰め： [s0,...,s_{L-2}]←[s_next] , s_{L-1} 落とす
    // 先頭に s_next を入れ、残りを後ろにシフト
    std::vector<double> xnew(6*prm_.L, 0.0);
    for (int k=0;k<6;++k) xnew[k] = s[k];
    // 残りコピー
    for (int j=0;j<prm_.L-1;++j){
      for (int k=0;k<6;++k){
        xnew[6*(j+1)+k] = xbuf_[6*j + k];
      }
    }
    xbuf_.swap(xnew);
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_joint_, sub_target_;
  ros::Publisher pub_p12_, pub_counts_;
  ros::Timer timer_;

  NarxMpcSolver solver_;
  NarxParams prm_;
  std::vector<double> xbuf_; // 6*L

  // state
  bool theta_ok_{false};
  double theta_meas_{0.0};
  double theta_ref_{0.0};
  bool target_in_deg_{true};

  double ps_cmd_{0.4}, pd_cmd_{0.0};
  double tau_theta_{0.08}, tau_cmd_{0.10};
  double rate_sigma_{0.5}, rate_delta_{0.5};

  bool dz0_init_{false};
  double dz0_{0.0};

  double ctrl_rate_hz_{100.0};
  bool also_pub_counts_{true};
  double raw_counts_per_MPa_{4096.0/0.9};

  std::string joint_topic_, joint_name_, target_topic_;
  std::string pub_topic_p12_, pub_topic_counts_;

  Lpf lpf_theta_;
  bool theta_filt_init_{false};
};

int main(int argc, char** argv){
  ros::init(argc, argv, "narx_mpc_controller");
  ros::NodeHandle nh, pnh("~");
  try{
    NarxMpcNode node(nh, pnh);
    ros::spin();
  }catch(const std::exception& e){
    ROS_FATAL("Exception: %s", e.what());
  }
  return 0;
}
