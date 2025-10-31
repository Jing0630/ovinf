#ifndef POLICY_CONTROLLER_HPP
#define POLICY_CONTROLLER_HPP

#include <chrono>
#include "controller/controller_base.hpp"
#include "filter/filter_mean.hpp"
#include "ovinf/ovinf_factory.hpp"

namespace ovinf {

class PolicyController : public ControllerBase<float> {
 public:
  using Ptr = std::shared_ptr<PolicyController>;

  PolicyController() = delete;
  ~PolicyController() = default;

  PolicyController(RobotBase<float>::RobotPtr robot, YAML::Node const& config)
      : ControllerBase<float>(robot, config),
        decimation_(config["decimation"].as<int>()) {
    // ----------------------------
    // 1️⃣ 读取 PD 增益
    // ----------------------------
    p_gains_ = VectorT::Zero(robot_->joint_size_);
    d_gains_ = VectorT::Zero(robot_->joint_size_);

    for (auto const& pair : robot_->joint_names_) {
      p_gains_(pair.second) = config["p_gains"][pair.first].as<float>();
      d_gains_(pair.second) = config["d_gains"][pair.first].as<float>();
    }

    // ----------------------------
    // 2️⃣ 感知模式
    // ----------------------------
    perception_enabled_ = config["perception_enabled"]
                              ? config["perception_enabled"].as<bool>()
                              : false;

    // ----------------------------
    // 3️⃣ 读取策略类型（与Factory一致）
    // ----------------------------
    if (config["inference"]["policy_type"]) {
      policy_type_ = config["inference"]["policy_type"].as<std::string>();
    } else {
      policy_type_ = "Unknown";
    }

    // ----------------------------
    // 4️⃣ 创建推理网络
    // ----------------------------
    inference_net_ = ovinf::PolicyFactory::CreatePolicy(config["inference"]);

    command_ = VectorT::Zero(3);
    counter_ = 0;
  }

  // ----------------------------
  // WarmUp
  // ----------------------------
  virtual void WarmUp() final {
    if (counter_++ % decimation_ != 0) return;

    if (IsRunPolicy()) {
      // 🟦 Run类策略使用14关节 + 调换顺序
      Eigen::VectorXf q = robot_->Observer()->JointActualPosition().segment(0, 14);
      Eigen::VectorXf qd = robot_->Observer()->JointActualVelocity().segment(0, 14);
      Eigen::VectorXf q_used(14), qd_used(14);
      int order[14] = {12,0,1,2,3,4,5,13,6,7,8,9,10,11};
      for (int i = 0; i < 14; ++i) {
        q_used[i]  = q[order[i]];
        qd_used[i] = qd[order[i]];
      }

      RunInferenceWarmUp(q_used, qd_used);
    } else {
      // 🟩 其他策略使用12关节
      Eigen::VectorXf q = robot_->Observer()->JointActualPosition().segment(0, 12);
      Eigen::VectorXf qd = robot_->Observer()->JointActualVelocity().segment(0, 12);
      RunInferenceWarmUp(q, qd);
    }
  }

  // ----------------------------
  // Init
  // ----------------------------
  virtual void Init() final {
    counter_ = 0;
    command_ = VectorT::Zero(3);
    ready_ = true;
  }

  // ----------------------------
  // Step
  // ----------------------------
  virtual void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "PolicyController not ready" << std::endl;
      return;
    }

    if (counter_++ % decimation_ == 0) {
      if (IsRunPolicy()) {
        Eigen::VectorXf q = robot_->Observer()->JointActualPosition().segment(0, 14);
        Eigen::VectorXf qd = robot_->Observer()->JointActualVelocity().segment(0, 14);
        Eigen::VectorXf q_used(14), qd_used(14);
        int order[14] = {12,0,1,2,3,4,5,13,6,7,8,9,10,11};
        for (int i = 0; i < 14; ++i) {
          q_used[i]  = q[order[i]];
          qd_used[i] = qd[order[i]];
        }

        RunInferenceStep(q_used, qd_used);
      } else {
        Eigen::VectorXf q = robot_->Observer()->JointActualPosition().segment(0, 12);
        Eigen::VectorXf qd = robot_->Observer()->JointActualVelocity().segment(0, 12);
        RunInferenceStep(q, qd);
      }
    }

    if (set_target) {
      auto target_pos = inference_net_->GetResult();
      if (target_pos.has_value()) {
        if (IsRunPolicy()) {
          // 反向映射
          Eigen::VectorXf mapped(14);
          int order[14] = {12,0,1,2,3,4,5,13,6,7,8,9,10,11};
          for (int i = 0; i < 14; ++i) mapped[order[i]] = target_pos.value()[i];
          for (size_t i = 0; i < 14; i++)
            robot_->Executor()->JointTargetPosition()[i] = mapped[i];
          for (size_t i = 14; i < robot_->joint_size_; i++)
            robot_->Executor()->JointTargetPosition()[i] = 0.0;
        } else {
          for (size_t i = 0; i < 12; i++)
            robot_->Executor()->JointTargetPosition()[i] = target_pos.value()[i];
          for (size_t i = 12; i < robot_->joint_size_; i++)
            robot_->Executor()->JointTargetPosition()[i] = 0.0;
        }
      }
      ComputeJointPd();
    }
  }

  // ----------------------------
  // Stop
  // ----------------------------
  virtual void Stop() final {
    command_ = VectorT::Zero(3);
    ready_ = false;
  }

  VectorT& GetCommand() { return command_; }

 private:
  // 工具函数：判断是否为 Run 策略
  inline bool IsRunPolicy() const {
    return (policy_type_ == "HumanoidRun");
  }

  // 工具函数：统一封装 WarmUp/Step 调用逻辑
  void RunInferenceWarmUp(const Eigen::VectorXf& q, const Eigen::VectorXf& qd) {
    if (perception_enabled_) {
      inference_net_->WarmUp(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = q,
           .joint_vel = qd,
           .scan = robot_->Observer()->Scan()});
    } else {
      inference_net_->WarmUp(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = q,
           .joint_vel = qd});
    }
  }

  void RunInferenceStep(const Eigen::VectorXf& q, const Eigen::VectorXf& qd) {
    if (perception_enabled_) {
      inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = q,
           .joint_vel = qd,
           .scan = robot_->Observer()->Scan()});
    } else {
      inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = q,
           .joint_vel = qd});
    }
  }

 private:
  const int decimation_;
  bool perception_enabled_ = false;
  std::string policy_type_;

  VectorT command_;
  size_t counter_ = 0;
  ovinf::BasePolicy<float>::BasePolicyPtr inference_net_;
};

}  // namespace ovinf

#endif  // !POLICY_CONTROLLER_HPP
