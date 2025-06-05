#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h> 
#include <gtsam/inference/Symbol.h>

#include <GeographicLib/UTMUPS.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <cmath>
#include <memory>
#include <functional>
#include <queue>

using namespace std;
using namespace gtsam;

class FactorGraphNode : public rclcpp::Node
{
public:
  FactorGraphNode() : Node("factor_graph_node")
  {
    RCLCPP_INFO(this->get_logger(),"factor_graph_node started");

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, bind(&FactorGraphNode::imuCallback, this, placeholders::_1));
    encoder_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("encoder/twist", 10, bind(&FactorGraphNode::preintegrateImu, this, placeholders::_1));
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("Ublox_gps/fix", 10, bind(&FactorGraphNode::gpsCallback, this, placeholders::_1));
    imu_variance_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "imu/variance", 10, std::bind(&FactorGraphNode::varianceCallback, this, std::placeholders::_1));

    local_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("odometry/local",10);
    global_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("odometry/global",10);
    
    local_odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    global_odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    graph_ = gtsam::NonlinearFactorGraph();
    initial_estimate_ = gtsam::Values();
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    isam2_ = gtsam::ISAM2(params);
    prev_pose_ = gtsam::Pose3();
    prev_velocity_ = gtsam::Vector3(0,0,0);

    imuCovarianceParams = PreintegrationParams::MakeSharedU(9.80665);
    // acc_sigma = 0.02483647066;
    // gyro_sigma = 0.15130551861;
    if (imu_variance_received_) {
      Matrix3 I_3x3 = Matrix3::Identity();
      imuCovarianceParams->accelerometerCovariance = I_3x3 * imu_variance_[0];
      imuCovarianceParams->gyroscopeCovariance = I_3x3 * imu_variance_[3];
    } else {
      Matrix3 I_3x3 = Matrix3::Identity();
      imuCovarianceParams->accelerometerCovariance = I_3x3 * pow(0.02483647066,2);
      imuCovarianceParams->gyroscopeCovariance = I_3x3 * pow(0.15130551861,2);
    }
    imuCovarianceParams->integrationCovariance = Matrix3::Identity() * 1e-8;
    imu_count_ = 0;

    Pose3 initial_pose = Pose3();  // Identity by default
    Vector3 initial_velocity(0.0, 0.0, 0.0);
    imuBias::ConstantBias initial_bias;  // Zero bias

    // Add prior factors
    auto pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1e-2), Vector3::Constant(1e-2)).finished());
    auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-2);
    auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);

    graph_.add(PriorFactor<Pose3>(symbol('p', 0), initial_pose, pose_noise));
    graph_.add(PriorFactor<Vector3>(symbol('v', 0), initial_velocity, velocity_noise));
    graph_.add(PriorFactor<imuBias::ConstantBias>(symbol('b', 0), initial_bias, bias_noise));

    // Add initial estimate
    initial_estimate_.insert(symbol('p', 0), initial_pose);
    initial_estimate_.insert(symbol('v', 0), initial_velocity);
    initial_estimate_.insert(symbol('b', 0), initial_bias);

    // Initialize ISAM2
    isam2_.update(graph_, initial_estimate_);
    graph_.resize(0);
    initial_estimate_.clear();

    // Save previous states
    prev_pose_ = initial_pose;
    prev_velocity_ = initial_velocity;
    prev_bias_ = initial_bias;
    prev_state_ = NavState(initial_pose, initial_velocity);

    // 초기 시간도 설정 필요
    prev_imu_time_ = this->now();
  }

private:
  // loop closure에 맞도록 다시 수정하기
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  gtsam::ISAM2 isam2_;
  gtsam::Pose3 prev_pose_;
  gtsam::Vector3 prev_velocity_;

  gtsam::Vector3 encoder_velocity_;

  int keyframe_idx_ = 0;
  gtsam::Symbol prev_pose_key_;
  gtsam::Symbol prev_velocity_key_;
  gtsam::Symbol prev_bias_key_;
  gtsam::Symbol curr_pose_key_;
  gtsam::Symbol curr_velocity_key_;
  gtsam::Symbol curr_bias_key_;

  bool gps_origin_initialized_ = false;

  double utm_origin_x_ = 0.0;
  double utm_origin_y_ = 0.0;

  std::map<rclcpp::Time, size_t> keyframe_idx_map_;

  std::map<rclcpp::Time, rclcpp::Time> keyframe_timestamps_;
  std::map<rclcpp::Time, size_t> keyframe_index_map_;

  // declare subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imu_variance_sub_;

  size_t imu_count_;
  deque<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr local_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_pub_;

  // init imu_bias
  imuBias::ConstantBias imu_bias_;

  shared_ptr<PreintegrationParams> imuCovarianceParams;
  shared_ptr<PreintegratedImuMeasurements> preIntegratedImu;

  std::shared_ptr<tf2_ros::TransformBroadcaster> local_odom_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> global_odom_broadcaster_;

  std::thread factor_graph_thread_;
  std::mutex factor_graph_mutex_;
  std::condition_variable factor_condition_variable_;
  std::queue<std::function<void()>> factor_graph_queue_;
  bool thread_stop_flag_ = false;
  double acc_sigma;
  double gyro_sigma;

  // GTSAM 관련 상태 변수
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::NavState prev_state_;

  // 이전 IMU 메시지 시간 (시간 간격 계산용)
  rclcpp::Time prev_imu_time_;

  std::array<double, 6> imu_variance_;
  bool imu_variance_received_ = false;

  // imu callback func
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg){
    imu_buf_.push_back(imu_msg);

    const double buffer_duration_sec = 1.0;
    rclcpp::Time timestamp_now = imu_msg->header.stamp;
    while (!imu_buf_.empty() && (timestamp_now - imu_buf_.front()->header.stamp).seconds() > buffer_duration_sec)
    {
      imu_buf_.pop_front();
    }

    imu_count_++;
    if (imu_count_ % 1000 == 0){
      RCLCPP_INFO(this->get_logger(), "Received 1000 IMU messages");
      imu_count_ = 0;
    }
  }

  // create keyframe when encoder topic has arrived
  void preintegrateImu(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr encoder_msg){
    if (imu_buf_.size() < 2) return; // imu_buff_ must have more than 2 imu topics

    imuCovarianceParams = PreintegrationParams::MakeSharedU(9.80665);
    if (imu_variance_received_) {
      Matrix3 I_3x3 = Matrix3::Identity();
      imuCovarianceParams->accelerometerCovariance = I_3x3 * imu_variance_[0];
      imuCovarianceParams->gyroscopeCovariance = I_3x3 * imu_variance_[3];
    }

    rclcpp::Time prev_time = imu_buf_.front()->header.stamp;
    rclcpp::Time encoder_time = encoder_msg->header.stamp;
    preIntegratedImu = std::make_shared<PreintegratedImuMeasurements>(imuCovarianceParams, imu_bias_);

    for(size_t i = 1; i<imu_buf_.size(); i++)
    {
      auto imu_msg = imu_buf_[i];
      rclcpp::Time curr_time = imu_msg->header.stamp;
      if(curr_time > encoder_time) break;
      rclcpp::Time t_curr(imu_buf_[i]->header.stamp);
      rclcpp::Time t_prev(imu_buf_[i-1]->header.stamp);
      double dt = (t_curr - t_prev).seconds();
      prev_time = curr_time;

      Vector3 accel(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z
      );
      Vector3 gyro(
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z
      );
      preIntegratedImu->integrateMeasurement(accel, gyro, dt);
    }

    gtsam::NavState prev_state(prev_pose_,prev_velocity_);
    gtsam::NavState estimated_state = preIntegratedImu->predict(prev_state,imu_bias_);

    gtsam::Symbol prev_pose_key('p',keyframe_idx_);
    gtsam::Symbol prev_velocity_key('v',keyframe_idx_);
    gtsam::Symbol prev_bias_key('b',keyframe_idx_);
    gtsam::Symbol curr_pose_key('p',keyframe_idx_+1);
    gtsam::Symbol curr_velocity_key('v',keyframe_idx_+1);
    gtsam::Symbol curr_bias_key('b',keyframe_idx_+1);    

    auto imu_factor = gtsam::ImuFactor(prev_pose_key, prev_velocity_key, curr_pose_key, curr_velocity_key, prev_bias_key, *preIntegratedImu);
    graph_.add(imu_factor);

    // Ackermann model 적용
    double L = 0.72;  // 차량 휠베이스
    double v = hypot(encoder_msg->twist.twist.linear.x, encoder_msg->twist.twist.linear.y);
    double delta = encoder_msg->twist.twist.angular.z;  // 조향각(rad), 이 값은 조향 각도로 입력되어야 함

    if (encoder_msg->twist.twist.linear.x < 0)
      v *= -1;
    double theta_dot = (v / L) * tan(delta);  // 회전률

    // 추정된 각속도 벡터 적용 (Z 축만 회전한다고 가정)
    encoder_velocity_ = Vector3(v, 0.0, 0.0);  // 전진 속도만 적용
    Vector3 angular_velocity(0.0, 0.0, theta_dot);

    // 노이즈 모델 설정 및 factor 추가
    auto dynamic_sigma = std::max(0.05, 0.02 * v);  
    auto encoder_noise = gtsam::noiseModel::Isotropic::Sigma(3, dynamic_sigma);

    graph_.add(gtsam::PriorFactor<gtsam::Vector3>(curr_velocity_key, encoder_velocity_, encoder_noise));

    auto bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);
    graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(prev_bias_key, curr_bias_key, gtsam::imuBias::ConstantBias(),bias_noise_model)); // bias 사이에 between factor로 noise_model을 삽입
    initial_estimate_.insert(curr_pose_key, estimated_state.pose());
    initial_estimate_.insert(curr_velocity_key, estimated_state.v());
    initial_estimate_.insert(curr_bias_key,imu_bias_);
    
    keyframe_idx_map_.insert({encoder_msg->header.stamp, keyframe_idx_});
    keyframe_idx_++;

    isam2_.update(graph_, initial_estimate_);
    
    gtsam::Values result = isam2_.calculateEstimate();

    prev_pose_ = result.at<gtsam::Pose3>(curr_pose_key);
    prev_velocity_ = result.at<gtsam::Vector3>(curr_velocity_key);
    imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(curr_bias_key);

    graph_.resize(0);
    initial_estimate_.clear();

    geometry_msgs::msg::PoseWithCovarianceStamped local_pose_msg;
    local_pose_msg.header.stamp = encoder_msg->header.stamp;

    auto q = prev_pose_.rotation().toQuaternion();
    
    local_pose_msg.pose.pose.position.x = prev_pose_.translation().x();
    local_pose_msg.pose.pose.position.y = prev_pose_.translation().y();
    local_pose_msg.pose.pose.position.z = prev_pose_.translation().z();

    local_pose_msg.pose.pose.orientation.x = q.x();
    local_pose_msg.pose.pose.orientation.y = q.y();
    local_pose_msg.pose.pose.orientation.z = q.z();
    local_pose_msg.pose.pose.orientation.w = q.w();
    
    local_pose_msg.header.frame_id = "odom";
    local_pose_pub_->publish(local_pose_msg);

    geometry_msgs::msg::TransformStamped local_odom_tf;
    local_odom_tf.header.stamp = encoder_msg->header.stamp;
    local_odom_tf.header.frame_id = "odom";
    local_odom_tf.child_frame_id = "base_link";

    local_odom_tf.transform.translation.x = prev_pose_.translation().x();
    local_odom_tf.transform.translation.y = prev_pose_.translation().y();
    local_odom_tf.transform.translation.z = prev_pose_.translation().z();
    
    local_odom_tf.transform.rotation.x = q.x();
    local_odom_tf.transform.rotation.y = q.y();
    local_odom_tf.transform.rotation.z = q.z();
    local_odom_tf.transform.rotation.w = q.w();

    local_odom_broadcaster_->sendTransform(local_odom_tf);
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
  {
    double latitude = gps_msg->latitude;
    double longitude = gps_msg->longitude;
    
    double x,y;
    int zone;
    bool northp;
    
    try{
      GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x,y);
    }
    catch (const exception& e)
    {
      RCLCPP_WARN(this->get_logger(),"UTM convert failed: %s", e.what());
      return;
    }

    if(!gps_origin_initialized_)
    {
      utm_origin_x_ = x;
      utm_origin_y_ = y;

      gps_origin_initialized_ = true;
    }

    double local_x = x - utm_origin_x_; 
    double local_y = y - utm_origin_y_;

    gtsam::Pose3 gps_position(gtsam::Rot3(), gtsam::Point3(local_x, local_y, 0.0));
    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 999, 999, 999).finished());  // 6DOF Pose3

    rclcpp::Time gps_key_time = gps_msg->header.stamp;
    gtsam::Symbol curr_pose_key('p', keyframe_idx_);

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(curr_pose_key, gps_position, gps_noise));
    isam2_.update(graph_, initial_estimate_);
    graph_.resize(0);
    initial_estimate_.clear();

    gtsam::Values result = isam2_.calculateEstimate();
    gtsam::Pose3 gps_pose = result.at<gtsam::Pose3>(curr_pose_key);
    auto q = gps_pose.rotation().toQuaternion();

    geometry_msgs::msg::PoseWithCovarianceStamped global_pose_msg;
    global_pose_msg.header.stamp = gps_msg->header.stamp;

    global_pose_msg.pose.pose.position.x = x;
    global_pose_msg.pose.pose.position.y = y;
    global_pose_msg.pose.pose.position.z = 0.0;

    global_pose_msg.pose.pose.orientation.x = q.x();
    global_pose_msg.pose.pose.orientation.y = q.y();
    global_pose_msg.pose.pose.orientation.z = q.z();
    global_pose_msg.pose.pose.orientation.w = q.w();

    global_pose_msg.header.frame_id = "map";
    global_pose_pub_->publish(global_pose_msg);

    geometry_msgs::msg::TransformStamped global_map_tf;
    global_map_tf.header.stamp = gps_msg->header.stamp;
    global_map_tf.header.frame_id = "map";
    global_map_tf.child_frame_id = "odom";

    global_map_tf.transform.translation.x = x - utm_origin_x_;
    global_map_tf.transform.translation.y = y - utm_origin_y_;
    global_map_tf.transform.translation.z = 0.0;
    
    global_map_tf.transform.rotation.x = q.x();
    global_map_tf.transform.rotation.y = q.y();
    global_map_tf.transform.rotation.z = q.z();
    global_map_tf.transform.rotation.w = q.w();

    global_odom_broadcaster_->sendTransform(global_map_tf);
    keyframe_idx_++;
  }

  size_t findClosestKeyframe(const rclcpp::Time& stamp) {
    if (keyframe_index_map_.empty()) return 0;

    auto it = keyframe_index_map_.lower_bound(stamp);
    if (it == keyframe_index_map_.begin()) return it->second;
    if (it == keyframe_index_map_.end()) return std::prev(it)->second;

    auto after = it;
    auto before = std::prev(it);
    auto diff_after = (after->first - stamp).nanoseconds();
    auto diff_before = (stamp - before->first).nanoseconds();

    return (diff_after < diff_before) ? after->second : before->second;
  }

  rclcpp::Time getKeyframeTimestamp(const rclcpp::Time& stamp)
  {
    if (keyframe_timestamps_.empty()) return rclcpp::Time(0);

    auto it = keyframe_index_map_.lower_bound(stamp);

    if (it == keyframe_index_map_.begin()) return it->first;
    if (it == keyframe_index_map_.end()) return std::prev(it)->first;

    auto after = it;
    auto before = std::prev(it);

    auto diff_after = (after->first - stamp).nanoseconds();
    auto diff_before = (stamp - before->first).nanoseconds();

    return (diff_after < diff_before) ? after->first : before->first;
  }

  void varianceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 6) return;
    for (size_t i = 0; i < 6; ++i) {
      imu_variance_[i] = msg->data[i];
    }
    imu_variance_received_ = true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FactorGraphNode>());
  rclcpp::shutdown();
  return 0;
}