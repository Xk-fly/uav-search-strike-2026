#include "nodes/perception/balloon_observe.h"
#include "utils/pose_utils.h"
#include <algorithm>
#include <cmath>
namespace {
constexpr double PI = 3.14159265358979323846;
double gap(const geometry_msgs::Point&a,const geometry_msgs::Point&b) {
  return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);
}
}

BalloonObserve::BalloonObserve(const std::string& name,const NodeConfig& config)
 : DroneActionNode(name,config) {
  status_pub_=mct->getROSHandle()->advertise<std_msgs::String>("/balloon_search/status",20);
}
PortsList BalloonObserve::providedPorts() {
  return {InputPort<std::string>("mode","front","front旋转扫描/down下视观察"),
          InputPort<std::string>("object_name","目标气球类别"),
          InputPort<double>("confidence_threshold",0.75,"置信度"),
          InputPort<double>("scan_yaw_rate",0.30,"扫描最大偏航速率rad/s"),
          InputPort<double>("timeout",26.0,"最长观测秒数"),
          InputPort<double>("max_drift",0.50,"原地旋转最大位移m")};
}
void BalloonObserve::status(const std::string& tag) {
  std_msgs::String msg; msg.data=mode_+":"+target_+":"+tag; status_pub_.publish(msg);
  ROS_INFO("[%s] %s",name().c_str(),msg.data.c_str());
}
void BalloonObserve::objectsCallback(const common_msgs::Objects::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_); objects_=*msg; ++rx_seq_;
}
void BalloonObserve::stopDown() {
  if(!down_on_) return;
  common_msgs::DetEnable msg; msg.header.stamp=ros::Time::now();
  msg.is_open=false; down_control_pub_.publish(msg); down_on_=false;
}
NodeStatus BalloonObserve::onStart() {
  mode_=getInput<std::string>("mode").value();
  if(mode_!="front" && mode_!="down") throw BT::RuntimeError("BalloonObserve bad mode");
  auto target_res=getInput<std::string>("object_name");
  if(!target_res||target_res.value().empty())throw BT::RuntimeError("BalloonObserve missing object_name");
  target_=target_res.value();
  confidence_=getInput<double>("confidence_threshold").value();
  yaw_rate_=getInput<double>("scan_yaw_rate").value();
  timeout_=getInput<double>("timeout").value();
  max_drift_=getInput<double>("max_drift").value();
  if(!(std::isfinite(confidence_)&&confidence_>0&&confidence_<=1&&
       std::isfinite(yaw_rate_)&&yaw_rate_>0&&std::isfinite(timeout_)&&timeout_>0&&
       std::isfinite(max_drift_)&&max_drift_>0))
    throw BT::RuntimeError("BalloonObserve invalid params");
  if(!fcu_odom_ptr||fcu_odom_ptr->header.stamp.isZero())
    throw BT::RuntimeError("BalloonObserve missing odom");
  hover_=fcu_odom_ptr->pose.pose.position;
  starting_yaw_=PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
  if(!std::isfinite(starting_yaw_))throw BT::RuntimeError("BalloonObserve invalid yaw");
  start_=ros::Time::now(); last_hit_=ros::Time(0);hits_=0;down_on_=false;
  auto nh=mct->getROSHandle();
  sub_=nh->subscribe(mode_=="front"?"/objects":"/objects_down",10,&BalloonObserve::objectsCallback,this);
  {std::lock_guard<std::mutex> lock(mutex_);rx_seq_=consumed_seq_=0;objects_.objects.clear();}
  if(mode_=="down") {
    down_control_pub_=nh->advertise<common_msgs::DetEnable>("/detect_ctrl_down",1,true);
    common_msgs::DetEnable ctrl;ctrl.header.stamp=ros::Time::now();ctrl.is_open=true;
    down_control_pub_.publish(ctrl);down_on_=true;
  }
  status("BEGIN");
  return NodeStatus::RUNNING;
}
NodeStatus BalloonObserve::onRunning() {
  const ros::Time now=ros::Time::now();
  double elapsed=(now-start_).toSec();
  if(mode_=="front") {
    if(!fcu_odom_ptr||fcu_odom_ptr->header.stamp.isZero()||
       (now-fcu_odom_ptr->header.stamp).toSec()>1.0||
       !std::isfinite(fcu_odom_ptr->pose.pose.position.x)||
       !std::isfinite(fcu_odom_ptr->pose.pose.position.y)||
       !std::isfinite(fcu_odom_ptr->pose.pose.position.z)||
       gap(fcu_odom_ptr->pose.pose.position,hover_)>max_drift_) {
      sendHoverCommand();
      throw BT::RuntimeError("BalloonObserve front scan unsafe odom/drift");
    }
    // 固定XYZ、从当前朝向缓慢扫描完整360度，无平移/冲刺。
    double angle=std::min(2*PI,yaw_rate_*std::max(0.0,elapsed));
    publishPosition(hover_,std::remainder(starting_yaw_+angle,2*PI));
  } else {
    sendHoverCommand();  // 下视只观察，不参与伺服与移动。
  }
  common_msgs::Objects frame;bool fresh=false;
  {std::lock_guard<std::mutex> lock(mutex_);
   if(rx_seq_!=consumed_seq_){frame=objects_;consumed_seq_=rx_seq_;fresh=true;}}
  if(fresh) {
    bool seen=false;
    const bool right_source=(mode_=="front")?
      (frame.source_id==0||frame.source_id==1):(frame.source_id==2);
    if(right_source)for(const auto&o:frame.objects) {
      if(o.class_name==target_ && o.score>=confidence_){seen=true;break;}
    }
    if(seen){++hits_;last_hit_=now;}else hits_=0;
    if(hits_>=2) {
      sendHoverCommand();status("FOUND");stopDown();return NodeStatus::SUCCESS;
    }
  }
  if(!last_hit_.isZero()&&(now-last_hit_).toSec()>0.8)hits_=0;
  if(mode_=="front") {
    if(elapsed>=timeout_||elapsed>=2*PI/yaw_rate_+0.5) {
      sendHoverCommand();status("NOT_FOUND_RAISE");return NodeStatus::FAILURE;
    }
  } else if(elapsed>=timeout_) {
    sendHoverCommand();status("NOT_FOUND_CONTINUE_LEGACY");
    stopDown();return NodeStatus::SUCCESS;
  }
  ROS_INFO_THROTTLE(2.0,"[%s] BalloonObserve %s %s hits=%d elapsed=%.1f",
    name().c_str(),mode_.c_str(),target_.c_str(),hits_,elapsed);
  return NodeStatus::RUNNING;
}
void BalloonObserve::onHalted() {stopDown();DroneActionNode::onHalted();}
BT_REGISTER_NODES(factory) {factory.registerNodeType<BalloonObserve>("BalloonObserve");}
