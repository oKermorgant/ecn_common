#ifndef ECN_TOKEN_H
#define ECN_TOKEN_H

#include <ros/ros.h>
#include <ecn_common/TokenCurrent.h>
#include <ecn_common/TokenRequest.h>

namespace ecn
{

typedef enum
{
  BAXTER_LEFT,
  BAXTER_RIGHT,
  BAXTER_BOTH
} Side;

class TokenHandle
{
public:
  TokenHandle(const Side& side = BAXTER_BOTH, const std::string& group = "")
  {
    if(group == "")
      init(side, ros::this_node::getName());
    else
      init(side, group);
  }
  TokenHandle(const std::string &group, Side side = BAXTER_BOTH)
    : TokenHandle(side, group) {}

  void update()
  {
    pub_.publish(req_);
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ecn_common::TokenRequest req_;
  std::string current_;
  double t_;

  void callback(const ecn_common::TokenCurrentConstPtr & msg)
  {
    t_ = ros::Time::now().toSec();
    if(req_.arm < 2)
      current_ = msg->left;
    else if(req_.arm % 2 == 0 || msg->right != req_.group)
      current_  = msg->right;
  }

  void init(Side side, std::string group)
  {
    // init message & publisher
    req_.group = group;
    if(side == BAXTER_BOTH)
      req_.arm = 0;
    else if(side == BAXTER_LEFT)
      req_.arm = 1;
    else
      req_.arm = 2;
    pub_ = nh_.advertise<ecn_common::TokenRequest>("/token_manager/request", 1);

    // init subscriber
    sub_ = nh_.subscribe("/token_manager/current", 1, &TokenHandle::callback, this);
    current_ = "";
    double t0;
    t0 = ros::Time::now().toSec();

    // wait for token manager
    ros::Rate loop(1);
    while(ros::ok() && current_ != req_.group)
    {
      t_ = ros::Time::now().toSec();
      update();
      if(current_ != req_.group && current_ != "")
      {
        std::string side = "both arms";
        if(req_.arm == 1)
          side = "left arm";
        else if(req_.arm == 2)
          side = "right arm";

        ROS_INFO("Group %s (%s): current token is for group %s", req_.group.c_str(), side.c_str(), current_.c_str());
      }
      if(current_ == "" && t_ - t0 > 5)
        break;

      loop.sleep();
      ros::spinOnce();
    }

    // now we have the hand - no need to continue subcribing
    if(ros::ok())
      sub_.shutdown();
  }
};


}

#endif // ECN_TOKEN_H
