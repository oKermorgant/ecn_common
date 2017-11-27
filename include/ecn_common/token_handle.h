#ifndef ECN_TOKEN_H
#define ECN_TOKEN_H

#include <ros/ros.h>
#include <ecn_common/Token.h>

namespace ecn
{
class TokenHandle
{
public:

    TokenHandle(std::string group_name, bool _wait = true)
    {
        // init message
        srv_.request.id = group_name;
        srv_.request.init = true;
        srv_.response.available = false;

        client_ = nh_.serviceClient<ecn_common::Token>("/token_manager/manager");

        if(_wait)
            wait();
    }

    void wait()
    {
        // check server actually runs
        if(!client_.waitForExistence(ros::Duration(5)))
        {
            srv_.response.available = true;
            ROS_INFO("Token manager not running, skipping");
        }

        // wait for token
        ros::Rate loop(1);
        while(ros::ok() && !srv_.response.available)
        {
            update();
            if(!srv_.response.available)
            {
                ROS_INFO("Current token is for group %s", srv_.response.current.c_str());
                if(srv_.request.id == srv_.response.current)

                    ROS_INFO("Current token has the same ID as yours");
            }

            loop.sleep();
            ros::spinOnce();
        }

        // now we have the hand
        if(ros::ok())
            srv_.request.init = false;
    }

    void update()
    {
        client_.call(srv_);
    }


protected:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    ecn_common::Token srv_;

};


}

#endif // ECN_TOKEN_H
