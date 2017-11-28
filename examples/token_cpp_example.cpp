#include <ecn_common/token_handle.h>
#include <sstream>
#include <time.h>

using namespace std;

int main(int argc, char** argv)
{
    // this node name
    stringstream ss;
    srand (time(NULL));
    ss << rand() % 255;
    string node_name = "node_" + ss.str();
    string group_name = ss.str();

    ros::init(argc, argv, node_name.c_str());

    ecn::TokenHandle token(group_name);

    ros::Rate loop(1);

    while(ros::ok())
    {
        cout << group_name << " doing its C++ job" << endl;

        token.update();

        loop.sleep();
        ros::spinOnce();
    }








}
