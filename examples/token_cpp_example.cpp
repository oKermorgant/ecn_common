#include <ecn_common/token_handle.h>
#include <sstream>
#include <time.h>

using namespace std;

int main(int argc, char** argv)
{
    int kill = 0;
    if(argc == 5)
    {
        kill = atoi(argv[2]);
        std::cout << "Sleeping for " << atoi(argv[1]) << " s\n";
        std::cout << "Will end after " << kill << " s" << std::endl;
        sleep(atoi(argv[1]));
    }

    // this node name
    stringstream ss;
    srand (time(NULL));
    ss << rand() % 255;
    const string node_name = "node_" + ss.str();

    ros::init(argc, argv, node_name.c_str());

    ecn::TokenHandle token;

    ros::Rate loop(1);
    double t0 = ros::Time::now().toSec();

    while(ros::ok())
    {
        if(kill && ros::Time::now().toSec() - t0 > kill)
            break;

        cout << node_name << " doing its C++ job" << endl;

        token.update();

        loop.sleep();
        ros::spinOnce();
    }








}
