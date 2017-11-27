#! /usr/bin/env python
from ecn_common.token_handle import TokenHandle
from pylab import rand
import rospy

ss = str(int(1000*rand()))

rospy.init_node('node_' + ss)

group_name = 'group_' + ss
token = TokenHandle(group_name)

while not rospy.is_shutdown():
    print(group_name + " doing its Python job")
    
    token.update();
    rospy.sleep(1)
