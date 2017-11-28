#! /usr/bin/env python

import rospy
from ecn_common.msg import TokenCurrent, TokenRequest

class TokenHandle:
    def __init__(self, group, side = ''):
        
        # init request and publisher
        self.req = TokenRequest()
        self.req.group = group
        if side == '':
            self.req.arm = 0
        elif side == 'left':
            self.req.arm = 1
        else:
            self.req.arm = 2
        self.pub = rospy.Publisher('/token_manager/request', TokenRequest, queue_size=1)
        
        
        # init subscriber and wait
        self.sub = rospy.Subscriber('token_manager/current', TokenCurrent, self.currentCB)
        self.current = ""
        self.t = rospy.Time.now().to_sec()
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown() and self.current != self.req.group:
            self.update()
            
            if self.current != self.req.group and self.current != '':
                print("Current token is for group " + self.current)
            if self.current == '' and self.t - t0 > 5:
                break

            rospy.sleep(1)

        # now we have the hand - no need to continue subcribing
        if not rospy.is_shutdown():
            self.sub.unregister()
        
    def currentCB(self, msg):
        self.t = rospy.Time.now().to_sec()
        if self.req.arm == 2:
            self.current = msg.right
        else:
            self.current = msg.left
                                
            
    def update(self):
        self.pub.publish(self.req)
