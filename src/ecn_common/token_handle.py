#! /usr/bin/env python

import rospy
from ecn_common.msg import TokenCurrent, TokenRequest

class TokenHandle:
    def __init__(self, side = '', group = ''):
        if group == '':
            self.init(side, rospy.get_name())
        else:
            self.init(side, group)        
        
    def init(self, side, group):
        
        # init request and publisher
        self.req = TokenRequest()
        self.req.group = group
        if side == '':
            self.req.arm = 0
            side = 'both arms'
        elif side == 'left':
            self.req.arm = 1
            side = 'left arm'
        else:
            self.req.arm = 2
            side = 'right arm'
        self.pub = rospy.Publisher('/token_manager/request', TokenRequest, queue_size=1)
        
        # init subscriber and wait
        self.sub = rospy.Subscriber('token_manager/current', TokenCurrent, self.currentCB)
        self.current = ''
        self.t = rospy.Time.now().to_sec()
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown() and self.current != self.req.group:
            self.t = rospy.Time.now().to_sec()
            self.update()
            
            if self.current != self.req.group and self.current != '':
                print("Group {} ({}): current token is for group {}".format(self.req.group, side, self.current))
            if self.current == '' and self.t - t0 > 5:
                break

            rospy.sleep(1)

        # now we have the hand - no need to continue subcribing
        if not rospy.is_shutdown():
            self.sub.unregister()
        
    def currentCB(self, msg):
        self.t = rospy.Time.now().to_sec()
        if self.req.arm in (0,1):
            self.current = msg.left
        elif self.req.arm in (0,2) or msg.right != self.req.group:
            self.current = msg.right
                                
            
    def update(self):
        self.pub.publish(self.req)
