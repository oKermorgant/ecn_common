#! /usr/bin/env python

import rospy
from ecn_common.srv import Token, TokenResponse, TokenRequest

class TokenHandle:
    def __init__(self, name, wait = True):
        self.request = TokenRequest()
        self.request.id = name
        self.request.init = True
        self.response = TokenResponse()
        self.response.available = False
        
        self.client = rospy.ServiceProxy('/token_manager/manager', Token)
        
        if wait:
            self.wait()            
            
    def wait(self):
        try:
            self.client.wait_for_service(5)                
        except:
            print("Token manager not running, skipping")
            self.response.available = True
            
        while not rospy.is_shutdown() and not self.response.available:
            self.update()
            if not self.response.available:
                print("Current token is for group " + self.response.current)
                if self.response.current == self.request.id:
                    print("Current token has the same ID as yours")
                    
            rospy.sleep(1)
            
        if not rospy.is_shutdown():
            self.request.init = False
            
            
    def update(self):
        self.response = self.client.call(self.request)                
