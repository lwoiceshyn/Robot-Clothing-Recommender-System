#!/usr/bin/env python
'''
Debugger script which allows for manually running ROS clients and services to test functionality
'''
from prototype.srv import *
import sys
import rospy


def say_something_client(lol):
    rospy.wait_for_service('say_something')
    print "Attempting to send speech"
    try:
        say_something = rospy.ServiceProxy('say_something', SaySomething)
        say_something(lol)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e

def username_client(name):
    rospy.wait_for_service('username')
    print "Attempting to send username"
    try:
        username = rospy.ServiceProxy('username', UserName)
        username(name)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e

def recommendations_client():
    rospy.wait_for_service('recommendations')
    print "Requesting Recommendation from the Clothing Recommender Module"
    try:
        recommendations = rospy.ServiceProxy('recommendations', Recommendations)
        resp = recommendations()
        print resp.upper, resp.lower, resp.outer, resp.shoes

    except rospy.ServiceException, e:
        print"Service call failed: %s"%e   

def recommendationstwo_client():
    rospy.wait_for_service('recommendationstwo')
    print "Requesting Recommendation from the Clothing Recommender Module"
    try:
        recommendations = rospy.ServiceProxy('recommendationstwo', RecommendationsTwo)
        resp = recommendations()
        print resp.upper, resp.lower, resp.outer, resp.shoes

    except rospy.ServiceException, e:
        print"Service call failed: %s"%e   


def features_client(inside_outside, outside_warmth, athletic, casual_formal, comfort, snow, rain):
    rospy.wait_for_service('features')
    print "Attempting to send features to node"
    try:
        features = rospy.ServiceProxy('features', Features)
        features(inside_outside, outside_warmth, athletic, casual_formal, comfort, snow, rain)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e   

def featuretype_client(preset_or_choose, activity):
    rospy.wait_for_service('featuretype')
    print "Attempting to send features to node"
    try:
        featuretype = rospy.ServiceProxy('featuretype', FeatureType)
        featuretype(preset_or_choose, activity)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e   

def waitnext_client():
    rospy.wait_for_service('waitnext')
    print "Letting program know user hit next button"
    try:
        waitnext = rospy.ServiceProxy('waitnext', WaitNext)
        number = waitnext()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e   

def waitacceptance_client(acceptance):
    rospy.wait_for_service('waitacceptance')
    print "Letting program know user accepted/reject outfit"
    try:
        waitacceptance = rospy.ServiceProxy('waitacceptance', WaitAcceptance)
        waitacceptance(acceptance)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e  

def waitacceptancetwo_client(acceptance):
    rospy.wait_for_service('waitacceptancetwo')
    print "Letting program know user accepted/reject outfit"
    try:
        waitacceptancetwo = rospy.ServiceProxy('waitacceptancetwo', WaitAcceptanceTwo)
        waitacceptancetwo(acceptance)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e  

def waitreplace_client(all, upper, lower, outer, shoes):
    rospy.wait_for_service('waitreplace')
    print "Letting program know if user wants to replace all or part of outfit"
    try:
        waitreplace = rospy.ServiceProxy('waitreplace', WaitReplace)
        waitreplace(all, upper, lower, outer, shoes)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e  

def waitreplacetwo_client(all, upper, lower, outer, shoes):
    rospy.wait_for_service('waitreplacetwo')
    print "Letting program know if user wants to replace all or part of outfit"
    try:
        waitreplacetwo = rospy.ServiceProxy('waitreplacetwo', WaitReplaceTwo)
        waitreplacetwo(all, upper, lower, outer, shoes)
    except rospy.ServiceException, e:
        print"Service call failed: %s"%e  
        
        
if __name__ == "__main__":
    speech = "Wedding"
    while True:
        client = raw_input("Please enter which client to run\n")
    
        if client == "username":
            username_client("Leo")
        elif client == "featuretype":
            featuretype_client(0, speech)
        elif client == "features":
            features_client(1, 2, 0, 1, 4, 1, 1)
        elif client == "recommendations":
            recommendations_client()
        elif client == "recommendationstwo":
            recommendationstwo_client()
        elif client == "waitnext":
            waitnext_client()
        elif client == "waitacceptancetwo":
            waitacceptancetwo_client(1)
        elif client == "waitacceptance":
            waitacceptance_client(1)
        elif client == "waitreplacetwo":    
            waitreplacetwo_client(1, 1, 0, 1, 0)
        elif client == "waitreplace":    
            waitreplace_client(1, 1, 0, 1, 0)
        elif client == "exit":
            break
        else:
            print "Invalid command."

       