#!/usr/bin/env python
import rospy

def cli_recommendations(req):
    global outer
    global upper
    global lower
    global shoes
    outer = req.outer
    lower = req.lower
    upper = req.upper
    shoes = req.shoes
    ready = True
    stop_spinning = True
    return

def recom_server():
    stop_spinning = False 
    rospy.init_node('recommendationstwo_service')
    s = rospy.Service('recommendationstwo', CliRecommendations, cli_recommendations)
    print "Ready for recommendations"
    while not stop_spinning:
        rospy.rostime.wallsleep(0.5)


def recom_client():
    global outer
    global upper
    global lower
    global shoes
    rospy.wait_for_service('recommendationstwo')
    try:
        recom_srv = rospy.ServiceProxy('recommendationstwo', RecommendationsTwo)
        req = recom_srv()
        outer = req.outer
        lower = req.lower
        upper = req.upper
        shoes = req.shoes

        ready = True
        return
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def reselect(outer, upper, lower, shoes):

    replaceall = ((outer) and (upper) and (lower) and (shoes))
    if replaceall == True:
        replaceall = 1
    else:
        replaceall = 0
    rospy.wait_for_service('waitreplacetwo')
    try:
        recom_srv = rospy.ServiceProxy('waitreplacetwo', WaitReplaceTwo)
        resp1 = recom_srv(replaceall, upper, lower, outer, shoes)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.wait_for_service('waitreplace')
    try:
        recom_srv = rospy.ServiceProxy('waitreplace', WaitReplace)
        resp1 = recom_srv(replaceall, upper, lower, outer, shoes)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def presetoutfit(activity):

    temp1 = urllib.urlopen(url).read()
    temp = temp1.split("\"temp\":")
    temp = temp[1].split(",\"pressure\"")

    if float(temp[0]) < 5:
        temperature = 5
    elif float(temp[0]) < 13:
        temperature = 4
    elif float(temp[0]) < 21:
        temperature = 3
    elif float(temp[0]) < 26:
        temperature = 2
    else:
        temperature = 1

    temp = temp1.split("\"main\":\"")
    temp = temp[1].split("\",\"description\"")

    if "rain" in temp:
        rain = 3
    else:
        rain = 1

    if "snow" in temp:
        snow = 3
    else:
        snow = 1


    rospy.wait_for_service('featuretype')
    try:
        recom_srv = rospy.ServiceProxy('featuretype', FeatureType)
        recom_srv(0, activity)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    time.sleep(1)

    rospy.wait_for_service('features')
    try:
        recom_srv = rospy.ServiceProxy('features', Features)
        recom_srv(indoor, temperature, active, formal, comfort, snow, rain)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def customoutfit():
    print("customoutfit")
    rospy.wait_for_service('featuretype')
    try:
        recom_srv = rospy.ServiceProxy('featuretype', FeatureType)
        recom_srv(1, "")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def wait():
    rospy.wait_for_service('waitnext')
    try:
        recom_srv = rospy.ServiceProxy('waitnext', WaitNext)
        recom_srv()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendprofile(profile_name):
    rospy.wait_for_service('username')
    try:
        recom_srv = rospy.ServiceProxy('username', UserName)
        recom_srv(profile_name)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def sendfeatures():

    temp1 = urllib.urlopen(url).read()
    temp = temp1.split("\"temp\":")
    temp = temp[1].split(",\"pressure\"")

    if float(temp[0]) < 5:
        temperature = 5
    elif float(temp[0]) < 13:
        temperature = 4
    elif float(temp[0]) < 21:
        temperature = 3
    elif float(temp[0]) < 26:
        temperature = 2
    else:
        temperature = 1

    temp = temp1.split("\"main\":\"")
    temp = temp[1].split("\",\"description\"")

    if "rain" in temp:
        rain = 3
    else:
        rain = 1

    if "snow" in temp:
        snow = 3
    else:
        snow = 1


    print("wait")
    rospy.wait_for_service('features')
    print("wait2")
    try:
        recom_srv = rospy.ServiceProxy('features', Features)
        recom_srv(indoor, temperature, active, formal, comfort, snow, rain)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print("wait3")

def accept(accept):
    rospy.wait_for_service('waitacceptancetwo')
    try:
        recom_srv = rospy.ServiceProxy('waitacceptancetwo', WaitAcceptanceTwo)
        recom_srv(accept)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    rospy.wait_for_service('waitacceptance')
    try:
        recom_srv = rospy.ServiceProxy('waitacceptance', WaitAcceptance)
        recom_srv(accept)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]