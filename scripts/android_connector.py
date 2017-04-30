#!/usr/bin/env python

'''
Script for connecting to and communicating with the touchscreen tablet using bluetooth
'''

import sys
import rospy
import bluetooth
from beginner_tutorials.srv import *
from utils import *
import time
import urllib

#Weather API for weather information
url = 'http://api.openweathermap.org/data/2.5/weather?q={toronto}&APPID=8875cbe7ee239104c9cf97da3c828720&units=metric'

#Default Values if not given
active = 0
formal = 0
indoor = 0
comfort = 5
temperature = 3
snow = 3
rain = 3

profile = ""
upper = "00000001"
lower = "00000001"
outer = "00000001"
shoes = "00000001"

ready = False

stop_spinning = False


if __name__ == "__main__":
    
    #Connect to bluetooth socket
    server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

    server_sock.bind(("",bluetooth.PORT_ANY))
    server_sock.listen(1)
    uuid="00001101-0000-1000-8000-00805F9B34FB"
    bluetooth.advertise_service(server_sock, "helloService",
            service_id = uuid,
                     service_classes=[bluetooth.SERIAL_PORT_CLASS],
                     profiles=[bluetooth.SERIAL_PORT_PROFILE])

    print("running")
    while True:
        client_sock, address = server_sock.accept()
        print "Accepted connection from ",address

        data = client_sock.recv(1024)

        #start recommendation session
        if (data == "REQUEST START"):
            print "received [%s]" % data
            wait()
            client_sock.send("ACK START")


        data = client_sock.recv(1024)
        if (data[:8] == "PROFILE "):
            profile = data[8:]
            sendprofile(profile)
            client_sock.send("ACK PROFILE")
        



        #client requests recommended outfit
        data = client_sock.recv(1024)
        print "received [%s]" % data
        if (data[:15] == "REQUEST PRESET "):
            activity = data[15:]
            client_sock.send("ACK PRESET")

            #client sends preferences
            data = client_sock.recv(1024)
            print "received [%s]" % data
            if (data[:7] == "ACTIVE "):

                if (data[7:13] == "ACTIVE"):
                    active = 1
                else:
                    active = 0
                client_sock.send("ACK ACTIVE")

            data = client_sock.recv(1024)
            print "received [%s]" % data
            if (data[:7] == "FORMAL "):
                if (data[7:13] == "FORMAL"):
                    formal = 2
                elif (data[7:14] == "SFORMAL"):
                    formal = 1
                else:
                    formal = 0
                client_sock.send("ACK FORMAL")
        
            data = client_sock.recv(1024)
            if (data[:7] == "INDOOR "):
                print "received [%s]" % data
                if (data[7:13] == "INDOOR"):
                    indoor = 1
                else:
                    indoor = 0
                client_sock.send("ACK INDOOR")

            data = client_sock.recv(1024)
            if (data[:8] == "COMFORT "):
                print "received [%s]" % data
                if (data[8:] == "1"):
                    comfort = 1
                elif (data[8:] == "2"):
                    comfort = 2
                elif (data[8:] == "3"):
                    comfort = 3
                elif (data[8:] == "4"):
                    comfort = 4
                elif (data[8:] == "5"):
                    comfort = 5
                client_sock.send("ACK COMFORT")

            presetoutfit(activity)
            client_sock.send("ACK CONTINUE")
        elif (data == "REQUEST CUSTOM"):
            print "received [%s]" % data
            customoutfit()
            client_sock.send("ACK CONTINUE")   

            #client sends preferences
            print ("waiting")
            data = client_sock.recv(1024)
            print "received [%s]" % data
            if (data[:7] == "ACTIVE "):

                if (data[7:13] == "ACTIVE"):
                    active = 1
                else:
                    active = 0

                wait()
                client_sock.send("ACK CONTINUE")

            data = client_sock.recv(1024)
            if (data[:7] == "INDOOR "):
                print "received [%s]" % data
                if (data[7:13] == "INDOOR"):
                    indoor = 1
                else:
                    indoor = 0
                wait()
                client_sock.send("ACK CONTINUE")

            data = client_sock.recv(1024)
            if (data[:8] == "COMFORT "):
                print "received [%s]" % data
                if (data[8:] == "1"):
                    comfort = 1
                elif (data[8:] == "2"):
                    comfort = 2
                elif (data[8:] == "3"):
                    comfort = 3
                elif (data[8:] == "4"):
                    comfort = 4
                elif (data[8:] == "5"):
                    comfort = 5
                wait()
                client_sock.send("ACK CONTINUE")

            data = client_sock.recv(1024)
            print "received [%s]" % data
            if (data[:7] == "FORMAL "):
                if (data[7:13] == "FORMAL"):
                    formal = 2
                elif (data[7:14] == "SFORMAL"):
                    formal = 1
                else:
                    formal = 0
                wait()
                sendfeatures()
                wait()
                client_sock.send("ACK CONTINUE")

        while True:
            #Server sends recommended outfit
            recom_client()
            print(outer)
            print(lower)
            print(upper)
            print(shoes)
            client_sock.send("ROUTERWEAR " + outer)

            data = client_sock.recv(1024)
            if (data == "ACK ROUTERWEAR"):
                print "received [%s]" % data

            client_sock.send("RUPPERWEAR " + upper)

            data = client_sock.recv(1024)
            if (data == "ACK RUPPERWEAR"):
                print "received [%s]" % data

            client_sock.send("RLOWERWEAR " + lower)

            data = client_sock.recv(1024)
            if (data == "ACK RLOWERWEAR"):
                print "received [%s]" % data

            client_sock.send("RSHOES " + shoes)

            data = client_sock.recv(1024)
            if (data == "ACK RSHOES"):
                print "received [%s]" % data

            #Server sends recommended outfit again for satisfaction
            client_sock.send("ROUTERWEAR " + outer)

            data = client_sock.recv(1024)
            if (data == "ACK ROUTERWEAR"):
                wait()
                print "received [%s]" % data

            client_sock.send("RUPPERWEAR " + upper)

            data = client_sock.recv(1024)
            if (data == "ACK RUPPERWEAR"):
                print "received [%s]" % data

            client_sock.send("RLOWERWEAR " + lower)

            data = client_sock.recv(1024)
            if (data == "ACK RLOWERWEAR"):
                print "received [%s]" % data

            client_sock.send("RSHOES " + shoes)

            data = client_sock.recv(1024)
            if (data == "ACK RSHOES"):
                print "received [%s]" % data

            data = client_sock.recv(1024)
            if (data == "SATISFY NO"):
                print "received [%s]" % data
                accept(0)
                client_sock.send("ACK NSATISFY")

            if (data == "SATISFY YES"):
                print "received [%s]" % data
                accept(1)
                client_sock.send("ACK YSATISFY")
                break;

            #Server sends recommended outfit again for reselection
            client_sock.send("ROUTERWEAR " + outer)

            data = client_sock.recv(1024)
            if (data == "ACK ROUTERWEAR"):
                print "received [%s]" % data

            client_sock.send("RUPPERWEAR " + upper)

            data = client_sock.recv(1024)
            if (data == "ACK RUPPERWEAR"):
                print "received [%s]" % data

            client_sock.send("RLOWERWEAR " + lower)

            data = client_sock.recv(1024)
            if (data == "ACK RLOWERWEAR"):
                print "received [%s]" % data

            client_sock.send("RSHOES " + shoes)

            data = client_sock.recv(1024)

            if (data == "ACK RSHOES"):
                print "received [%s]" % data
            res = []
            for i in range(4):
                data = client_sock.recv(1024)
                print(data)
                if data == "RESELECT 00000000":
                    res.append(False)
                else:
                    res.append(True)
                client_sock.send("ACK RESELECT " + data[9:])
                print "ACK RESELECT " + data[9:]

            reselect(res[0], res[1], res[2], res[3])
           
            time.sleep(2)

        client_sock.close()
    server_sock.close()

    client_sock.close()
server_sock.close()
