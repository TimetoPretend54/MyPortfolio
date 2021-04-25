#!/usr/bin/env python
#
#
# Raspberry Pi to Alexa 
#
# SwitchDoc Labs
#

PI2ALEXAVERSION = "002"

#imports 

import sys
import os
import time
import json

from time import time
from datetime import datetime
from datetime import timedelta

from pubnub.pubnub import PubNub
from pubnub.pubnub import PNConfiguration
from pubnub.callbacks import SubscribeCallback


# debug setting
DEBUG = False

# PubNub configuration

Pubnub_Publish_Key = "pub-c-a2209755-b405-4167-88d6-c34039971ef6"
Pubnub_Subscribe_Key = "sub-c-2d3ae220-03c6-11e8-a55d-d67d19117359"

#GPIO LED Pin configuration

# pubnub configuration from https://www.pubnub.com/docs/python/pubnub-python-sdk
pnconf = PNConfiguration()
 
pnconf.subscribe_key = Pubnub_Subscribe_Key
pnconf.publish_key = Pubnub_Publish_Key

pubnub = PubNub(pnconf)

def publish_callback(result, status):
        if (DEBUG):
			print "status.is_error", status.is_error()
			print "status.original_response", status.original_response
			pass
		# handle publish result, status always present, result if successful
		# status.isError to see if error happened

def publishStatusToPubNub(coffeeCmd):
	global coffeeCommand
	myMessage = {}
						# I might just check if Baxter is "enabled" or something from rosrun baxter_tools enable_robot.py -s, for "Busy" Status, and for "Off" Status, make user close ROS Node/Python Program, and send message first before exiting
	print "What is it here? ",coffeeCmd
	if coffeeCmd == False:
		myMessage["Baxter_CurrentStatus"] = "Active" 
		myMessage["Baxter_CoffeeCommand"] = "False"
	else:
		myMessage["Baxter_CurrentStatus"] = "Busy" 
		myMessage["Baxter_CoffeeCommand"] = "True"
	myMessage["TimeStamp"] = '{}'.format( datetime.now().strftime( "%m/%d/%Y %H:%M:%S"))

	if (DEBUG):
		print myMessage

	pubnub.publish().channel('Baxter_Status').message(myMessage).async(publish_callback)



# As a listner, instead of checking PubNub periodically, anytime the Alexa publisher
# publishes something, (like make me coffee), the "message" funtion will run, use a boolean
# to make sure Baxter can only run make coffee once though
class AlexaMyListener(SubscribeCallback):
	def status(self, pubnub, status):
		pass 		# happens if radio / connectivity is lost
	def message(self, pubnub, message):		# handle new message in message.message
		print "message=", message.message
		
		Baxter_Coffee_Command = message.message["Coffee_Request"]
		print "Baxter_Coffee_Command = ", Baxter_Coffee_Command
		if Baxter_Coffee_Command == "Make Coffee":
			coffeeCommand = True
		
		print "Publish tp Pubnub that coffee command is ", coffeeCommand
		publishStatusToPubNub(coffeeCommand)
		
		# Assigns JSON message from subscribe to variable (we'll change this too later)
		#LED_State = message.message["LED"]
			#print "LED_State = ", LED_State

		# where we will determine if told to make coffee or not (example here for now)
		# After making coffee, will set coffeeCommand = false
		#if (LED_State == "Off"):
			#GPIO.output(LED_PIN, GPIO.LOW)

		#if (LED_State == "On"):
			#GPIO.output(LED_PIN, GPIO.HIGH)


	def presence(self, pubnub, presence):
		pass		# handle incoming presence data

# set up subscribe channel

my_listener = AlexaMyListener()
 
pubnub.add_listener(my_listener)
 
pubnub.subscribe().channels("Alexa_Data").execute()

prev = time()
prev2 = time()
i = 0
publishStatusToPubNub(False) #originally send false

while True:
	now = time()
	
	# countdown timer
	if now - prev2 > 1:
		i=i+1
		print "\r",str(i)+"...",
		sys.stdout.flush()
		prev2 = time()
	
	# Wait for x amount of seconds before sending info
	if now - prev > 30:					# 10 minutes is 600 seconds
		if(i != 30):
			print "\r",str(10)+"...",
		coffeeCommand = False
		publishStatusToPubNub(coffeeCommand)
		#time.sleep(10.0)	
		prev = time()
		i = 0
	else:
		pass
