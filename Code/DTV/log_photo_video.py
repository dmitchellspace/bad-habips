#!/usr/bin/env python

"""
file: log_photo_video.py
author: Chris Schwab
project: High Altitude Balloon Instrumentation Platform
description: Script to capture set amount of video and then a set amount of photos FOREVER
"""

###########################
# Imports
###########################
# sudo apt-get install python-picamera
import picamera
import sys
import time
import csv
import subprocess
import os
import RPi.GPIO as GPIO
from fractions import Fraction
import socket

# Custom logger class
import logger

###########################
# QUICK SETUP
###########################
photo_burst_amount 		= 4			# number of photos to take
video_capture_time 		= 60 			# length of video to capture

printing_enabled 		= 0			# enable printing of log data to terminal
logging_enabled 		= 1			# enable csv logging

###########################
# GPIO Pins
###########################
# GPIO pin numbers
dbg_led1 	= 26

# set GPIO pin numbering to match the RasPi board header
GPIO.setmode(GPIO.BCM)

# set GPIOs as outputs
GPIO.setwarnings(False)
GPIO.setup(dbg_led1, GPIO.OUT)

###########################
# Log File Naming
###########################
# NOTE: Use absolute paths since this script will be run from /etc/rc.local on boot!

# script_log
# 	check for already used log names (aka if the RasPi had been previously booted or crashed --> XXXXX_photo_video_logger.log)
log_file_base = "photo_video_logger"
log_base_path = "/home/pi/habip/photo_video_sw/logs/"

# 	start log index at 0
log_file_index = 0
log_files_found = False
# check for all log_file_base files and increment the index to the next unique number
for file in os.listdir(log_base_path):
	if (file[6:].startswith(log_file_base) and file.endswith('.log')):
		log_files_found = True
		number_in_use = int(file[0:5])
		if (number_in_use > log_file_index):
			log_file_index = number_in_use
if (log_files_found):
	print "Found log file index up to: {}".format(log_file_index)
	log_file_index = log_file_index + 1
# pad index to 5 places
log_file_index = "{:05d}".format(log_file_index)
print "Log file index set to: {}".format(log_file_index)

###########################
# photo_video logger object
###########################
photo_video_logger = logger.logger("photo_video_logger", logFileName="{}{}_{}.log".format(log_base_path, log_file_index, log_file_base))

###########################
# CSV Camera Settings
###########################
# 	use the unique log_file_index from the script_log search
csv_logfile_name 	= "{}_camera_settings_logged.csv".format(log_file_index)
csv_logfile_base_path = "/home/pi/habip/photo_video_sw/logs/"

csv_logfile_header 	= ["Sample Index", "Elapsed Time (s)", "Epoch Time (s)", "Date Time Stamp", "File Name", "analog_gain", "digital_gain", "exposure_speed", "shutter_speed", "hflip", "vflip", "rotation", "resolution", "framerate", "meter_mode"]

# 	create log file
if (logging_enabled):
	# camera settings
	with open(csv_logfile_base_path + csv_logfile_name, 'w+') as f:
		writer = csv.writer(f)
		writer.writerow(csv_logfile_header)

###########################
# Photo/Video Dirs
###########################
# NOTE: Use absolute paths since this script will be run from /etc/rc.local on boot!
photo_save_file_base = "photo"
photo_save_base_path = "/home/pi/habip/photo_video_sw/photos/"
video_save_file_base = "video"
video_save_base_path = "/home/pi/habip/photo_video_sw/videos/"

# photo_naming / video_naming
# 	check for already used video/photo names (aka if the RasPi had been previously booted or crashed --> videoXXXXX_DATE_TIME.h264, photoXXXXX_DATE_TIME.jpg)
# start photo index at 0
photo_file_index = 0
photo_files_found = False
# check for all photo_save_file_base files and increment the index to the next unique number
for file in os.listdir(photo_save_base_path):
	if (file.startswith(photo_save_file_base) and file.endswith('.jpeg')):
		photo_files_found = True
		number_in_use = int(file[5:10])
		if (number_in_use > photo_file_index):
			photo_file_index = number_in_use
if (photo_files_found):
	print "Found photo file index up to: {}".format(photo_file_index)
	photo_video_logger.log.warning("Found photo file index up to: {}".format(photo_file_index))
	photo_file_index = photo_file_index + 1
# pad index to 5 places
photo_file_index_padded = "{:05d}".format(photo_file_index)
print "Photo file index set to: {}".format(photo_file_index_padded)
photo_video_logger.log.warning("Photo file index set to: {}".format(photo_file_index_padded))

# start video index at 0
video_file_index = 0
video_files_found = False
# check for all video_save_file_base files and increment the index to the next unique number
for file in os.listdir(video_save_base_path):
	if (file.startswith(video_save_file_base) and file.endswith('.h264')):
		video_files_found = True
		number_in_use = int(file[5:10])
		if (number_in_use > video_file_index):
			video_file_index = number_in_use
if (video_files_found):
	print "Found video file index up to: {}".format(video_file_index)
	photo_video_logger.log.warning("Found video file index up to: {}".format(video_file_index))
	video_file_index = video_file_index + 1
# pad index to 5 places
video_file_index_padded = "{:05d}".format(video_file_index)
print "Video file index set to: {}".format(video_file_index_padded)
photo_video_logger.log.warning("Video file index set to: {}".format(video_file_index_padded))

###########################
# "Main"
###########################

# #	Resolution	AspRat	Framerates	Video	Image	FoV			Binning
# 1	1920x1080	16:9	0.1-30fps	x	 			Partial		None
# 2	3280x2464	4:3		0.1-15fps	x		x		Full		None
# 3	3280x2464	4:3		0.1-15fps	x		x		Full		None
# 4	1640x1232	4:3		0.1-40fps	x	 			Full		2x2
# 5	1640x922	16:9	0.1-40fps	x	 			Full		2x2
# 6	1280x720	16:9	40-90fps	x	 			Partial		2x2
# 7	640x480		4:3		40-90fps	x	 			Partial		2x2

camera = picamera.PiCamera()

# configure camera settings
#camera.resolution 				= (1280, 720) 				# default
#camera.framerate 				= 30 						# default
#camera.sharpness 				= 0 						# defualt
#camera.contrast 				= 0 						# defualt
#camera.brightness 				= 50 						# defualt
#camera.drc_strength 			= 'off' 					# default
#camera.saturation 				= 0 						# defualt
#camera.iso 					= 0 						# defualt (0 = auto mode)
#camera.exposure_compensation	= 0 						# defualt
#camera.exposure_mode 			= 'auto' 					# default
#camera.exposure_speed 			= 0 						# defualt
#camera.flash_mode 				= 'off' 					# default
#camera.awb_mode 				= 'auto' 					# default
#camera.image_effect 			= 'none' 					# default
#camera.color_effects 			= None 						# default
#camera.zoom 					= (0.0, 0.0, 1.0, 1.0) 		# default

# Disable camera LED to save power
camera.led 						= False 					# disable camera LED

# Set metering mode for multi-spot (spot mode is bad since the camera center will not always be on the target object aka earth)
camera.meter_mode 				= 'matrix' 					# default = 'average'

# Eanbling this reduces FOV
camera.video_stabilization 		= False 					# defualt = False

# NEED TO CHANGE based on camera orientation
camera.rotation 				= 0 						# default
camera.hflip 					= False 					# default
camera.vflip 					= False 					# default

# loop counter
loop_index = 0

# GPIO output values
dbg_led1_value 	= 0x0

# starting relative time stamp
t_start_rel = time.time()

# Setting resolution and framerate of the video stream
camera.resolution = (1280, 720)
camera.framerate = 15

# main logging loop
while(1):
#############################################################################################
# This sections was meant to have the pi 0 reconnect to the pi 3 if the pi 3 fails during
# launch. This is not working. The pi 0's don't seem to disconnect from the pi 3 like they
# should which makes it so they don't truly try to reconnect to the pi 3.
# Side note I am a MECE and am not quite sure what is going on.
#############################################################################################
#	# This section is to set up the picamera to stream to the ground
#	# This will connect to the DTV PI 3
#	# Use Port 8000 for B0
#	# Use Port 8100 for B1
#	# Use Port 8200 for B2
#	# Use Port 8230 for B3
#	try:
#		client_socket = socket.socket()
#		client_socket.connect(('192.168.1.15', 8200))  # This is where you change the port #
#		print("Connection achieved.")
#		connection = client_socket.makefile('wb')
#		camera.start_recording(connection, format='h264', splitter_port=2, bitrate=3500000)
#		# Annotating W2RIT-11 with a black background to specify that we are using METEOR's callsign and a HAB
#		camera.annotate_text = "W2RIT-11"
#		camera.annotate_text_size = 40
#	except socket.error:
#		print("No connection found.")
#############################################################################################


	# configure for photo capture
#	camera.resolution 	= (3280, 2464)	# highest resolution --> uses entire sensor
	# capture set amount of photos
	for x in range (0, photo_burst_amount):
		# set save file name: photoXXXX_DATE_TIME.jpeg
		photo_file_index_padded = "{:05d}".format(photo_file_index)
		photo_file_name = photo_save_base_path + photo_save_file_base + photo_file_index_padded + "_" + time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime()) + ".jpeg"
		
		if (printing_enabled):
			print "capturing photo..."
		photo_video_logger.log.warning("Capturing photo: {}".format(photo_file_name))
		# log camera settings
		if (logging_enabled):
			# epoch time
			t_photo_epoch = time.time()
			# relative time stamp
			t_photo_rel = t_photo_epoch - t_start_rel
			# date time stamp
			date_time_stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
			# loop index and time data
			sample_time_data = [loop_index, t_photo_rel, t_photo_epoch, date_time_stamp]
			# read camera config data
			all_data = [photo_file_name, float(camera.analog_gain), float(camera.digital_gain), float(camera.exposure_speed), float(camera.shutter_speed), str(camera.hflip), str(camera.vflip), str(camera.rotation), str(camera.resolution), str(camera.framerate), str(camera.meter_mode)]
			# log all data
			with open(csv_logfile_base_path + csv_logfile_name, 'a+') as f:
				writer = csv.writer(f)
				writer.writerow(sample_time_data + all_data)
		# capture photo
		camera.capture(photo_file_name, format='jpeg')
		
		# increment phot index number
		photo_file_index = photo_file_index + 1
		
		# toggle the DGB1 LED to show script is still running
		dbg_led1_value = dbg_led1_value ^ 0x1
		GPIO.output(dbg_led1, dbg_led1_value)
		
		# wait 0.1 seconds
		time.sleep(0.1)

	# configure for video
#	camera.resolution 	= (1920, 1080)
	#camera.resolution 	= (1280, 720)
#	camera.framerate 	= 30 			# 30fps is smooth enough
	# record video
	# set save file name: videoXXXX_DATE_TIME.h264
	video_file_index_padded = "{:05d}".format(video_file_index)
	video_file_name = video_save_base_path + video_save_file_base + video_file_index_padded + "_" + time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime()) + ".h264"
	
	if (printing_enabled):
		print "capturing video..."
	photo_video_logger.log.warning("Capturing video: {}".format(video_file_name))
	# log camera settings
	if (logging_enabled):
		# epoch time
		t_video_epoch = time.time()
		# relative time stamp
		t_video_rel = t_video_epoch - t_start_rel
		# date time stamp
		date_time_stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
		# loop index and time data
		sample_time_data = [loop_index, t_video_rel, t_video_epoch, date_time_stamp]
		# read camera config data
		all_data = [video_file_name, float(camera.analog_gain), float(camera.digital_gain), float(camera.exposure_speed), float(camera.shutter_speed), str(camera.hflip), str(camera.vflip), str(camera.rotation), str(camera.resolution), str(camera.framerate), str(camera.meter_mode)]
		# log all data
		with open(csv_logfile_base_path + csv_logfile_name, 'a+') as f:
			writer = csv.writer(f)
			writer.writerow(sample_time_data + all_data)
	# record
	camera.start_recording(video_file_name,splitter_port=1)
	camera.wait_recording(video_capture_time)
	camera.stop_recording(splitter_port=1)
	# increment video index number
	video_file_index = video_file_index + 1

	# increment loop counter
	loop_index = loop_index + 1
	if (printing_enabled):
		print "loop index: {}".format(loop_index)
	photo_video_logger.log.warning("loop index: {}".format(loop_index))
	if (printing_enabled):
		print "elapsed time (s): {}\n".format(time.time() - t_start_rel)
	photo_video_logger.log.warning("elapsed time (s): {}".format(time.time() - t_start_rel))

camera.close()
sys.exit(1)

