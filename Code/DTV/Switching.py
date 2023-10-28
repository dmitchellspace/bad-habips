import socket
import subprocess
import time

# Setting the duration each video feed will be shown
view_time = 20

# Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
# all interfaces)
B0_socket = socket.socket()
B1_socket = socket.socket()
B2_socket = socket.socket()
B3_socket = socket.socket()

B0_socket.bind(('0.0.0.0', 8000))
B1_socket.bind(('0.0.0.0', 8100))
B2_socket.bind(('0.0.0.0', 8200))
B3_socket.bind(('0.0.0.0', 8230))

# This section can be used to time out the connection in order to keep the code
# from getting stuck at this point. Due to time constraints this will be commented out.
#B0_socket.settimeout(5)
#B1_socket.settimeout(5)
#B2_socket.settimeout(5)
#B3_socket.settimeout(5)

B0_socket.listen(0)
B1_socket.listen(0)
B2_socket.listen(0)
B3_socket.listen(0)

# Accept a single connection and make a file-like object out of it
try:
	connection0 = B0_socket.accept()[0].makefile('rb')
	print("Connection to B0 made")
except:
	print("B0 connection timeout")

try:
	connection1 = B1_socket.accept()[0].makefile('rb')
	print("Connection to B1 made")
except:
	print("B1 connection timeout")

try:
	connection2 = B2_socket.accept()[0].makefile('rb')
	print("Connection to B2 made")
except:
	print("B2 connection timeout")

try:
	connection3 = B3_socket.accept()[0].makefile('rb')
	print("Connection to B3 made")
except:
	print("B3 connection timeout")

try:
    # Run a viewer with an appropriate command line.
    cmdline = ['ffmpeg', '-re', '-an', '-i' ,'-', '-vcodec', 'copy', '-f', 'mpegts', '-mpegts_flags', 'system_b', '-']
    out = open('video.fifo', 'wb')
    player = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=out)
    loop_start = time.time()
    while True:
        # Repeatedly read 1k of data from the connection and write it to
        # the media player's stdin
	video0 = connection0.read(1024)
	video1 = connection1.read(1024)
	video2 = connection2.read(1024)
	video3 = connection3.read(1024)
	if not video0 or not video1 or not video2 or not video3:
		break
	if time.time()-loop_start<=view_time:
		player.stdin.write(video0)
        elif time.time()-loop_start<=view_time*2:
		player.stdin.write(video1)
	elif time.time()-loop_start<=view_time*3:
		player.stdin.write(video2)
	elif time.time()-loop_start<=view_time*4:
		player.stdin.write(video3)
	else:
		loop_start=time.time()
finally:
	try:
		connection0.close()
	except NameError:
		print("No B0")
	try:		
		connection1.close()
	except NameError:
		print("No B1")
	try:
		connection2.close()
	except NameError:
		print("No B2")
	try:
		connection3.close()
	except NameError:
		print("No B3")
	B0_socket.close()
	B1_socket.close()
	B2_socket.close()
	B3_socket.close()
	try:
		player.terminate()
	except NameError:
		print("No player")
