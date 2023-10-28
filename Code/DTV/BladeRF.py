# This will convert the BladeRF command into a script

from subprocess import call

command =  "load fpga withdvb.rbf; set frequency tx 1240000000; set bandwidth tx 3500000; set samplerate tx 5200000;tx config file=video.fifo; tx config repeat=1; set txvga1 -4; set txvga2 17; tx start; tx wait;"

while(1):
	call(['bladeRF-cli', '-e', command])
	break

