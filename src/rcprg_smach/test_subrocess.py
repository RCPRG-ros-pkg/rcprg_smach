import subprocess
import os 
import shlex
import psutil
print("Calling Actinf execution")
FNULL = open(os.devnull, 'w')
DEVNULL = open(os.devnull, 'wb')
print("Calling Actinf execution")
subp = subprocess.Popen(shlex.split("roslaunch tiago_rosplan_sim launch_actinf.launch"), stdout=DEVNULL)
p = psutil.Process(subp.pid)
print("Waiting for Actinf to setup")
# subp = subprocess.call("roslaunch tiago_rosplan_sim launch_actinf.launch", shell=True)
# p = psutil.Process(subp.pid)
print("Waiting for Actinf to setup")