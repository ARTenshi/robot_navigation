Put common information about arena, common actions of the robot, etc.,
in python scripts at src/common/
so that scripts in other packages can use them by 
from common import *
or something like it.

Every time the IP address of tk1 changes, do
ssh -oHostKeyAlgorithms='ssh-rsa' ubuntu@hsrb-tk1.local
from hsrb.local to keep roslaunch available.

