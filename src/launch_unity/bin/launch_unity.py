#!/usr/bin/env python
import rospy, rospkg
import subprocess
import os

def main():
    rospy.init_node('executable_launcher_node')

    map_name = rospy.get_param('~map_name', 'factory')
    map_exe = map_name + '.x86_64'

    rospack = rospkg.RosPack()
    path = rospack.get_path('launch_unity')
    executable_path = os.path.join(path,'../vehicle_simulator/mesh',map_name,'environment/',map_exe)

    # # Launch the executable
    process = subprocess.Popen(executable_path)
    # rospy.loginfo("Executable launched")

    # # Wait for the process to complete
    process.wait()
    # rospy.loginfo("Executable finished with exit code: {}".format(process.returncode))

if __name__ == '__main__':
    main()