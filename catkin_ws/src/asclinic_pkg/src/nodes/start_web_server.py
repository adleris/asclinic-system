#!/usr/bin/env python3
import rospy
import subprocess

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "web_server"
    rospy.init_node(node_name)

    web_server_path = "/home/mattheww/ASC/asclinic-system/webpages"

    cmd_str = "cd " + web_server_path + "; python3 -m http.server"
    subprocess.run(cmd_str, shell=True)
    # Spin as a single-threaded node
    rospy.spin()

