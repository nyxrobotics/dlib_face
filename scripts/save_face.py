#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from dlib_face.srv import SaveFace, SaveFaceRequest


if __name__ == '__main__':
    rospy.init_node('save_face')
    name = raw_input("Please input your name: ")
    raw_input('Hello {}. Please press Enter to capture your face.: '.format(name))
    while True:
        rospy.wait_for_service("save_face")
        save_face_service = rospy.ServiceProxy("save_face", SaveFace)
        response = save_face_service(SaveFaceRequest(name=name))
        if response.success:
            rospy.loginfo('Saved face.')
        else:
            rospy.logwarn(response.message)
        if raw_input('Would you like to take another one? [y/n]: ') not in ['', 'y', 'Y']:
            break
