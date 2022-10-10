#!/usr/bin/env python
# coding: utf-8
"""NatNet client ROS node.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/natnet_ros/blob/master/LICENSE.
No part of natnet_ros, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file."""


import rospy

from natnet_ros import NatnetClientNode


if __name__ == '__main__':
    rospy.init_node('aimotion_mocap_node', disable_signals=True, anonymous=True)
    try:
        node = NatnetClientNode(rospy.get_param("car_id", None), rospy.get_param("/AIMotionLab/mocap_server"))
        node.run()
    finally:
        rospy.signal_shutdown(reason='Finished')
