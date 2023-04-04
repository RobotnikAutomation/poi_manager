#!/usr/bin/env python
import sys
import rospy

if sys.version_info.major == 3.0:
    from poi_manager.poi import PoiManager
else:
    from poi import PoiManager


def main():

    rospy.init_node("poi_manager")

    rc_node = PoiManager()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
