#!/usr/bin/env python

# Python
import signal

# ROS
import rospy


class BaseNode:
    """
    A python base node that can be stopped catching SIG_INT
    Usage:

    class TestNode(BaseNode):
        def __init__(self):
            BaseNode.__init__(self, 'TestNode', anonymous=False)
            self.running = True
            while self.running
                # Do something

            # Here handle node shutting down
            rospy.loginfo("Test node is shutdown")
    """
    def __init__(self, node_name, anonymous=False):
        self.running = False
        signal.signal(signal.SIGINT, self.stop_node_handler)
        rospy.init_node(node_name, anonymous=anonymous, disable_signals=True)

    def stop_node_handler(self, *args, **kwargs):
        self.running = False
        rospy.signal_shutdown("Shutdown...")
