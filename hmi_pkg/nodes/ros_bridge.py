#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS bridge module - Optional ROS 2 integration
"""

import threading
import queue
import logging
from typing import Optional
from config import Config

log = logging.getLogger("hmi.ros")

# Try to import ROS 2
ROS_ENABLED = Config.ROS_ENABLED
_ros_import_error = ""

if ROS_ENABLED:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String as RosString
    except Exception as e:
        _ros_import_error = f"rclpy import failed: {e}"
        ROS_ENABLED = False

class ROSPublisherThread(threading.Thread):
    """ROS publisher thread for ASR and intent messages"""
    
    def __init__(self):
        super().__init__(daemon=True)
        self.q: queue.Queue[str] = queue.Queue()
        self.ready = False
        self.init_error = ""

    def run(self):
        try:
            rclpy.init(args=None)
            node = Node("hmi_bridge")
            pub_asr = node.create_publisher(RosString, Config.ROS_ASR_TOPIC, 10)
            pub_intent = node.create_publisher(RosString, Config.ROS_INTENT_TOPIC, 10)
            self.ready = True
            
            def timer_cb():
                import queue as _q
                try:
                    while True:
                        kind, payload = self.q.get_nowait().split(":",1)
                        msg = RosString()
                        msg.data = payload
                        if kind == "asr":
                            pub_asr.publish(msg)
                        elif kind == "intent":
                            pub_intent.publish(msg)
                except _q.Empty:
                    pass
            
            node.create_timer(0.05, timer_cb)
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            self.init_error = f"ROS thread init failed: {e}"
            self.ready = False

    def publish_asr(self, text: str):
        if text: 
            self.q.put(f"asr:{text}")

    def publish_intent(self, intent_json: str):
        if intent_json: 
            self.q.put(f"intent:{intent_json}")

# Global ROS thread instance
_ros_thread: Optional[ROSPublisherThread] = None

def initialize_ros():
    """Initialize ROS bridge if enabled"""
    global _ros_thread
    
    if ROS_ENABLED:
        log.info("ROS bridge enabled (ASR→%s, intent→%s) confirm_mode=%s", 
                 Config.ROS_ASR_TOPIC, Config.ROS_INTENT_TOPIC, Config.ROS_CONFIRM_MODE)
        _ros_thread = ROSPublisherThread()
        _ros_thread.start()
    else:
        if _ros_import_error:
            log.warning("ROS bridge disabled: %s", _ros_import_error)
        else:
            log.info("ROS bridge disabled")

def get_ros_thread() -> Optional[ROSPublisherThread]:
    """Get ROS thread instance"""
    return _ros_thread
