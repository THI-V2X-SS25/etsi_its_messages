#!/usr/bin/env python

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import rclpy
from rclpy.node import Node
from etsi_its_mcm_thi_prima_msgs.msg import *
import utils


class Publisher(Node):

    def __init__(self):

        super().__init__("mcm_thi_prima_publisher")
        topic = "/etsi_its_conversion/mcm_thi_prima/in"
        self.publisher = self.create_publisher(MCM, topic, 1)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):

        msg = MCM()

        msg.header.protocol_version = 3
        msg.header.message_id = msg.header.MESSAGE_ID_MCM
        msg.header.station_id.value = 1994

        msg.mcm.generation_delta_time.value = int(utils.get_t_its(self.get_clock().now().nanoseconds) % 65536)

        msg.mcm.mcm_parameters.basic_container_mcm.station_type.value = msg.mcm.mcm_parameters.basic_container_mcm.station_type.PASSENGER_CAR
        msg.mcm.mcm_parameters.basic_container_mcm.reference_position.latitude.value = int(1e7 * 48.765922)
        msg.mcm.mcm_parameters.basic_container_mcm.reference_position.longitude.value = int(1e7 * 11.434697)

        
        intention_sharing_container = IntentionSharingContainer()                
        intention_sharing_container.speed.speed_value.value = intention_sharing_container.speed.speed_value.ONE_CENTIMETER_PER_SEC
        intention_sharing_container.speed.speed_confidence.value = intention_sharing_container.speed.speed_confidence.EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC
        intention_sharing_container.heading.heading_value.value = 0
        intention_sharing_container.heading.heading_confidence.value = intention_sharing_container.heading.heading_confidence.EQUAL_OR_WITHIN_ONE_DEGREE
        intention_sharing_container.vehicle_length.vehicle_length_value.value = intention_sharing_container.vehicle_length.vehicle_length_value.TEN_CENTIMETERS * 42
        intention_sharing_container.vehicle_width.value = intention_sharing_container.vehicle_width.TEN_CENTIMETERS * 18
        intention_sharing_container.vehicle_automation_level.value = intention_sharing_container.vehicle_automation_level.SAE_LEVEL3
        intention_sharing_container.lane_position_is_present = False
        
        #add at least 1 trajactory
        veh_traj = TrajectoryPointMCM()
        veh_traj.delta_longitudinal_position.value = 0
        veh_traj.delta_lateral_position.value = 0
        veh_traj.delta_heading.value = 0
        veh_traj.delta_time.value = 1
        
        intention_sharing_container.planned_trajectory.array.append(veh_traj)
        
        msg.mcm.mcm_parameters.intention_sharing_container = intention_sharing_container
        self.get_logger().info(f"Publishing MCM (PRIMA)")
        self.publisher.publish(msg)


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
