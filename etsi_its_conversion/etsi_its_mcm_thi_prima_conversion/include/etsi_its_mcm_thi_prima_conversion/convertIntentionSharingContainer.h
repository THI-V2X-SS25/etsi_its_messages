/** ============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

/** Auto-generated by https://github.com/ika-rwth-aachen/etsi_its_messages -----
python3 \
  asn1ToConversionHeader.py \
  -t \
  asn \
  /home/lobo/Development/etsi_its_message/ros2_ws/src/etsi_its_messages/asn1/raw/mcm_thi_prima/cdd/ITS-Container.asn \
  /home/lobo/Development/etsi_its_message/ros2_ws/src/etsi_its_messages/asn1/raw/cam_en302637_2/CAM-PDU-Descriptions.asn \
  /home/lobo/Development/etsi_its_message/ros2_ws/src/etsi_its_messages/asn1/raw/mcm_thi_prima/MCM-PDU-Descriptions.asn \
  -o \
  /home/lobo/mcm_conversion/
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
IntentionSharingContainer ::= SEQUENCE {
    plannedTrajectory TrajectoryMCM,
    heading Heading,
    speed Speed,
    driveDirection DriveDirection,
    vehicleLength VehicleLength,
    vehicleWidth VehicleWidth,
    vehicleAutomationLevel VehicleAutomationLevel,
    lanePosition LanePosition OPTIONAL
    -- roadID LaneID OPTIONAL,
    -- laneID LaneID OPTIONAL,
    -- laneIDEnd LaneID OPTIONAL
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_mcm_thi_prima_coding/asn_IntentionSharingContainer.h>
#include <etsi_its_mcm_thi_prima_conversion/convertDriveDirection.h>
#include <etsi_its_mcm_thi_prima_conversion/convertHeading.h>
#include <etsi_its_mcm_thi_prima_conversion/convertLanePosition.h>
#include <etsi_its_mcm_thi_prima_conversion/convertSpeed.h>
#include <etsi_its_mcm_thi_prima_conversion/convertTrajectoryMCM.h>
#include <etsi_its_mcm_thi_prima_conversion/convertVehicleAutomationLevel.h>
#include <etsi_its_mcm_thi_prima_conversion/convertVehicleLength.h>
#include <etsi_its_mcm_thi_prima_conversion/convertVehicleWidth.h>
#ifdef ROS1
#include <etsi_its_mcm_thi_prima_msgs/IntentionSharingContainer.h>
namespace asn_msgs = etsi_its_mcm_thi_prima_msgs::msg;
#else
#include <etsi_its_mcm_thi_prima_msgs/msg/intention_sharing_container.hpp>
namespace asn_msgs = etsi_its_mcm_thi_prima_msgs::msg;
#endif


namespace etsi_its_mcm_thi_prima_conversion {

void toRos_IntentionSharingContainer(const asn_IntentionSharingContainer_t& in, asn_msgs::IntentionSharingContainer& out) {
  toRos_TrajectoryMCM(in.plannedTrajectory, out.planned_trajectory);
  toRos_Heading(in.heading, out.heading);
  toRos_Speed(in.speed, out.speed);
  toRos_DriveDirection(in.driveDirection, out.drive_direction);
  toRos_VehicleLength(in.vehicleLength, out.vehicle_length);
  toRos_VehicleWidth(in.vehicleWidth, out.vehicle_width);
  toRos_VehicleAutomationLevel(in.vehicleAutomationLevel, out.vehicle_automation_level);
  if (in.lanePosition) {
    toRos_LanePosition(*in.lanePosition, out.lane_position);
    out.lane_position_is_present = true;
  }
}

void toStruct_IntentionSharingContainer(const asn_msgs::IntentionSharingContainer& in, asn_IntentionSharingContainer_t& out) {
  memset(&out, 0, sizeof(asn_IntentionSharingContainer_t));
  toStruct_TrajectoryMCM(in.planned_trajectory, out.plannedTrajectory);
  toStruct_Heading(in.heading, out.heading);
  toStruct_Speed(in.speed, out.speed);
  toStruct_DriveDirection(in.drive_direction, out.driveDirection);
  toStruct_VehicleLength(in.vehicle_length, out.vehicleLength);
  toStruct_VehicleWidth(in.vehicle_width, out.vehicleWidth);
  toStruct_VehicleAutomationLevel(in.vehicle_automation_level, out.vehicleAutomationLevel);
  if (in.lane_position_is_present) {
    out.lanePosition = (asn_LanePosition_t*) calloc(1, sizeof(asn_LanePosition_t));
    toStruct_LanePosition(in.lane_position, *out.lanePosition);
  }
}

}
