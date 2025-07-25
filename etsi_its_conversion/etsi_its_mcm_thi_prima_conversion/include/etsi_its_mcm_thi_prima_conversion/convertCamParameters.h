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
CamParameters ::= SEQUENCE {
	basicContainer BasicContainer,
	highFrequencyContainer HighFrequencyContainer,
	lowFrequencyContainer LowFrequencyContainer OPTIONAL,
	specialVehicleContainer SpecialVehicleContainer OPTIONAL,
	...
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_mcm_thi_prima_coding/asn_CamParameters.h>
#include <etsi_its_mcm_thi_prima_conversion/convertBasicContainer.h>
#include <etsi_its_mcm_thi_prima_conversion/convertHighFrequencyContainer.h>
#include <etsi_its_mcm_thi_prima_conversion/convertLowFrequencyContainer.h>
#include <etsi_its_mcm_thi_prima_conversion/convertSpecialVehicleContainer.h>
#ifdef ROS1
#include <etsi_its_mcm_thi_prima_msgs/CamParameters.h>
namespace asn_msgs = etsi_its_mcm_thi_prima_msgs::msg;
#else
#include <etsi_its_mcm_thi_prima_msgs/msg/cam_parameters.hpp>
namespace asn_msgs = etsi_its_mcm_thi_prima_msgs::msg;
#endif


namespace etsi_its_mcm_thi_prima_conversion {

void toRos_CamParameters(const asn_CamParameters_t& in, asn_msgs::CamParameters& out) {
  toRos_BasicContainer(in.basicContainer, out.basic_container);
  toRos_HighFrequencyContainer(in.highFrequencyContainer, out.high_frequency_container);
  if (in.lowFrequencyContainer) {
    toRos_LowFrequencyContainer(*in.lowFrequencyContainer, out.low_frequency_container);
    out.low_frequency_container_is_present = true;
  }
  if (in.specialVehicleContainer) {
    toRos_SpecialVehicleContainer(*in.specialVehicleContainer, out.special_vehicle_container);
    out.special_vehicle_container_is_present = true;
  }
}

void toStruct_CamParameters(const asn_msgs::CamParameters& in, asn_CamParameters_t& out) {
  memset(&out, 0, sizeof(asn_CamParameters_t));
  toStruct_BasicContainer(in.basic_container, out.basicContainer);
  toStruct_HighFrequencyContainer(in.high_frequency_container, out.highFrequencyContainer);
  if (in.low_frequency_container_is_present) {
    out.lowFrequencyContainer = (asn_LowFrequencyContainer_t*) calloc(1, sizeof(asn_LowFrequencyContainer_t));
    toStruct_LowFrequencyContainer(in.low_frequency_container, *out.lowFrequencyContainer);
  }
  if (in.special_vehicle_container_is_present) {
    out.specialVehicleContainer = (asn_SpecialVehicleContainer_t*) calloc(1, sizeof(asn_SpecialVehicleContainer_t));
    toStruct_SpecialVehicleContainer(in.special_vehicle_container, *out.specialVehicleContainer);
  }
}

}
