// Copyright 2024 HIWIN Technologies Corp.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HIWIN_DRIVER_EC_MDP_MODULE_HPP_
#define HIWIN_DRIVER_EC_MDP_MODULE_HPP_

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_generic_mdp/ec_mdp_device.hpp"

namespace ethercat_generic_plugins
{

class EcMDPModule : public EcMDPDevice
{
public:
  EcMDPModule();
  virtual ~EcMDPModule();

  virtual bool initialized();

  virtual void set_state_is_operational(bool value)
  {
    is_operational_ = value;
  }

  virtual bool setupSlave(std::unordered_map<std::string, std::string> slave_paramters,
                          std::vector<double>* state_interface, std::vector<double>* command_interface);

  virtual void rxProcessData(size_t index, uint8_t* domain_address);
  virtual void txProcessData(size_t index, uint8_t* domain_address);

protected:
  std::map<uint32_t, ethercat_interface::EcPdoChannelManager> rpdo_channels_info_;
  std::map<uint32_t, ethercat_interface::EcPdoChannelManager> tpdo_channels_info_;

  std::vector<ec_pdo_entry_info_t> rpdo_channels_;
  std::vector<ec_pdo_entry_info_t> tpdo_channels_;
  std::vector<unsigned int> rpdo_domain_map_;
  std::vector<unsigned int> tpdo_domain_map_;

  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node slave_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);

private:
  uint32_t axis_num_;
};
}  // namespace ethercat_generic_plugins

#endif  // HIWIN_DRIVER_EC_MDP_MODULE_HPP_
