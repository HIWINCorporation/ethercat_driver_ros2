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

#include <iomanip>

#include "ethercat_generic_mdp/ec_mdp_module.hpp"

namespace ethercat_generic_plugins
{

EcMDPModule::EcMDPModule() : EcMDPDevice()
{
}

EcMDPModule::~EcMDPModule()
{
}

bool EcMDPModule::initialized()
{
  return initialized_;
}

bool EcMDPModule::setupSlave(std::unordered_map<std::string, std::string> slave_paramters,
                             std::vector<double>* state_interface, std::vector<double>* command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("axis") != paramters_.end())
  {
    axis_num_ = std::stod(paramters_["axis"]) - 1;
  }
  else
  {
    std::cerr << "EcMDPModule: failed to find 'axis' tag in URDF." << std::endl;
    return false;
  }

  if (paramters_.find("slave_config") != paramters_.end())
  {
    if (!setup_from_config_file(paramters_["slave_config"]))
    {
      return false;
    }
  }
  else
  {
    std::cerr << "EcMDPModule: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  if (paramters_.find("mode_of_operation") != paramters_.end())
  {
    mode_of_operation_display_ = mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  std::cout << "Slot: " << axis_num_ << " RxPDO" << std::endl;
  for (auto& iter : rpdo_channels_info_)
  {
    uint32_t index = iter.first;
    ethercat_interface::EcPdoChannelManager* channel = &iter.second;
    std::cout << "        0x" << std::hex << channel->index;
    if (channel->pdo_type == ethercat_interface::RPDO)
    {
      if (paramters_.find("command_interface/" + channel->interface_name) != paramters_.end())
      {
        channel->interface_index = std::stoi(paramters_["command_interface/" + channel->interface_name]);
        std::cout << " "
                  << "interface index: " << channel->interface_index << " "
                  << "command_interface/" + channel->interface_name;
      }
    }
    std::cout << std::endl;
    channel->setup_interface_ptrs(nullptr, command_interface_ptr_);
  }

  std::cout << "Slot: " << axis_num_ << " TxPDO" << std::endl;
  for (auto& iter : tpdo_channels_info_)
  {
    uint32_t index = iter.first;
    ethercat_interface::EcPdoChannelManager* channel = &iter.second;
    std::cout << "        0x" << std::hex << channel->index;
    if (channel->pdo_type == ethercat_interface::TPDO)
    {
      if (paramters_.find("state_interface/" + channel->interface_name) != paramters_.end())
      {
        channel->interface_index = std::stoi(paramters_["state_interface/" + channel->interface_name]);
        std::cout << " "
                  << "interface index: " << channel->interface_index << " "
                  << "state_interface/" + channel->interface_name;
      }
    }
    std::cout << std::endl;
    channel->setup_interface_ptrs(state_interface_ptr_, nullptr);
  }

  return true;
}

void EcMDPModule::rxProcessData(size_t index, uint8_t* domain_address)
{
  if (!rpdo_channels_info_.count(index))
  {
    return;
  }

  if (rpdo_channels_info_[index].index == CiA402D_RPDO_CONTROLWORD)
  {
    if (is_operational_)
    {
      if (auto_state_transitions_)
      {
        rpdo_channels_info_[index].default_value =
            transition(state_, rpdo_channels_info_[index].ec_read(domain_address));
      }
    }
  }

  if (rpdo_channels_info_[index].index == CiA402D_RPDO_POSITION)
  {
    if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE)
    {
      rpdo_channels_info_[index].default_value =
          rpdo_channels_info_[index].factor * last_position_ + rpdo_channels_info_[index].offset;
    }
    rpdo_channels_info_[index].override_command =
        (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;
  }

  if (rpdo_channels_info_[index].index == CiA402D_RPDO_MODE_OF_OPERATION)
  {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10)
    {
      rpdo_channels_info_[index].default_value = mode_of_operation_;
    }
  }

  rpdo_channels_info_[index].ec_update(domain_address);
}

void EcMDPModule::txProcessData(size_t index, uint8_t* domain_address)
{
  if (!tpdo_channels_info_.count(index))
  {
    return;
  }

  tpdo_channels_info_[index].ec_update(domain_address);

  if (tpdo_channels_info_[index].index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY)
  {
    mode_of_operation_display_ = tpdo_channels_info_[index].last_value;
  }

  if (tpdo_channels_info_[index].index == CiA402D_TPDO_POSITION)
  {
    last_position_ = tpdo_channels_info_[index].last_value;
  }

  if (tpdo_channels_info_[index].index == CiA402D_TPDO_STATUSWORD)
  {
    status_word_ = tpdo_channels_info_[index].last_value;
  }

  if (status_word_ != last_status_word_)
  {
    state_ = EcCiA402Drive::deviceState(status_word_);
    if (state_ != last_state_)
    {
      std::cout << "STATE: " << DEVICE_STATE_STR.at(state_) << " with status word : 0x" << std::setfill('0') << std::hex
                << std::setw(4) << status_word_ << std::endl;
    }
  }

  initialized_ = ((state_ == STATE_OPERATION_ENABLED) && (last_state_ == STATE_OPERATION_ENABLED)) ? true : false;

  last_status_word_ = status_word_;
  last_state_ = state_;
}

bool EcMDPModule::setup_from_config(YAML::Node slave_config)
{
  if (slave_config.size() != 0)
  {
    for (auto i = 0ul; i < slave_config["rpdo"].size(); i++)
    {
      auto rpdo_channels_size = slave_config["rpdo"][i]["channels"].size();
      for (auto c = 0ul; c < rpdo_channels_size; c++)
      {
        ethercat_interface::EcPdoChannelManager channel_info;
        channel_info.pdo_type = ethercat_interface::RPDO;
        channel_info.load_from_config(slave_config["rpdo"][i]["channels"][c]);
        if (i == axis_num_)
        {
          channel_info.index = channel_info.index - (0x0800 * axis_num_);
          rpdo_channels_info_[rpdo_channels_.size()] = channel_info;
        }
        rpdo_channels_.push_back(channel_info.get_pdo_entry_info());
      }
    }

    for (auto i = 0ul; i < slave_config["tpdo"].size(); i++)
    {
      auto tpdo_channels_size = slave_config["tpdo"][i]["channels"].size();

      for (auto c = 0ul; c < tpdo_channels_size; c++)
      {
        ethercat_interface::EcPdoChannelManager channel_info;
        channel_info.pdo_type = ethercat_interface::TPDO;
        channel_info.load_from_config(slave_config["tpdo"][i]["channels"][c]);
        if (i == axis_num_)
        {
          channel_info.index = channel_info.index - (0x0800 * axis_num_);
          tpdo_channels_info_[tpdo_channels_.size()] = channel_info;
        }
        tpdo_channels_.push_back(channel_info.get_pdo_entry_info());
      }
    }

    for (auto i = 0ul; i < rpdo_channels_.size(); i++)
    {
      rpdo_domain_map_.push_back(i);
    }

    for (auto i = 0ul; i < tpdo_channels_.size(); i++)
    {
      tpdo_domain_map_.push_back(i);
    }

    if (slave_config["auto_fault_reset"])
    {
      auto_fault_reset_ = slave_config["auto_fault_reset"].as<bool>();
    }
    if (slave_config["auto_state_transitions"])
    {
      auto_state_transitions_ = slave_config["auto_state_transitions"].as<bool>();
    }

    return true;
  }

  std::cerr << "EcMDPModule: failed to load slave configuration: empty configuration" << std::endl;
  return false;
}

bool EcMDPModule::setup_from_config_file(std::string config_file)
{
  try
  {
    slave_config_ = YAML::LoadFile(config_file);
  }
  catch (const YAML::ParserException& ex)
  {
    std::cerr << "EcMDPModule: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  catch (const YAML::BadFile& ex)
  {
    std::cerr << "EcMDPModule: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_))
  {
    return false;
  }
  return true;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcMDPModule, ethercat_interface::EcSlave)
