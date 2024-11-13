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

#include "ethercat_generic_mdp/ec_mdp_device.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_generic_plugins
{

EcMDPDevice::EcMDPDevice() : EcCiA402Drive()
{
}

EcMDPDevice::~EcMDPDevice()
{
}

void EcMDPDevice::set_state_is_operational(bool value)
{
  for (auto& iter : slot_)
  {
    uint32_t slot = iter.first;
    slot_[slot]->set_state_is_operational(value);
  }
}

bool EcMDPDevice::setupSlave(std::unordered_map<std::string, std::string> slave_paramters,
                             std::vector<double>* state_interface, std::vector<double>* command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("slave_config") != paramters_.end())
  {
    if (!setup_from_config_file(paramters_["slave_config"]))
    {
      return false;
    }
  }
  else
  {
    std::cerr << "EcMDPDevice: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  setup_syncs();
  return true;
}

void EcMDPDevice::rxProcessData(size_t index, uint8_t* domain_address)
{
  for (auto& iter : slot_)
  {
    uint32_t slot = iter.first;
    slot_[slot]->rxProcessData(index, domain_address);
  }
}

void EcMDPDevice::txProcessData(size_t index, uint8_t* domain_address)
{
  for (auto& iter : slot_)
  {
    uint32_t slot = iter.first;
    slot_[slot]->txProcessData(index, domain_address);
  }
}

void EcMDPDevice::domains(DomainMap& domains) const
{
  domains = { { RXPDO_DOMAIN, rpdo_domain_map_ }, { TXPDO_DOMAIN, tpdo_domain_map_ } };
}

bool EcMDPDevice::setup_from_config(YAML::Node slave_config)
{
  // basic parameter
  if (slave_config.size() != 0)
  {
    if (slave_config["vendor_id"])
    {
      vendor_id_ = slave_config["vendor_id"].as<uint32_t>();
    }
    else
    {
      std::cerr << "EcMDPDevice: failed to load drive vendor ID." << std::endl;
      return false;
    }

    if (slave_config["product_id"])
    {
      product_id_ = slave_config["product_id"].as<uint32_t>();
    }
    else
    {
      std::cerr << "EcMDPDevice: failed to load drive product ID." << std::endl;
      return false;
    }

    if (slave_config["assign_activate"])
    {
      assign_activate_ = slave_config["assign_activate"].as<uint32_t>();
    }

    if (slave_config["sm"])
    {
      for (const auto& sm : slave_config["sm"])
      {
        ethercat_interface::SMConfig config;
        if (config.load_from_config(sm))
        {
          sm_configs_.push_back(config);
        }
      }
    }

    if (slave_config["sdo"])
    {
      for (const auto& sdo : slave_config["sdo"])
      {
        ethercat_interface::SdoConfigEntry config;
        if (config.load_from_config(sdo))
        {
          sdo_config.push_back(config);
        }
      }
    }

    auto rpdo_channels_nbr = 0;
    auto tpdo_channels_nbr = 0;

    if (slave_config["rpdo"])
    {
      for (auto i = 0ul; i < slave_config["rpdo"].size(); i++)
      {
        rpdo_channels_nbr += slave_config["rpdo"][i]["channels"].size();
      }
    }
    if (slave_config["tpdo"])
    {
      for (auto i = 0ul; i < slave_config["tpdo"].size(); i++)
      {
        tpdo_channels_nbr += slave_config["tpdo"][i]["channels"].size();
      }
    }

    all_channels_.reserve(rpdo_channels_nbr + tpdo_channels_nbr);

    rpdo_channels_.reserve(rpdo_channels_nbr);
    rpdo_channels_nbr = 0;
    if (slave_config["rpdo"])
    {
      for (auto i = 0ul; i < slave_config["rpdo"].size(); i++)
      {
        auto rpdo_channels_size = slave_config["rpdo"][i]["channels"].size();
        for (auto c = 0ul; c < rpdo_channels_size; c++)
        {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::RPDO;
          channel_info.load_from_config(slave_config["rpdo"][i]["channels"][c]);
          all_channels_.push_back(channel_info.get_pdo_entry_info());
          rpdo_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        rpdos_.push_back({ slave_config["rpdo"][i]["index"].as<uint16_t>(), rpdo_channels_size,
                           rpdo_channels_.data() + rpdo_channels_nbr });
        rpdo_channels_nbr += rpdo_channels_size;
      }
    }

    tpdo_channels_.reserve(tpdo_channels_nbr);
    tpdo_channels_nbr = 0;
    if (slave_config["tpdo"])
    {
      for (auto i = 0ul; i < slave_config["tpdo"].size(); i++)
      {
        auto tpdo_channels_size = slave_config["tpdo"][i]["channels"].size();

        for (auto c = 0ul; c < tpdo_channels_size; c++)
        {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::TPDO;
          channel_info.load_from_config(slave_config["tpdo"][i]["channels"][c]);
          all_channels_.push_back(channel_info.get_pdo_entry_info());
          tpdo_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        tpdos_.push_back({ slave_config["tpdo"][i]["index"].as<uint16_t>(), tpdo_channels_size,
                           tpdo_channels_.data() + tpdo_channels_nbr });
        tpdo_channels_nbr += tpdo_channels_size;
      }
    }

    // Remove gaps from domain mapping
    for (auto i = 0ul; i < rpdo_channels_.size(); i++)
    {
      if (rpdo_channels_[i].index != 0x0000)
      {
        domain_map_.push_back(i);
        rpdo_domain_map_.push_back(domain_map_.size() - 1);
      }
    }

    for (auto i = 0ul; i < tpdo_channels_.size(); i++)
    {
      if (tpdo_channels_[i].index != 0x0000)
      {
        domain_map_.push_back(i);
        tpdo_domain_map_.push_back(domain_map_.size() - 1);
      }
    }
    return true;
  }

  std::cerr << "EcMDPDevice: failed to load slave configuration: empty configuration" << std::endl;
  return false;
}

bool EcMDPDevice::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try
  {
    slave_config_ = YAML::LoadFile(config_file);
  }
  catch (const YAML::ParserException& ex)
  {
    std::cerr << "EcMDPDevice: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  catch (const YAML::BadFile& ex)
  {
    std::cerr << "EcMDPDevice: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_))
  {
    return false;
  }
  return true;
}

void EcMDPDevice::registerMDPModule(unsigned int index, EcSlave* slave)
{
  slot_.insert({ index, slave });
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcMDPDevice, ethercat_interface::EcSlave)
