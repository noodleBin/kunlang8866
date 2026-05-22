/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef VEHICLE_MESSAGE_H
#define VEHICLE_MESSAGE_H

#include <iostream>
#include <vector>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <memory>
#include "encryption_strategy.h"

namespace century {
namespace mcloud {

class VehicleMessage {
public:
    const std::string START_FLAG = "##";
    
    uint8_t command_id;
    uint8_t response_flag;
    std::string unique_id;
    uint8_t encryption_method;
    uint16_t data_length;
    std::vector<uint8_t> data_unit;
    uint8_t checksum;

    VehicleMessage() {}
    // Constructor
    VehicleMessage(uint8_t command_id, uint8_t response_flag, const std::string& unique_id,
                   uint8_t encryption_method, const std::vector<uint8_t>& data_unit);

    // Pack the message into binary data
    std::vector<uint8_t> pack_message() const;

    // Unpack binary data into a VehicleMessage object
    static VehicleMessage unpack_message(std::vector<uint8_t>& binary_data);

private:
    // Calculate checksum (BCC)
    uint8_t calculate_checksum() const;

    // Get encryption strategy based on encryption method
    std::shared_ptr<EncryptionStrategy> get_encryption_strategy(uint8_t encryption_method) const;

    // Encrypt data unit
    std::vector<uint8_t> encrypt_data(const std::vector<uint8_t>& data) const;

    // Decrypt data unit
    std::vector<uint8_t> decrypt_data(const std::vector<uint8_t>& data) const;
};

} // namespace mcloud
} // namespace century

#endif // VEHICLE_MESSAGE_H
