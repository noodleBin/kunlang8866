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

#include "vehicle_message.h"

namespace century {
namespace mcloud {

// VehicleMessage Constructor
VehicleMessage::VehicleMessage(uint8_t command_id, uint8_t response_flag, const std::string& unique_id,
                               uint8_t encryption_method, const std::vector<uint8_t>& data_unit)
    : command_id(command_id), response_flag(response_flag), unique_id(unique_id),
      encryption_method(encryption_method), data_unit(data_unit) {

    if (unique_id.length() != 17 && unique_id.length() != 21) {
        throw std::invalid_argument("Unique ID must be 17 (VIN) or 21 (custom ID) characters long.");
    }

    data_length = static_cast<uint16_t>(data_unit.size());

    // Calculate checksum
    checksum = calculate_checksum();
}

// Get encryption strategy based on encryption method
std::shared_ptr<EncryptionStrategy> VehicleMessage::get_encryption_strategy(uint8_t encryption_method) const {
    switch (encryption_method) {
        case 0x01:
            return std::make_shared<NoEncryption>();
        case 0x02:
            return std::make_shared<RsaEncryption>();
        case 0x03:
            return std::make_shared<AesEncryption>();
        default:
            throw std::invalid_argument("Unsupported encryption method.");
    }
}

// Encrypt data unit using the selected encryption strategy
std::vector<uint8_t> VehicleMessage::encrypt_data(const std::vector<uint8_t>& data) const {
    auto strategy = get_encryption_strategy(encryption_method);
    return strategy->encrypt(data);
}

// Decrypt data unit using the selected encryption strategy
std::vector<uint8_t> VehicleMessage::decrypt_data(const std::vector<uint8_t>& data) const {
    auto strategy = get_encryption_strategy(encryption_method);
    return strategy->decrypt(data);
}

// Pack the message into binary data
std::vector<uint8_t> VehicleMessage::pack_message() const {
    std::vector<uint8_t> binary_message;
    // binary_message.push_back(0x23); // ASCII #
    // binary_message.push_back(0x24); // ASCII #

    binary_message.push_back(0x23); // ASCII #
    binary_message.push_back(0x23); // ASCII #

    binary_message.push_back(command_id);
    binary_message.push_back(response_flag);

    for (char c : unique_id) {
        binary_message.push_back(static_cast<uint8_t>(c));
    }

    binary_message.push_back(encryption_method);

    // Add data unit length (big-endian)
    binary_message.push_back((data_length >> 8) & 0xFF);
    binary_message.push_back(data_length & 0xFF);

    // Encrypt data unit
    std::vector<uint8_t> encrypted_data = encrypt_data(data_unit);
    binary_message.insert(binary_message.end(), encrypted_data.begin(), encrypted_data.end());
    uint8_t bcc = 0;
    for (int i = 2; i < binary_message.size(); i++) {
        bcc ^= binary_message[i];
    }
    // Add checksum
    binary_message.push_back(bcc);
    // bcc = 0;
    // for (int i = 2; i < binary_message.size() - 1; ++i) {
    //     bcc ^= binary_message[i];
    // }
    // binary_message.push_back(bcc);
    return binary_message;
}

// Unpack binary data into a VehicleMessage object
VehicleMessage VehicleMessage::unpack_message(std::vector<uint8_t>& binary_data) {
    if (binary_data.size() < 24) {
        throw std::invalid_argument("Invalid binary data, too short.");
    }
    
    // uint8_t bcc = 0;
    // for (int i = 1; i < binary_data.size() - 1; i++) {
    //     bcc ^= binary_data[i];
    // }

    // if (bcc != binary_data[binary_data.size() - 1]) {
    //     throw std::invalid_argument("Invalid bcc.");
    // }

    // binary_data.erase(binary_data.begin());
    // binary_data.erase(binary_data.begin());

    // binary_data.pop_back();
    // binary_data.erase(binary_data.begin());

    if (binary_data[0] != 0x23 || binary_data[1] != 0x23) {
        throw std::invalid_argument("Invalid start flag.");
    }

    uint8_t command_id = binary_data[2];
    uint8_t response_flag = binary_data[3];

    std::string unique_id(binary_data.begin() + 4, binary_data.begin() + 21);

    uint8_t encryption_method = binary_data[21];

    uint16_t data_length = (binary_data[22] << 8) | binary_data[23];

    if (binary_data.size() < 24 + data_length + 1) {
        throw std::invalid_argument("Invalid data length.");
    }

    // Extract encrypted data unit
    std::vector<uint8_t> encrypted_data(binary_data.begin() + 24, binary_data.begin() + 24 + data_length);
    std::vector<uint8_t> data_unit = VehicleMessage(command_id, response_flag, unique_id, encryption_method, std::vector<uint8_t>()).decrypt_data(encrypted_data);

    uint8_t received_checksum = binary_data[24 + data_length];

    VehicleMessage message(command_id, response_flag, unique_id, encryption_method, data_unit);

    if (message.checksum != received_checksum) {
        throw std::invalid_argument("Checksum mismatch.");
    }

    return message;
}

// Calculate checksum (BCC)
uint8_t VehicleMessage::calculate_checksum() const {
    uint8_t bcc = 0;

    bcc ^= command_id;
    bcc ^= response_flag;

    for (char c : unique_id) {
        bcc ^= static_cast<uint8_t>(c);
    }

    bcc ^= encryption_method;
    bcc ^= (data_length >> 8) & 0xFF;
    bcc ^= data_length & 0xFF;

    for (uint8_t byte : data_unit) {
        bcc ^= byte;
    }

    return bcc;
}

} // namespace mcloud
} // namespace century
