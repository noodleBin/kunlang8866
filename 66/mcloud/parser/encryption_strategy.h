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

#ifndef ENCRYPTION_STRATEGY_H
#define ENCRYPTION_STRATEGY_H

#include <vector>
#include <cstdint>

// Encryption Strategy Interface
class EncryptionStrategy {
public:
    virtual ~EncryptionStrategy() = default;
    virtual std::vector<uint8_t> encrypt(const std::vector<uint8_t>& data) const = 0;
    virtual std::vector<uint8_t> decrypt(const std::vector<uint8_t>& data) const = 0;
};

// Encryption Strategy Implementations
class NoEncryption : public EncryptionStrategy {
public:
    std::vector<uint8_t> encrypt(const std::vector<uint8_t>& data) const override;
    std::vector<uint8_t> decrypt(const std::vector<uint8_t>& data) const override;
};

class RsaEncryption : public EncryptionStrategy {
public:
    std::vector<uint8_t> encrypt(const std::vector<uint8_t>& data) const override;
    std::vector<uint8_t> decrypt(const std::vector<uint8_t>& data) const override;
};

class AesEncryption : public EncryptionStrategy {
public:
    std::vector<uint8_t> encrypt(const std::vector<uint8_t>& data) const override;
    std::vector<uint8_t> decrypt(const std::vector<uint8_t>& data) const override;
};

#endif // ENCRYPTION_STRATEGY_H
