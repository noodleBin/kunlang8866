/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/tools/image_decompress/proto/config.pb.h"

namespace century {
namespace image_decompress {

class ImageDecompressComponent final
    : public cyber::Component<century::drivers::CompressedImage> {
 public:
  bool Init() override;
  bool Proc(
      const std::shared_ptr<century::drivers::CompressedImage>& image) override;

 private:
  std::shared_ptr<cyber::Writer<century::drivers::Image>> writer_;
  Config config_;
};

CYBER_REGISTER_COMPONENT(ImageDecompressComponent)

}  // namespace image_decompress
}  // namespace century
