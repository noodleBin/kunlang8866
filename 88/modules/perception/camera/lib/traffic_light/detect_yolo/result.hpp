#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/macro.hpp"

namespace century {
namespace perception {
namespace camera {

struct KLDEPLOY Image {
    const void* data;
    int width = 0;
    int height = 0;

    Image(const void* data, int width, int height) : data(data), width(width), height(height) {
        if (width <= 0 || height <= 0) {
            throw std::invalid_argument(MAKE_ERROR_MESSAGE("Image: width and height must be positive"));
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const Image& img) {
        os << "Image(width=" << img.width << ", height=" << img.height << ", data=" << img.data << ")";
        return os;
    }
};
struct KLDEPLOY Mask {
    std::vector<uint8_t> data;
    int width = 0;
    int height = 0;

    Mask() = default;

    Mask(int width, int height) : width(width), height(height) {
        if (width < 0 || height < 0) {
            throw std::invalid_argument(MAKE_ERROR_MESSAGE("Mask: width and height must be positive"));
        }
        data.resize(width * height);
    }

    friend std::ostream& operator<<(std::ostream& os, const Mask& mask) {
        os << "Mask(width=" << mask.width << ", height=" << mask.height << ", data size=" << mask.data.size() << ")";
        return os;
    }

    Mask(const Mask& other) = default;
    Mask& operator=(const Mask& other) = default;

    Mask(Mask&& other) noexcept = default;
    Mask& operator=(Mask&& other) noexcept = default;
};

struct KLDEPLOY KeyPoint {
    float x;
    float y;

    float conf = -1.0f;

    KeyPoint() = default;

    KeyPoint(float x_ = 0, float y_ = 0, float conf_ = -1.0f)
      : x(x_), y(y_), conf(conf_) {}
    friend std::ostream& operator<<(std::ostream& os, const KeyPoint& kp) {
        os << "KeyPoint(x=" << kp.x << ", y=" << kp.y;
        if (0.0f <= kp.conf) {
            os << ", conf=" << kp.conf;
        }
        os << ")";
        return os;
    }

    KeyPoint(const KeyPoint& other) = default;
    KeyPoint& operator=(const KeyPoint& other) = default;

    KeyPoint(KeyPoint&& other) noexcept = default;
    KeyPoint& operator=(KeyPoint&& other) noexcept = default;
};

struct KLDEPLOY Box {
    float left;
    float top;
    float right;
    float bottom;

    Box() = default;

    Box(float left, float top, float right, float bottom)
        : left(left), top(top), right(right), bottom(bottom) {}

    friend std::ostream& operator<<(std::ostream& os, const Box& box) {
        os << "Box(left=" << box.left << ", top=" << box.top << ", right=" << box.right << ", bottom=" << box.bottom << ")";
        return os;
    }

    Box(const Box& other) = default;
    Box& operator=(const Box& other) = default;

    Box(Box&& other) noexcept = default;
    Box& operator=(Box&& other) noexcept = default;
};

struct KLDEPLOY RotatedBox : public Box {
    float theta;

    RotatedBox() = default;

    RotatedBox(float left, float top, float right, float bottom, float theta)
        : Box(left, top, right, bottom), theta(theta) {}

    friend std::ostream& operator<<(std::ostream& os, const RotatedBox& rbox) {
        os << "RotatedBox(left=" << rbox.left << ", top=" << rbox.top << ", right=" << rbox.right << ", bottom=" << rbox.bottom << ", theta=" << rbox.theta << ")";
        return os;
    }

    RotatedBox(const RotatedBox& other) = default;
    RotatedBox& operator=(const RotatedBox& other) = default;

    RotatedBox(RotatedBox&& other) noexcept = default;
    RotatedBox& operator=(RotatedBox&& other) noexcept = default;
};

struct KLDEPLOY BaseRes {
    int num = 0;
    std::vector<int> classes;
    std::vector<float> scores;

    BaseRes() = default;

    BaseRes(int num, const std::vector<int>& classes, const std::vector<float>& scores)
        : num(num), classes(classes), scores(scores) {}

    BaseRes(const BaseRes& other) = default;
    BaseRes& operator=(const BaseRes& other) = default;

    BaseRes(BaseRes&& other) noexcept = default;
    BaseRes& operator=(BaseRes&& other) noexcept = default;
};

struct KLDEPLOY ClassifyRes : public BaseRes {

    friend std::ostream& operator<<(std::ostream& os, const BaseRes& res) {
        os << "ClassifyRes(\n num=" << res.num << ",\n classes=[";
        for (const auto& c : res.classes) {
            os << c << ", ";
        }
        os << "],\n scores=[";
        for (const auto& s : res.scores) {
            os << s << ", ";
        }
        os << "]\n)";
        return os;
    }
};

struct KLDEPLOY DetectRes : public BaseRes {
    std::vector<Box> boxes;

    DetectRes() = default;

    DetectRes(int num, const std::vector<int>& classes, const std::vector<float>& scores, const std::vector<Box>& boxes)
        : BaseRes(num, classes, scores), boxes(boxes) {}

    friend std::ostream& operator<<(std::ostream& os, const DetectRes& res) {
        os << "DetectRes(\n num=" << res.num << ",\n classes=[";
        for (const auto& c : res.classes) {
            os << c << ", ";
        }
        os << "],\n scores=[";
        for (const auto& s : res.scores) {
            os << s << ", ";
        }
        os << "],\n boxes=[\n";
        for (const auto& box : res.boxes) {
            os << " " << box << ",\n";
        }
        os << " ]\n)";
        return os;
    }

    DetectRes(const DetectRes& other) = default;
    DetectRes& operator=(const DetectRes& other) = default;

    DetectRes(DetectRes&& other) noexcept = default;
    DetectRes& operator=(DetectRes&& other) noexcept = default;
};

struct KLDEPLOY OBBRes : public BaseRes {
    std::vector<RotatedBox> boxes;

    OBBRes() = default;

    OBBRes(int num, const std::vector<int>& classes, const std::vector<float>& scores, const std::vector<RotatedBox>& boxes)
        : BaseRes(num, classes, scores), boxes(boxes) {}

    friend std::ostream& operator<<(std::ostream& os, const OBBRes& res) {
        os << "OBBRes(\n num=" << res.num << ",\n classes=[";
        for (const auto& c : res.classes) {
            os << c << ", ";
        }
        os << "],\n scores=[";
        for (const auto& s : res.scores) {
            os << s << ", ";
        }
        os << "],\n boxes=[\n";
        for (const auto& box : res.boxes) {
            os << " " << box << ",\n";
        }
        os << " ]\n)";
        return os;
    }

    OBBRes(const OBBRes& other) = default;
    OBBRes& operator=(const OBBRes& other) = default;

    OBBRes(OBBRes&& other) noexcept = default;
    OBBRes& operator=(OBBRes&& other) noexcept = default;
};

struct KLDEPLOY SegmentRes : public BaseRes {
    std::vector<Box> boxes;
    std::vector<Mask> masks;

    SegmentRes() = default;

    SegmentRes(int num, const std::vector<int>& classes, const std::vector<float>& scores, const std::vector<Box>& boxes, const std::vector<Mask>& masks)
        : BaseRes(num, classes, scores), boxes(boxes), masks(masks) {}

    friend std::ostream& operator<<(std::ostream& os, const SegmentRes& res) {
        os << "SegmentRes(\n num=" << res.num << ",\n classes=[";
        for (const auto& c : res.classes) {
            os << c << ", ";
        }
        os << "],\n scores=[";
        for (const auto& s : res.scores) {
            os << s << ", ";
        }
        os << "],\n boxes: [\n";
        for (const auto& box : res.boxes) {
            os << " " << box << ",\n";
        }
        os << "],\n masks: [\n";
        for (const auto& mask : res.masks) {
            os << " " << mask << "\n";
        }
        os << " ]\n)";
        return os;
    }

    SegmentRes(const SegmentRes& other) = default;
    SegmentRes& operator=(const SegmentRes& other) = default;

    SegmentRes(SegmentRes&& other) noexcept = default;
    SegmentRes& operator=(SegmentRes&& other) noexcept = default;
};

struct KLDEPLOY PoseRes : public BaseRes {
    std::vector<Box> boxes;
    std::vector<std::vector<KeyPoint>> kpts;

    PoseRes() = default;

    PoseRes(int num, const std::vector<int>& classes, const std::vector<float>& scores, const std::vector<Box>& boxes, const std::vector<std::vector<KeyPoint>>& kpts)
        : BaseRes(num, classes, scores), boxes(boxes), kpts(kpts) {}

    friend std::ostream& operator<<(std::ostream& os, const PoseRes& res) {
        os << "PoseRes(\n num=" << res.num << ",\n classes=[";
        for (const auto& c : res.classes) {
            os << c << ", ";
        }
        os << "],\n scores=[";
        for (const auto& s : res.scores) {
            os << s << ", ";
        }
        os << "],\n boxes=[\n";
        for (const auto& box : res.boxes) {
            os << " " << box << "\n";
        }
        os << "],\n kpts=[\n";
        for (const auto& kp_list : res.kpts) {
            os << " [ ";
            for (const auto& kp : kp_list) {
                os << " " << kp << ", ";
            }
            os << " ],\n";
        }
        os << " ]\n)";
        return os;
    }

    PoseRes(const PoseRes& other) = default;
    PoseRes& operator=(const PoseRes& other) = default;

    PoseRes(PoseRes&& other) noexcept = default;
    PoseRes& operator=(PoseRes&& other) noexcept = default;
};

}
}
}
