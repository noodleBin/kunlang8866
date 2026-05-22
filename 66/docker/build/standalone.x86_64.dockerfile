ARG DOCKER_REPO=centuryauto/century
ARG TARGET_ARCH=x86_64
ARG IMAGE_VERSION=18.04-20210914_1336
ARG BASE_IMAGE=${DOCKER_REPO}:runtime-${TARGET_ARCH}-${IMAGE_VERSION}

FROM ${DOCKER_REPO}:data_volume-audio_model-${TARGET_ARCH}-latest as century_audio_volume
FROM ${DOCKER_REPO}:yolov4_volume-emergency_detection_model-${TARGET_ARCH}-latest as century_yolov4_volume
FROM ${DOCKER_REPO}:faster_rcnn_volume-traffic_light_detection_model-${TARGET_ARCH}-latest as century_faster_rcnn_volume
FROM ${DOCKER_REPO}:smoke_volume-yolo_obstacle_detection_model-${TARGET_ARCH}-latest as century_smoke_volume

FROM ${BASE_IMAGE}

COPY output /century

COPY --from=century_audio_volume \
    /century/modules/audio \
    /century/modules/audio

COPY --from=century_yolov4_volume \
    /century/modules/perception/camera/lib/obstacle/detector/yolov4 \
    /century/modules/perception/camera/lib/obstacle/detector/yolov4

COPY --from=century_faster_rcnn_volume \
    /century/modules/perception/production/data/perception/camera/models/traffic_light_detection \
    /century/modules/perception/production/data/perception/camera/models/traffic_light_detection

COPY --from=century_smoke_volume \
    /century/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector \
    /century/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector
