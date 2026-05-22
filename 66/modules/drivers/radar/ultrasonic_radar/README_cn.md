## ultrasonic_radar
该驱动基于Century cyber开发，支持ultrasonic ARS。

### 运行
该驱动需要在century docker环境中运行。
```bash
# in docker
cd /century
source scripts/century_base.sh
# 启动
./scripts/ultrasonic_radar.sh start
# 停止
./scripts/ultrasonic_radar.sh stop
```

### Topic
**topic name**: /century/sensor/ultrasonic_radar
**data type**:  century::drivers::Ultrasonic
**channel ID**: CHANNEL_ID_THREE
**proto file**: [modules/drivers/proto/ultrasonic_radar.proto](https://github.com/CenturyAuto/century/blob/master/modules/drivers/proto/ultrasonic_radar.proto)
