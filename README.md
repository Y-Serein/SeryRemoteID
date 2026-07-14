# SeryRemoteID

ESP-IDF 原生 SeryRemoteID 固件，不继承 Arduino 运行时、旧工程工具或 managed_components。

当前已实现：

- MAVLink `OPEN_DRONE_ID_*` 串口输入、ArmStatus、参数服务、SecureCommand。
- DroneCAN RemoteID 输入、动态节点分配、NodeStatus、ArmStatus、参数服务、SecureCommand。
- OpenDroneID 状态聚合、BasicID 参数持久化、arming 状态判断。
- WiFi Beacon Vendor IE 和 WiFi NAN action frame 广播。
- BLE 4 legacy advertising 和 BLE 5 coded extended advertising。
- Web 状态页、`/ajax/status.json` RemoteID 状态接口和 SeryRemoteID OTA 更新；OTA 校验 ESP-IDF app descriptor、项目名和 secure version，高锁级设备继续要求旧格式 Board ID + Monocypher 签名 descriptor。
- NVS 参数表，包含 `LOCK_LEVEL`、`CAN_NODE`、`UAS_ID`、`BT4_RATE`、`BT5_RATE`、`PUBLIC_KEY1..5` 等 SeryRemoteID 参数。
- 首次初始化默认写入 ArduPilot、BlueMark、CUAV 公钥到 `PUBLIC_KEY1..5`，后续可通过 SecureCommand/参数接口管理。
- LED/板级状态指示模块，默认 GPIO 关闭。

## 构建

```bash
cd firmware
export IDF_PATH=/tmp/esp-idf-v5.5.4
export IDF_TOOLS_PATH=/tmp/espressif-tools
. "$IDF_PATH/export.sh"
idf.py build
```

已验证 ESP-IDF v5.5.4 / target `esp32s3` 可编译通过。建议用独立 build 目录：

```bash
IDF_PATH=/tmp/esp-idf-v5.5.4 \
IDF_TOOLS_PATH=/tmp/espressif-tools \
bash -lc '. "$IDF_PATH/export.sh" >/tmp/seryremoteid-idf-export.log && cd firmware && idf.py -B /tmp/seryremoteid-build build'
```

源码包可以交给别人直接编译，前提是：

- 已安装 ESP-IDF，建议 5.5.x。
- 保留 `firmware/components/` 下的本地组件：`opendroneid`、`mavlink`、`monocypher`、`canard`、`dronecan_generated`。
- 不需要 Arduino、ESP Component Manager 在线下载或旧工程工具。
- 不打包 `firmware/build/`、`firmware/managed_components/`、`dependencies.lock` 等生成物；`firmware/sdkconfig.defaults` 已包含默认构建配置。

## 默认硬件

默认按 ESP32-S3 DevKit 风格配置：

- MAVLink UART: `UART1`
- TX: `GPIO18`
- RX: `GPIO17`
- Baud: `57600`
- WiFi Channel: `6`
- DroneCAN TWAI TX: `GPIO47`
- DroneCAN TWAI RX: `GPIO38`
- DroneCAN bitrate: `1Mbps`

具体默认值在 `firmware/main/config.h`。

## 结构

- `firmware/main/app_main.c`: 系统初始化和任务启动
- `firmware/main/cfg.*`: NVS 配置层
- `firmware/main/protocol/rid_state.*`: RemoteID 状态聚合
- `firmware/main/transport/rid_mavlink.*`: MAVLink 串口输入
- `firmware/main/transport/rid_dronecan.*`: DroneCAN 输入和服务
- `firmware/main/net/rid_wifi.*`: WiFi Beacon/NAN 输出
- `firmware/main/net/rid_ble.*`: BLE 4/5 输出
- `firmware/main/web/`: Web 状态页和签名 OTA
- `firmware/components/opendroneid`: OpenDroneID C 编码库
- `firmware/components/mavlink`: MAVLink generated headers
- `firmware/components/canard`: libcanard C 库
- `firmware/components/dronecan_generated`: DroneCAN DSDL generated C 代码

## 验证范围

- 已验证：ESP-IDF 5.5.4 编译通过，生成 `/tmp/seryremoteid-build/sery_remoteid.bin`。
- 未验证：真实 ESP32-S3 上板运行、WiFi/BLE 实际广播抓包、DroneCAN 总线互通、Web OTA 实刷。

## 合规提醒

本固件只是 RemoteID 发射链路实现。实际产品是否满足 FAA/EU 等法规，取决于 UAS ID、Operator ID、系统分类、广播介质、发射功率、证书和整机 DoC 等配置与认证流程。
