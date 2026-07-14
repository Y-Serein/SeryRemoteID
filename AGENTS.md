# SeryRemoteID Project Rules

本仓库目标是构建 ESP-IDF 原生 RemoteID 固件，不继承 Arduino 运行时。

- 固件主工程在 `firmware/`。
- 业务入口在 `firmware/main/app_main.c`。
- 硬件和默认参数在 `firmware/main/config.h`。
- 运行时配置在 `firmware/main/cfg.*`，NVS namespace 为 `storage`，兼容读取旧 namespace `sery_rid` 的短键。
- MAVLink 和 DroneCAN 输入在 `firmware/main/transport/`。
- WiFi RemoteID 广播在 `firmware/main/net/`。
- OpenDroneID、MAVLink、Monocypher、libcanard、DroneCAN generated 代码作为本地 ESP-IDF components 放在 `firmware/components/`。

## 目录组织规则：常见项目结构

新建文件优先遵守项目已有目录结构；不要把临时文件、测试输出、分析草稿平铺到项目根目录。

常用目录：

```text
docs/          # 正式文档
docs/design/   # 设计文档、架构、方案
docs/adr/      # 架构决策记录，可选
docs/reference/# 芯片资料、协议资料、外部参考
docs/runbooks/ # 操作手册、排障流程
docs/release/  # 发布说明、验收记录

scripts/       # 项目脚本，偏工程流程
tools/         # 独立小工具，偏可复用工具
tests/         # 测试
examples/      # 示例
assets/        # 正式资源
build/         # 构建产物，通常忽略
out/           # 临时输出或生成结果
```

- 正式文档优先放 `docs/`；设计文档、架构、方案优先放 `docs/design/`。
- 项目脚本优先放 `scripts/`；独立可复用工具优先放 `tools/`。
- 测试、示例和正式资源分别优先放 `tests/`、`examples/`、`assets/`。
- 构建产物和临时输出优先放 `build/` 或 `out/`，并按项目规则忽略。
- 文档命名要清晰，不要使用 `final_final`、`new_new`、`test1` 这类名字。
- 如果同一任务产生多个文件，必须说明每个文件的用途。
- 如果对应目录不存在，创建前先说明原因和将要创建的目录。

---

默认验证命令：

```bash
cd firmware
source /opt/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

注意事项：

- 不要引入 `arduino-esp32`、`Adafruit_NeoPixel` 或 Arduino `setup()/loop()`。
- 不要从其他 Arduino RemoteID 工程搬入 `managed_components`。
- 不要依赖其他工程的 `tools/`、`scripts/` 或 ESP Component Manager 在线拉取组件。
- `firmware/components/mavlink` 和 `firmware/components/dronecan_generated` 是生成后的协议代码，可以作为本地组件随源码分发。
- 当前目标是 ESP-IDF 原生实现完整 SeryRemoteID 功能：MAVLink、DroneCAN、WiFi、BLE、Web OTA、SecureCommand 都应保持可编译。
- RemoteID 合规配置由使用者负责，默认配置只用于开发联调。
