# tiny_platform_mac

小型麥克納姆輪（mecanum）四驅底盤。ROS2 Humble + micro-ROS + ESP32，全部包在 Docker 裡。

短期目標：**用 Switch Pro Controller 遙控這台車。**

```
主機 (Docker / ROS2 Humble)
  ├─ joy_node ──/joy──► tiny_teleop ──/tiny/cmd_vel──┐
  └─ micro_ros_agent ◄────────(USB serial 115200)────┴──► ESP32
                                                          ├─ 逆運動學（車上算）
                                                          ├─ encoder PID → 2× TB6612 → 4 顆馬達
                                                          └─ /tiny/wheel_actual_vel、/tiny/wheel_ticks
```

## 硬體

| 元件 | 規格 |
| --- | --- |
| 主控 | ESP32-D0WD-V3 rev3 / **WROOM-32**（無 PSRAM），38-pin，4 MB flash |
| 馬達 ×4 | Wheeltec **MG513P10_12V**，減速比 10:1，13 線霍爾 encoder（3.3V） |
| 驅動 | 雙 TB6612 集成板 `D64A`，12V 輸入 |
| 輪子 | 麥克納姆 ×4，半徑 40 mm。前後輪距 204 mm、左右輪距 325 mm |
| 手把 | Switch Pro Controller（藍牙，kernel 內建 `hid-nintendo`） |

**接線看 [`docs/WIRING.md`](docs/WIRING.md)，上電校正看 [`docs/BRINGUP.md`](docs/BRINGUP.md)。**

## 快速開始

```bash
cd ~/Desktop/esp/tiny_platform_mac
make up        # 起容器 + agent + 遙控，照正確順序，約 40 秒
```

全綠之後**直接推搖桿就會動**（DIRECT 模式，不用按任何鍵）。

| 動作 | 操作 |
| --- | --- |
| 前進／後退 | 左搖桿 上／下 |
| 左／右平移 | 左搖桿 左／右 |
| 逆時針／順時針 | 右搖桿 左／右 |
| Turbo（0.5 m/s） | 按住 **R** |
| 表情：開心／生氣／疲倦／驚訝 | 按住 **A**／**B**／**X**／**Y** |

表情會出現在 <http://localhost:8088>（`make up` 最後會印出網址）：一張全白的頁面、
兩顆眼睛，開著車的時候眼睛會看向行進方向，停下來就自己眨眼、四處張望，
90 秒沒事就睡著。全螢幕：**F11**（最保險），或頁面上按 `F`／雙擊。
車上那塊螢幕請用 kiosk 模式開，它一啟動就是全螢幕、沒有網址列：

```bash
chromium --kiosk http://localhost:8088     # 或 google-chrome --kiosk
```

```bash
make watch     # 即時看按鍵 / 指令 / 輪速
make status    # 檢查每一層
make down      # 收工
```

⚠️ **沒有 deadman。** 停車只剩手把斷線 1 秒、韌體 500 ms 看門狗、電源開關。
落地跑的時候手邊要有斷電開關。要改回「按住 L 才動」見 [`docs/GUIDE.md`](docs/GUIDE.md)。

**第一次用、或壞掉了 → 看 [`docs/GUIDE.md`](docs/GUIDE.md)**（從零開始的完整流程、
故障對照表、LED 判讀、校正工具一覽）。

### 其他指令

```bash
make install-udev              # 一次性：裝 udev 規則 → /dev/esp32
make build                     # 一次性：建映像（首次幾十分鐘）
make flash SKETCH=tiny_open    # 編譯 + 燒錄韌體
make rebuild-ws                # 改過 ros2_ws/src/ 之後重建
make shell                     # 進容器
make logs                      # 看 teleop 的輸出
make logs-face                 # 看表情頁面（face_node）的輸出
```

## 目錄

```
├── CLAUDE.md            # 給 Claude Code 的指引，含所有踩過的坑（先讀這個）
├── Dockerfile           # 自我完備的映像：ROS2 + micro-ROS + arduino-cli + esp32 core
├── Makefile             # build / run / attach / agent / flash / install-udev
├── docs/
│   ├── GUIDE.md         # 操作指南：日常、從零、故障排除（先看這個）
│   ├── WIRING.md        # 接線表、GPIO 預算、通電前檢查
│   └── BRINGUP.md       # 上電與校正 SOP（階段 A0~F）
├── firmware/
│   ├── tiny_probe/      # 拋棄式：鑑定 WROOM vs WROVER
│   └── tiny_open/       # 階段 1：開環，用來校正方向
├── ros2_ws/
│   ├── dds/             # Fast-DDS localhost unicast profile
│   └── src/tiny_teleop/ # 手把遙控（joy_teleop / joy_probe / launch / params）
├── scripts/             # tiny.sh（make up 的實作）、start-agent、udev
└── tools/               # 校正與診斷腳本（8 支，見 GUIDE.md 第 6 節）
```

## 三件會咬人的事

1. **GPIO12 絕對不接。** 它是 MTDI strapping pin，開機被拉高就 bootloop，
   而且「燒得進去、卻開不了機」，`erase_flash` 重燒都救不回來。
2. **agent 必須先於 ESP32 開機。** 韌體的 `error_loop()` 沒有退路。
   一律用 `make agent`（它會處理 CH340 的 RTS），**不要跑裸的 `micro_ros_agent`**。
3. **`ros2 topic list` 剛起來是空的很正常**，等 10 秒。
   即時可信的檢查是 `ros2 topic hz /tiny/wheel_ticks`。

其餘的坑（Arduino core 版本鎖定、`IRAM_ATTR` link 失敗、容器內禁跑 `udevadm`、
`ROS_DOMAIN_ID` 對 agent 無效…）全部記在 [`CLAUDE.md`](CLAUDE.md) 第 3 節。

## 授權

MIT，見 [LICENSE](LICENSE)。

`cachefile/gitstatus/gitstatusd-linux-x86_64` 是
[powerlevel10k](https://github.com/romkatv/powerlevel10k) 的預編二進位（MIT），
放在版控裡是為了讓 `make build` 不必連網下載它。
`ros2_ws/src/tiny_teleop` 與韌體的 `Encoder.h` / `Motor.h` / `Pid.h` / `KalmanFilter.h`
改寫自作者自己的另一台底盤專案。
