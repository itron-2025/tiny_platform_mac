# CLAUDE.md

本檔提供給 Claude Code 在此 repo 工作時的指引。**請務必遵守。**

---

## 0. 與使用者協作的規則（最高優先）

- **溝通語言：繁體中文。**
- **開發語言：以 Python 為主**（ROS2 節點）。韌體是 Arduino C++。
- **程式註解：一律使用全英文。**
- **使用者的 ROS 程度**：對 ROS2 幾乎沒有實作經驗，只大概用過 ROS1。
  凡是牽涉 ROS 的內容，都要用初學者能理解的方式說明，必要時對照 ROS1 的做法（見第 6 節）。
- 規格不明、或牽涉到地基決策（版本、硬體協定、腳位、檔案結構）時，**直接詢問使用者**，
  不要自行假設帶過。這是使用者在 `spec.md` 明確要求的。

---

## 1. 專案概觀

**tiny_platform_mac** —— 一台小型麥克納姆輪（mecanum）四驅底盤。

短期目標（`spec.md`）：**用連在主機上的 Switch Pro Controller 遙控這台車。**

| 元件 | 說明 |
| --- | --- |
| 主機 | 開發＋執行機（目前是筆電，Ubuntu 22.04）。跑 Docker 容器，micro-ROS agent 在此執行。 |
| ESP32 | **ESP32-D0WD-V3 rev3 / WROOM-32（無 PSRAM）**，38-pin 開發板，4 MB flash。CH340 USB-serial（`1a86:7523`）。透過 micro-ROS 成為 ROS2 節點 `/tiny/base`。 |
| 馬達 ×4（含 encoder） | **Wheeltec MG513P10_12V**，減速比 10:1，13 線霍爾 encoder（3.3V）。 |
| 雙 TB6612 集成板 ×1 | 型號 `D64A`（少見，無公開資料）。12V 輸入，一片驅 4 顆。**STBY 只有一根，已跳線直接接 3V3** → GPIO4 空出來成為全機唯一備用腳，代價見 `docs/WIRING.md` 5.3。 |
| Pro Controller | 藍牙遙控輸入。主機 kernel 內建 `hid-nintendo`，配對在**主機端**做。 |

### 控制資料流

```
主機 (Docker / ROS2 Humble)
  ├─ joy_node ──/joy──┬─► tiny_teleop ──/tiny/cmd_vel──┐
  │                   └─► face_node ──http/SSE :8088──► 瀏覽器（機器人的臉）
  └─ micro_ros_agent ◄─────────(USB serial 115200)─────┴──► ESP32
                                                       ├─ 逆運動學（車上算）
                                                       ├─ encoder PID → 2× TB6612 → 4 顆馬達
                                                       └─ 回報 /tiny/wheel_actual_vel、/tiny/wheel_ticks
```

### 開發階段

| 階段 | 韌體 | 目的 | 狀態 |
| --- | --- | --- | --- |
| 1 | `firmware/tiny_open/` | **開環**。驗證接線、校正四輪轉向與 encoder 正負號 | ✅ 完成。四輪方向已校正並複驗（2026-09-08） |
| 2 | `firmware/tiny_pid/` | **閉環** FF+PID + 卡爾曼，正式遙控用 | 待階段 1 實車校正 |

**為什麼先開環**：方向接反時，開環很好判讀（輪子轉錯邊，一眼看得出來）；
閉環會把方向錯誤變成「積分器暴衝或卡死」，症狀完全不直觀。

---

## 2. 這個專案與 `~/Desktop/Tomcat_with_Docker` 的關係

Tomcat 是**另一台**已經跑通的麥克納姆底盤，用的是**另一片** ESP32。
（兩片都是 CH340，udev 分辨不出來，見 3.8；要確認手上這片是哪一台的，
用 `esptool.py --port /dev/esp32 chip_id` 讀 MAC 跟自己的紀錄比對。）
本專案**沿用它的程式碼與經驗，但刻意獨立**：

- **映像獨立**：`tiny-platform:latest`，自己一份 Dockerfile。Tomcat 那邊重建映像不會影響這裡。
- **腳位獨立**：使用者拍板改良腳位、避開 strapping pin（見第 3 節），因此
  **兩台車的 `config.h` 不可互換**。
- **可以照抄的**：Dockerfile 結構、`scripts/`、DDS profile、韌體的
  `Encoder.h` / `Motor.h` / `Pid.h` / `KalmanFilter.h`、ROS2 節點的數學。
- **不可照抄的**：任何腳位、機構常數、PID 增益、`COUNTS_PER_WHEEL_REV`——
  那些是 Tomcat 那台車的實測值。

---

## 3. 🚨 從 Tomcat 繼承的地雷（每一項都是真的踩過的）

### 3.1 GPIO12 是 MTDI strapping pin——會造成「燒得進去、卻開不了機」

Tomcat 把 `MOT_PWM[FL]` 接在 GPIO12，某次接觸不良讓它開機時被拉高 →
ROM 把 flash 供電（VDD_SDIO）設成 1.8V → 第二階段 bootloader 用 80MHz 存取 flash 失敗 →
**每 27 ms 重開一次**。當時 `erase_flash` 全抹重燒、四段 hash 全過，**還是開不了機**。

- **指紋**：燒錄模式（GPIO0 拉低）完全正常，一般開機就 bootloop。
- **定位方法**：用 pyserial 開埠 115200，拉 RTS 重置後純聽 4 秒。
  正常只會看到**一次** boot ROM banner；bootloop 會看到同一段重複上百次。
- **本專案的做法**：**GPIO12 一律不接任何東西。** 見 `docs/WIRING.md`。
- 緊急 workaround（硬體還沒修、又急著用）：
  `--fqbn esp32:esp32:esp32:FlashFreq=40,FlashMode=dout`。
  ⚠️ 這只是繞過，而且**之後每次燒錄都必須帶同一組參數**，忘了帶就再次開不了機。

其他 strapping pin 的處置：

| GPIO | 角色 | 本專案的處置 |
| --- | --- | --- |
| 0 | BOOT 鍵，開機須為高 | 不接（30-pin 板子本來就沒拉出來） |
| 2 | 下載模式須為低/浮接 | 只當板載 LED 狀態燈 |
| 5 | 開機須為高，會輸出訊號 | 可用，但別接會被外部拉低的東西 |
| 12 | **MTDI，開機須為低** | **絕對不接** |
| 15 | MTDO，開機須為高 | 可用 |
| 6–11 | 內部 flash | 絕對不接 |
| 1, 3 | UART0（micro-ROS 走這條） | 絕對不接 |
| 34–39 | input-only、無內部上下拉 | 保留給 encoder |

### 3.2 啟動順序：agent 必須「先」跑起來，ESP32 才能開機

韌體的 `RCCHECK()` 在任何 `rcl_*` 失敗時跳進 `error_loop()`，而它是
`while(true){ toggle LED; delay(100); }`——**沒有退路、不會重試**。
所以 ESP32 開機時 agent 不在，它就**永久**卡在閃燈迴圈；這時才開 agent 也沒用。

- 症狀是 `ros2 node list` 空的，會被誤判成 DDS / baud rate / micro-ROS 版本問題。
- **正確順序**：先讓 agent 進入 listening，**再讓 ESP32 重新開機**。
- `scripts/start-agent.sh` 已自動處理：它利用 CH340「開埠就拉 RTS 壓住 EN」這個平常很煩的特性
  當成免費的自動重置——
  ```
  1. agent 開 serial 埠 → 核心拉高 RTS → 板子被按在 reset
  2. agent 進入 listening
  3. release_rts.py 放開 RTS → 板子這才開機
  4. setup() 執行時 agent 早就在等了 → 握手成功
  ```
- 🚫 **絕對不要跑裸的 `ros2 run micro_ros_agent`**，一律用 `start-agent.sh` / `make agent`。

### 3.3 Arduino ESP32 core 必須鎖在 2.0.x（本專案用 2.0.17）

**強制，非偏好。** `micro_ros_arduino` 是 `precompiled=true`，只出一包用 ESP-IDF v4.4 編好的
`libmicroros.a`，不會跟著你的 core 重編。core 3.x 用 IDF v5，ABI 不相容。

- 看到 `'ledcSetup' was not declared in this scope` → **是 core 版本錯**，
  **不要**去改韌體的 LEDC 呼叫。那只會讓編譯過關，然後死在 link 或上電 crash。
- 升級 core 前先確認 `micro_ros_arduino` 有對應版本（官方最新 `2.0.8-humble` 至今仍不支援 3.x）。

### 3.4 class 內定義的 ISR 不可加 `IRAM_ATTR`

會在 link 階段炸 `dangerous relocation: l32r: literal placed after use`：
class 內定義 → 隱含 `inline` → GCC 把程式碼放進 `.iram1`（低位址）卻把 literal pool 留在
flash（高位址），Xtensa 的 `l32r` 只能往低位址取 literal。

- 本專案的 GPIO ISR service 用 flag `0` 註冊（`CONFIG_ARDUINO_ISR_IRAM` 未開），
  ISR 待在 flash 完全合法。
- 因此**刻意不裝外部 Encoder library**——PJRC 版會把 `IRAM_ATTR` 帶回來、重現這個 link 失敗。
  `Encoder.h` / `Motor.h` 是 sketch 內的 vendored 本地檔。

### 3.5 udev 只在主機裝，容器內絕對禁止跑 `udevadm`

容器是 `--privileged --net=host`，和主機**共用 kernel uevent netlink socket**。
在容器內 `udevadm trigger` 會讓**主機**重新列舉所有裝置 → 桌面崩潰、被踢回登入畫面、甚至死當。

- 正確做法：主機上 `make install-udev`。
- 因為 `-v /dev:/dev` 是真 bind mount，主機建好的 symlink 容器內自動看得到。
- 即使在主機上，`udevadm trigger` 也要**限定 scope**（`--attr-match=...`），
  否則會連使用者正在打字的鍵盤滑鼠一起 re-fire。

### 3.6 手把用 `joy`，不是 `joy_linux`

這台主機**沒有 `/dev/input/js*` 節點**（`hid-nintendo` 沒有 joydev 綁定），
而 `joy_linux` 正是讀 `jsX` 的。ROS1 時代的手把教學幾乎都假設 `jsX`，在這裡是死路。

- `joy` 走 SDL2 / evdev (`/dev/input/event*`) 才對。
- ⚠️ `joy` **不吃裝置路徑**，只有 `device_name` / `device_id`，
  而且 **SDL 的名字是 `Nintendo Switch Pro Controller`**，跟 evdev 的 `Pro Controller` 不同。
- 軸／按鍵編號**必須實測**，不要抄文件（換手把、換 SDL 後端都會變）。

### 3.7 `pkill -f` 會比對到自己那行指令

`pkill -f micro_ros_agent` 會把「自己這行指令」也算進去 → 自殺或誤判有殘留 agent。
**寫成 `pkill -f 'micro_ros[_]agent'`**（中括號讓 pattern 不匹配自身）。
`joy_node` 同理：`pkill -f 'joy[_]node'`。

**但中括號只保護 pattern 本身**——如果同一行指令裡別的地方還有字面的
`micro_ros_agent`，一樣會自殺。2026-09-08 實際踩到：一行同時有
`pkill -f "micro_ros[_]agent"` 和 `micro_ros_agent --help`，後者讓 pkill 匹配到自己，
整個 shell 被 `kill -9`（exit 137）。

**而且上面那個「先取 PID 再殺」的寫法也一樣會中招**，只要 pattern 出現在同一行：

```bash
for p in $(pgrep -f "topic pub"); do kill $p; done   # ← shell 殺了自己
```

2026-09-08 這個坑咬了三次，包括這一次。

**唯一可靠的規則：pgrep 和 kill 要拆成兩個獨立的指令。**
先讓 `pgrep` 印出 PID，看過之後再用**字面的數字**去 kill：

```bash
docker exec C bash -c 'ps -eo pid,stat,args | grep -E "joy_node" | grep -v grep'
docker exec C kill -CONT 594          # 用實際數字，不要再帶 pattern
```

順帶一提，`ps` 的 `STAT` 欄要一起看：`T` 是 stopped（被 SIGSTOP 停住）、
`Z` 是 zombie。這兩種都還會被 `pgrep` 找到，但都不是「正常在跑」。

### 3.8 兩片 CH340 在 udev 眼中一模一樣

這片板子的 CH340 **沒有 `ATTRS{serial}`**，只有通用的 `1a86:7523`。
Tomcat 那片也是。所以**兩片同時插著時，`/dev/esp32` 指向誰是不確定的**——
燒錯車的韌體會是一個很難察覺的失敗。詳見 `scripts/esp32.rules` 的註解（含 port-path 匹配的替代寫法）。

### 3.9 Fast-DDS 的 `maxInitialPeersRange` 預設只有 4

`ros2_ws/dds/fastdds_localhost.xml` 把它設成 64，**不要改回去**。
沒有初始 peer 埠號的 `127.0.0.1` 會被展開成 participant ID 0..range-1 的 well-known 埠，
預設只有 4 個。第 5 個以後的 DDS participant（每個節點、CLI daemon、agent 的兩個 participant
都算）就探索不到彼此——症狀是 `ros2 topic info` 看得到 publisher、agent 也確實在寫，
但新的 subscriber 一筆都收不到。

### 3.10 🚨 `ROS_DOMAIN_ID` 對 micro-ROS agent **無效**——不要用它做隔離

**實測結論（2026-09-08，本專案自己量的）**：預編的 `micro_ros_agent` 直接 link Fast-DDS、
不經過 RMW，所以它**從不讀 `ROS_DOMAIN_ID`，永遠把 participant 建在 DDS domain 0**。
（跟它無視 `RMW_IMPLEMENTATION` 是同一個成因。）

- 實驗：把 agent 的環境設成 `ROS_DOMAIN_ID=7` 啟動 → 它的 topic 仍然出現在 **domain 0**，
  domain 7 完全空的。`micro_ros_agent --help` 也根本沒有 domain 選項
  （`-d/--discovery` 是 discovery **port**，不是 domain）。
- 所以設 `ROS_DOMAIN_ID` **比不設更糟**：我們自己的節點會搬到 domain 7，板子留在 0，
  `/cmd_vel` 永遠到不了輪子。

**本專案改用 namespace 隔離**：韌體的節點與所有 topic 都在 `/tiny/` 底下
（`/tiny/base`、`/tiny/cmd_vel`、`/tiny/wheel_duty`、`/tiny/wheel_actual_vel`、
`/tiny/wheel_ticks`），見 `firmware/*/config.h` 的 `MICROROS_NAMESPACE`。

**為什麼非做不可**：Tomcat 底盤的容器同樣用 `--net=host` + 同一份 127.0.0.1 unicast
DDS profile 跑在這台主機上。實測從本容器內看得到它的 `/joy_node`、`/joy_teleop`、
`/camera_web_stream`，而且 **`/cmd_vel` 顯示「我們的板子」是它的訂閱者**——
Tomcat 的手把按下 deadman 會直接開走這台車。兩邊當初還都把節點取名 `esp32_base`。

⚠️ 韌體的 topic 名稱寫**相對名稱**（`cmd_vel`），由 rcl 自動套上 namespace。
**不要手動加 `/tiny/` 前綴**，會變成 `/tiny/tiny/cmd_vel`。

### 3.11 `ros2 node list` / `topic list` 需要約 10 秒才穩定，`topic hz` 才是即時的

剛起 agent 就查清單常常是空的，**這不代表沒連上**。實測：
`ros2 topic list` 空的、`ros2 node list` 空的，但 `ros2 topic hz /tiny/wheel_ticks`
當下就讀到 **50.0 Hz**——訂閱是靠 DDS endpoint 直接 match 的，不靠 ROS graph。

**診斷順序因此要改**（覆寫第 7 節的舊順序）：
1. 先 `ros2 topic hz /tiny/wheel_ticks` —— 有 50 Hz 就代表**板子與 agent 都正常**。
2. 清單空的就等 10 秒再查，必要時 `ros2 daemon stop`（CLI daemon 會快取舊環境）。
3. **不要**因為清單是空的就重跑 `start-agent.sh`——那會製造第二個 agent，
   而兩個 agent 各讀一半 XRCE byte stream，症狀跟「完全沒有 agent」一模一樣。

### 3.12 zombie agent 會讓 `start-agent.sh` 永久拒絕啟動

`start-agent.sh` 用 `pgrep` 判斷「是不是已經有 agent 在跑」。
**但 zombie（`State: Z`）也會被 pgrep 找到**，而 zombie 既不佔 serial 埠也沒有 DDS session。
容器的 PID 1 如果不 reap（例如 CMD 是裸的 `sleep`），屍體會永遠留著，
這個防呆就會**永遠**拒絕啟動新 agent，同時 `ros2 topic hz` 什麼都收不到——
看起來完全就是「沒有 agent」。2026-09-08 實際踩到。

**已修**（兩層）：
- `scripts/start-agent.sh` 現在會過濾掉 `Z` 狀態的行程。
- `Makefile` 的 `run` 加了 `--init`，讓 tini 當 PID 1 負責 reap。

### 3.13 「行程還在」不等於「還在工作」——手把重連後 joy_node 會變成活死人

2026-09-08 實測：`make up` 印出三行全綠，但手把完全沒反應。
`ps` 看得到 joy_node、藍牙 `Connected: yes`、`/dev/input/event10` 也在——**但 `/tiny/joy` 一筆都沒有**。

**根因是時序**：手把待機斷線後又自動重連，核心重建了 `event10` 與 `hidraw7`
（時間戳比 joy_node 的啟動時間**晚**）。SDL 在啟動時就列舉完裝置，之後只會對著
**舊的**節點重試，於是無窮迴圈地印：

```
[joy_node] Unable to open joystick 0: Couldn't read device info
```

行程活著、CPU 也在跑，就是永遠開不了裝置。**重啟 joy_node 讓 SDL 重新列舉即可。**

這跟 3.12 的 zombie agent 是**同一類錯誤**：用「PID 存在」判斷健康。
**已修**：`scripts/tiny.sh` 的每一層現在都以**該層的 topic 有沒有資料**為判準，
行程在但 topic 沒聲音就自動重啟那一層。

> 一般化的教訓：這個系統裡每一層都有「看起來活著但沒在工作」的狀態——
> agent 的 zombie、joy_node 的失效裝置控制代碼、兩個 agent 互搶 byte stream。
> **健康檢查一律要看資料流，不要看行程表。**

### 3.14 🚨 `start-agent.sh` 的 RTS 釋放有競態——板子會被永久按在重置

`start-agent.sh` 啟動 agent 後固定 `sleep 2` 才跑 `release_rts.py`。
**容器剛開機時 agent 載入比較慢**，2 秒還沒開埠，於是順序變成：

```
release_rts 先跑（此時埠還沒開，放開的是「沒人按著」的 RTS，等於沒作用）
  ↓
agent 才開埠 → 核心拉高 RTS → 板子進入重置
  ↓
再也沒有人放開它 → 板子永遠開不了機
```

**症狀極具欺騙性**：agent 行程健康、板子沒壞、`make status` 只有
`FAIL no /tiny/wheel_ticks` 這一行，跟「板子燒壞了」完全一樣。
2026-09-08 燒完韌體後立刻踩到。

**定位法**：停掉 agent，用 pyserial 直接聽 serial。看到
`rst:0x1 (POWERON_RESET)` 一次 + 一堆 `XRCE` → **板子是好的**，
問題純粹在 RTS 被壓著。

**已修**：`scripts/tiny.sh` 的 `wait_for_esp32()` 在等遙測的同時**每 6 秒重放一次 RTS**
（`release_rts.py` 是冪等的，多放無害），直到 topic 出現為止。
畫面上會看到 `....R....R` 的 `R` 就是重放。

> 手動救援（agent 已經在跑、但板子沒握手）：
> ```bash
> docker exec tiny-platform python3 /root/scripts/release_rts.py /dev/esp32 115200
> ```

### 3.15 🚨 兩個 publisher 搶 `cmd_vel` 會讓閉環看起來壞掉

`joy_teleop` 在 DIRECT 模式下是**永久武裝**的，原本會以 20 Hz 持續發布零指令。
於是任何用 `/tiny/cmd_vel` 驅動的工具（`cmd_vel_check.py` 等）都在跟它**同頻互搶同一個 topic**。

**這不是外觀問題，是會毀掉控制的**：韌體把零指令視為「停止」，而停止會
**`pid.reset()` 清掉積分器**。零指令和真實指令交替出現 → **積分器每隔一個 tick 就被清空** →
閉環永遠累積不起來。

實測數字（2026-09-08，自轉 `wz = 2.0`）：

| 情境 | 追蹤誤差 |
| --- | --- |
| teleop 在背景閒置發零 | **54%**（看起來像 PID 壞了） |
| 停掉 teleop | 3% |
| 修好之後、teleop 照常開著 | **2.8%** |

當時第一反應是「PID 沒調好」，但**控制器沒問題，它是被打斷的**。

**已修**：`joy_teleop` 新增 `idle_quiet_after`（預設 1.0 秒）——
搖桿回中超過這個時間就**完全停止發布**。放開搖桿仍會送出一小段零指令確保停車，
之後由韌體自己的 500 ms 看門狗維持停止狀態（那本來就是它的職責）。

> 一般化：**在這個系統裡，「有東西在發 topic」不等於「只有它在發」。**
> 除錯控制問題時先看 `ros2 topic info /tiny/cmd_vel` 的 `Publisher count`——
> 大於 1 就先解決那個，再談調參。

---

## 4. 環境與指令

### 主機端（一次性）

```bash
make install-udev      # 裝 udev 規則 → /dev/esp32、/dev/input/pro_controller
make build             # 建映像 tiny-platform:latest（首次幾十分鐘）
```

### 日常（平常只需要這三個）

```bash
make up                # 起容器 + agent + 遙控 + 表情，照正確順序，且每層以資料流判斷健康
make watch             # 手把即時儀表板（按鍵 / 指令 / 輪速）
make down              # 收工
```

`make up` 最後會印出表情頁面的網址（預設 <http://localhost:8088>）。
A/B/X/Y = 開心／生氣／疲倦／驚訝，行駛時眼睛看向行進方向。

`make up` 是冪等的：已經在跑的層會跳過，**但若某層行程在卻沒有資料流，會自動重啟那一層**
（見 3.12、3.13）。

### 其他

```bash
make status            # 只檢查不動作
make logs              # teleop 的輸出
make logs-face         # face_node 的輸出（表情頁面起不來時看這個）
make shell             # 進容器
make rebuild-ws        # 改過 ros2_ws/src/ 之後（含 teleop_params.yaml）
make flash SKETCH=tiny_open    # 停 agent → 編譯 → 燒錄
make run / attach / agent      # 舊的手動流程，除錯時才用
```

### 容器內

```bash
# 編譯 / 燒錄（先停 agent，它佔著 serial 埠）
pkill -f 'micro_ros[_]agent'
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/tiny_open
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/esp32 firmware/tiny_open
/root/scripts/start-agent.sh /dev/esp32 115200

# ROS2 workspace
cd $TP_PROJ/ros2_ws && colcon build --symlink-install && source install/setup.zsh
```

---

## 5. 慣例

- 程式碼註解一律英文；與使用者的對話一律繁體中文。
- ROS2 行為用 **launch 檔的參數**調整，不要把數值寫死在 Python 節點裡。
- **所有會因車而異的東西（腳位、機構常數）必須集中在 `config.h`**，主程式不得出現裸露腳位號。
- 改動牽涉硬體協定（topic 名稱、訊息格式、baud rate、`COUNTS_PER_WHEEL_REV`）時，
  **主機端與 ESP32 端兩邊都要同步**，否則會對不上。
- 韌體控制迴圈內**禁止**阻塞呼叫：`delay()`、`Serial.print()`、`String` 配置、`malloc`。
  （micro-ROS 佔著 serial，本來也不能 `Serial.print`——**LED 是唯一的狀態通道**。）
- 任何腳位改動都要同步更新 `docs/WIRING.md`。

---

## 6. 給 ROS1 使用者的 ROS2 對照表

| 概念 | ROS1 | ROS2（本專案） |
| --- | --- | --- |
| **核心通訊** | 需要 `roscore` | **沒有 roscore**！用 DDS 自動探索 |
| 建置工具 | `catkin_make` | `colcon build` |
| Workspace | `catkin_ws/`，`source devel/setup.bash` | `ros2_ws/`，`source install/setup.bash` |
| 跑單一節點 | `rosrun pkg node` | `ros2 run pkg node` |
| 啟動檔 | `roslaunch pkg x.launch`（XML） | `ros2 launch pkg x.launch.py`（**Python**） |
| 看 topic | `rostopic list` / `echo` | `ros2 topic list` / `ros2 topic echo` |
| 參數 | `rosparam`（全域 server） | `ros2 param`（**參數綁在各節點上**） |
| Python 套件 | `package.xml` + `CMakeLists.txt` | `package.xml` + `setup.py`（`ament_python`） |
| 嵌入式橋接 | `rosserial` | **`micro-ROS`** |

> 解釋指令時，盡量附一句「這相當於 ROS1 的 XXX」。

---

## 7. 「`ros2 node list` 看不到 `/tiny/base`」的診斷順序

照做，不要跳步。這個症狀有至少四種完全不同的成因。

0. **先做這個**：`ros2 topic hz /tiny/wheel_ticks`。讀到 ~50 Hz
   → 板子與 agent 都正常，問題只是清單還沒穩定（見 3.11），到此為止。

1. **agent 有沒有在跑？** `pgrep -f 'micro_ros[_]agent'`
   （⚠️ zombie 不算，見 3.12）
   - 沒有 → `make agent`。
   - **有兩個以上 → 這就是原因。** 兩個 agent 各讀一半的 XRCE byte stream，
     誰都握不住 session，症狀跟「完全沒有 agent」一模一樣。
     `pkill -f 'micro_ros[_]agent'` 後重起一個。
2. **板子有沒有開機？** 看 LED（GPIO2）：
   - 快閃 = 韌體在跑但找不到 agent（→ 順序錯了，重跑 `start-agent.sh`）
   - 恆亮 = agent 已連上且正在收 `/cmd_vel`
   - **完全不亮 = 板子沒開機** → 走 3.1 的 bootloop 定位流程
3. **RMW 一致嗎？** `echo $RMW_IMPLEMENTATION` 應為 `rmw_fastrtps_cpp`，
   `echo $FASTRTPS_DEFAULT_PROFILES_FILE` 應指向存在的檔案。
   改過之後 `ros2 daemon stop`（CLI daemon 會快取舊環境）。
4. **baud 對嗎？** 韌體與 `start-agent.sh` 都是 115200。

---

## 8. 進度與待辦

### 已完成（2026-09-08）

- [x] 專案骨架、`Dockerfile`、`Makefile`、`scripts/`
- [x] 映像 `tiny-platform:latest` 建置並驗證（core 2.0.17 / micro_ros_arduino 2.0.7-humble / joy）
- [x] 板型鑑定：**WROOM-32 無 PSRAM** → GPIO16/17 可用（`firmware/tiny_probe` 實機測）
- [x] `docs/WIRING.md` 腳位表定案（避開 GPIO12；STBY 在 GPIO4 + 下拉）
- [x] `firmware/tiny_open/` 開環韌體，含 `/tiny/wheel_duty` 逐輪校正通道
- [x] `docs/BRINGUP.md` 校正 SOP
- [x] **micro-ROS 全鏈路實機驗證（未接線）**：`/tiny/base` 上線、
      `/tiny/wheel_ticks` 50.0 Hz、`/tiny/cmd_vel` 訂閱 match
- [x] namespace 隔離，與同機的 Tomcat 容器實測互不干擾（見 3.10）
- [x] `start-agent.sh` zombie 防呆 + `--init`（見 3.12）

### 待辦

- [x] 接線完成（使用者，2026-09-08），STBY 跳線接 3V3
- [x] `tools/encoder_check.py`：手轉輪子驗 encoder 的工具
- [ ] 🔴 **確認 encoder A/B 的高準位是 3.3V 不是 5V**（ESP32 非 5V 容忍，5V 會慢慢傷 GPIO）
- [x] **階段 A~C 完成（2026-09-08）**：四通道對位正確，方向校正完成
      `MOTOR_DIR = {-1,-1,+1,+1}`、`ENC_DIR = {-1,-1,-1,+1}`，複驗四輪皆正
- [x] `tools/wheel_sweep.py`：逐輪驅動 + encoder 判讀
- [x] **階段 D 完成**：`COUNTS_PER_WHEEL_REV` 實測 **574**（規格推算的 520 錯 10.4%）
- [x] **階段 E 完成**：滿檔 99.5~105.5 rad/s，死區 60/80/120/60，每輪前饋線已量到
- [x] **階段 F 完成**：三軸符號全對、誤差 <10%
- [x] `tools/`：`encoder_check` / `encoder_id` / `counts_per_rev` / `wheel_sweep`
      / `duty_sweep` / `cmd_vel_check`
- [ ] ⏳ **落地驗證 `+vy` 是否真的往左平移**（滾子方向，encoder 看不到）
- [x] **`ros2_ws/src/tiny_teleop/` 完成（2026-09-08）**：joy_node + joy_teleop + launch + params
      合成訊號實測通過（`axes[1]=0.6, buttons[9]=1` → `cmd_vel.x=0.15` → 四輪轉動）
- [x] 韌體加入**脫困補償**（`BREAKAWAY_DUTY`，雙向實測）——低速反轉原本四輪有兩顆不會動
- [x] **`face_node` 表情頁面（2026-09-09）**：`/tiny/joy` + `/tiny/cmd_vel` → 網頁機器人臉
      A/B/X/Y = 開心／生氣／疲倦／驚訝；行駛時眼睛看向行進方向，待機自動眨眼與張望
      （表情集抄 [FluxGarage/RoboEyes](https://github.com/FluxGarage/RoboEyes) 的定義，
      程式碼沒抄——那是 Adafruit GFX 的 Arduino C++，在瀏覽器裡用兩個圓角矩形短得多）
- [ ] 使用者實際用手把試駕（車仍架高）
- [ ] 落地測試 + 驗證 `+vy` 真的往左（滾子方向）
- [ ] `firmware/tiny_pid/` 閉環（收掉低速 ±25% 誤差與起步歪斜）
- [ ] 四輪方向校正（`MOTOR_DIR` / `ENC_DIR`）
- [ ] `COUNTS_PER_WHEEL_REV` 實測（規格推算 520，**必須用手轉法驗**）
      （⚠️ 這個常數同時縮放 PID 的量測值與 setpoint，**任何自洽性檢查都抓不到它錯**，
      只有手轉一圈數 tick、或地板上的捲尺看得見。Tomcat 那台的猜測值錯了 2.5 倍）
- [ ] `MAX_WHEEL_RAD_S` 與 `MIN_DUTY` 實測（目前是推算值）
- [ ] `firmware/tiny_pid/` 閉環韌體
- [ ] `ros2_ws/src/tiny_teleop/` 手把遙控節點（短期目標）
- [ ] `ros2_ws/src/tiny_base/` 里程計（`/tiny/wheel_ticks` → `/odom`）
