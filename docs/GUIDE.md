# 操作指南 — tiny_platform_mac

從「什麼都沒有」到「用手把開著跑」，以及日常怎麼用、壞了怎麼查。

---

## 1. 日常使用（只要三個指令）

```bash
cd ~/Desktop/esp/tiny_platform_mac

make up        # 起容器 + agent + 遙控，全部照正確順序，約 40 秒
make watch     # 想看按鍵／指令／輪速時（Ctrl-C 離開，不影響車子）
make down      # 收工
```

`make up` 跑完會印出每一層的狀態。**全綠就直接推搖桿**，不用按任何鍵（DIRECT 模式）。

| 動作 | 操作 |
| --- | --- |
| 前進／後退 | 左搖桿 上／下 |
| 左／右平移 | 左搖桿 左／右 |
| 逆時針／順時針 | 右搖桿 左／右 |
| Turbo（0.5 m/s） | 按住 **R** |

> ⚠️ **沒有 deadman。** 停車只剩三道：手把斷線 1 秒、韌體 500 ms 看門狗、電源開關。
> 搖桿卡住或你走開，車子會一直跑。**落地跑的時候手邊要有斷電開關。**
> 想改回「按住 L 才會動」：`config/teleop_params.yaml` 的 `enable_button` 從 `-1` 改回 `9`，
> 然後 `make rebuild-ws`（見第 4 節）。

### `make up` 為什麼要照順序

三層的順序不能顛倒，而且**顛倒後的症狀三種都長得一模一樣**（「什麼都不動」、`ros2 node list` 空的）：

```
1. 容器
2. micro-ROS agent   ← 必須在 ESP32 開機「之前」就位
3. 遙控               ← 必須在確認遙測有在流之後才起
```

第 2 層是關鍵：韌體的 `error_loop()` **沒有重試機制**。ESP32 開機時 agent 不在，它就
永久卡在閃燈迴圈，事後再開 agent 也沒用。`start-agent.sh` 的做法是先用 CH340 的 RTS
把板子按在重置狀態，等 agent 進入監聽後才放開 —— 所以**永遠不要跑裸的 `micro_ros_agent`**。

---

## 2. 從零開始（新機器，或重灌之後）

```bash
cd ~/Desktop/esp/tiny_platform_mac

# 一次性：把 udev 規則裝到「主機」（不是容器裡，原因見 CLAUDE.md 3.5）
make install-udev        # 產生 /dev/esp32、/dev/input/pro_controller

# 一次性：建映像（首次幾十分鐘，會下載 esp32 core 約 1~2 GB）
make build

# 一次性：手把藍牙配對（主機端做，容器幫不上忙）
bluetoothctl
  scan on            # 按住手把的同步鍵（充電孔旁的小圓鈕）直到出現 Pro Controller
  pair <MAC>
  trust <MAC>        # trust = 之後按 Home 自動重連，不用再配對
  connect <MAC>
  exit

# 燒韌體
make flash SKETCH=tiny_open

# 開起來
make up
```

### 檢查點

每一步做完可以先確認再往下：

| 步驟 | 怎麼確認 |
| --- | --- |
| udev | `ls -l /dev/esp32` 要看到 symlink 指向 `ttyUSB0` |
| 映像 | `docker images \| grep tiny-platform` |
| 藍牙 | `bluetoothctl info <MAC>` 要有 `Connected: yes` |
| 燒錄 | 訊息出現 `Hash of data verified` |
| 整體 | `make status` 全綠 |

---

## 3. 目前的機台常數（都是實測值，不是規格書抄的）

改硬體的話這些要重新量，程序見 [`BRINGUP.md`](BRINGUP.md)。

| 常數 | 值 | 怎麼來的 |
| --- | --- | --- |
| `MOTOR_DIR` | `{-1,-1,+1,+1}` | 逐輪驅動 + 人眼看轉向 |
| `ENC_DIR` | `{-1,-1,-1,+1}` | 同上 |
| `COUNTS_PER_WHEEL_REV` | **574** | 手轉一圈 ×4（規格推算的 520 錯了 10.4%） |
| 滿檔轉速 | 99.5~105.5 rad/s | duty 階梯掃描 |
| 死區（正／反） | 60/80、80/100、120/60、60/100 | 雙向掃描 |
| `FF_OFFSET` | `{22.1, 30.3, 34.9, 22.2}` | 線性擬合 |
| `FF_GAIN` | `{9.69, 9.62, 9.76, 9.50}` | 線性擬合 |
| `MAX_WHEEL_RAD_S` | 25.0（= 1.0 m/s 上限） | 由滿檔轉速降額 |

---

## 4. 改東西之後要做什麼

| 你改了 | 要跑 |
| --- | --- |
| `config/teleop_params.yaml`（搖桿設定、速度） | `make rebuild-ws` 然後 `make down && make up` |
| `ros2_ws/src/` 底下的 Python | 同上（`--symlink-install`，多數情況重啟即可） |
| `firmware/*/config.h` 或 `.ino` | `make flash SKETCH=tiny_open` 然後 `make down && make up` |
| `Dockerfile` | `make build` 然後 `make down && make up` |
| `scripts/*.rules` | `make install-udev` |

---

## 5. 壞掉的時候

**第一步永遠是 `make status`。** 它會告訴你是哪一層斷的。

| status 顯示 | 意思 | 怎麼辦 |
| --- | --- | --- |
| `FAIL container` | 容器沒跑 | `make up` |
| `FAIL micro-ROS agent` | agent 掛了 | `make up`（會自動重啟該層；**不要**手動再跑一個 agent，見下方） |
| `FAIL no /tiny/wheel_ticks` | 板子沒開機、或 **RTS 把板子壓在重置** | 先 `make up` 再一次（會自動重放 RTS）；還不行看下面的 LED 對照 |
| `FAIL no /tiny/joy` | 手把沒連上，**或連上但 joy_node 抓著舊裝置** | 先 `make up` 再一次（會自動重啟）；還不行才按手把 Home 鍵重連 |
| `WARN no /tiny/cmd_vel` | 遙控節點沒在發 | `make logs` 看原因 |

### 🚫 「什麼都不動」時最容易犯的錯

**不要重跑 `start-agent.sh`。** 兩個 agent 會各讀一半的 XRCE 資料流，誰都握不住連線 ——
**症狀跟「完全沒有 agent」一模一樣**，於是很容易越修越糟。`make up` 會偵測既有的 agent
而不會重複啟動。真的要重來就 `make down && make up`。

### LED（GPIO2）是韌體唯一的狀態通道

micro-ROS 佔著 serial 埠，所以韌體**不能** `Serial.print`。板上那顆 LED 就是全部的資訊：

| LED | 意思 |
| --- | --- |
| 快閃（5 Hz） | 韌體在跑，但找不到 agent → 順序錯了，`make down && make up` |
| 慢閃（1 Hz） | agent 已連上、沒有指令 → 正常待機 |
| 恆亮 | 正在收指令、正在驅動 |
| **完全不亮** | **板子沒開機** → 見 `CLAUDE.md` 3.1 的 bootloop 定位流程 |

### 板子「好好的卻沒有遙測」（燒完韌體後最常見）

症狀：`make status` 只有 `FAIL no /tiny/wheel_ticks`，agent 行程健康，LED 也不是不亮。

**原因**：CH340 的 RTS 接在 ESP32 的 EN(reset) 腳。agent 一開 serial 埠核心就拉高 RTS，
把板子按在重置狀態；正常會由 `release_rts.py` 放開，但如果它跑得比 agent 開埠還早，
就沒人放開了 —— **板子永遠開不了機**，而畫面上完全看不出來。

**解法：`make up` 再跑一次。** 它在等遙測時會每 6 秒重放一次 RTS（畫面上的 `R`）。

手動救援：

```bash
docker exec tiny-platform python3 /root/scripts/release_rts.py /dev/esp32 115200
```

確認板子本身是好的（先停 agent 再跑）：純聽 serial 應該看到**一次** `rst:0x1` 加一堆
`XRCE` —— 那代表韌體在跑、只是在等 agent。

### 手把「連著卻沒反應」

症狀：`make status` 顯示 `FAIL no /tiny/joy`，但 `bluetoothctl` 說 `Connected: yes`、
`joy_node` 行程也活著。

**原因是時序**：手把待機斷線後自動重連，核心重建了裝置節點，而 SDL 在 joy_node 啟動時
就列舉完了裝置，之後只會對著舊節點重試（log 會一直印 `Couldn't read device info`）。

**解法：`make up` 再跑一次就好。** 它會偵測到「行程在但 topic 沒資料」並自動重啟該層。

> 這是一整類問題：agent 可能是 zombie、joy_node 可能抓著失效的裝置控制代碼、
> 兩個 agent 可能互搶資料流。三者都是「行程存在但沒在工作」，
> 所以 `make up` / `make status` **一律以 topic 有沒有資料為準，不看行程表**。

### 按鍵／搖桿沒反應

```bash
make watch        # 看按了什麼、送出什麼、輪子怎麼動
```

- 搖桿數字會跑、`cmd_vel` 有值，但輪子不動 → 問題在韌體或電源（查 12V、STBY）
- 搖桿數字不動、`joy msgs` 也不增加 → 資料沒進到 joy_node，往下查：
  ```bash
  docker exec -it tiny-platform evtest /dev/input/event10
  ```
  這個有反應而 `make watch` 沒有 → SDL/joy_node 層的問題
  這個也沒反應 → 藍牙其實沒在傳（`Connected: yes` 也可能是假的），重新配對

---

## 6. 校正工具（改硬體之後才需要）

全部在 `tools/`，都在容器內跑（`make shell` 之後 `cd $TP_PROJ`）。

| 工具 | 用途 |
| --- | --- |
| `encoder_check.py` | 手轉輪子，看四個 encoder 活著沒 |
| `encoder_id.py` | 用「轉不同次數」判斷哪個接頭對哪顆輪子（不靠記憶） |
| `counts_per_rev.py` | 量 `COUNTS_PER_WHEEL_REV`（**改輪子或馬達必做**） |
| `wheel_sweep.py` | 逐輪驅動，校正 `MOTOR_DIR` / `ENC_DIR` |
| `duty_sweep.py` | duty 階梯掃描 → 死區、滿檔轉速、前饋係數（`--sign -1` 量反轉） |
| `cmd_vel_check.py` | 三軸驗證，比對「要求的輪速」與「實際輪速」 |
| `joy_watch.py` | 手把即時儀表板（= `make watch`） |
| `teleop_monitor.py` | 記錄遙控期間的指令與輪速 |

完整校正程序見 [`BRINGUP.md`](BRINGUP.md)，階段 A0 → F。

---

## 7. 還沒做的事

1. **落地測試** —— 特別要確認 `+vy` 真的讓車往**左**平移。這取決於麥克納姆滾子從上方看
   有沒有排成 X，是機構事實，**encoder 看不到**。裝反的話輪子會互相打架、磨損很快。
2. **閉環韌體 `firmware/tiny_pid/`** —— 目前是開環，低速（<0.15 m/s）誤差可到 25%，
   起步會稍微歪。前饋係數都量好了，直接可用。
3. **里程計** —— `/tiny/wheel_ticks` → `/odom`。
