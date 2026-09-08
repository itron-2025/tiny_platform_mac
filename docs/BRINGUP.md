# 上電與校正 SOP — tiny_platform_mac

依序做，**不要跳步**。每一步都是為了讓下一步的失敗只有一個可能的原因。

前置條件：`docs/WIRING.md` 第 8 節的通電前檢查全部通過，**車子架高、輪子離地**。

> ⚠️ **本車的 STBY 是跳線直接接 3V3**，所以驅動器從上電那一刻就致能。
> 開機約 300 ms 內 FR 與 RL 可能抽動一下（原因見 `WIRING.md` 5.3）。
> 校正期間車子架高就完全無所謂；落地後每次上電會有一個小竄動。

---

## 0. 名詞：什麼叫「輪子正轉」

**正轉 = 如果它是一般輪子，會把車子往前推的方向。**
也就是從側面看，輪子最上緣往**車頭**方向移動。

麥克納姆輪四顆的滾子角度不同，但「正轉」的定義對四顆都一樣。

---

## 1. 起容器與 agent

```bash
# 主機
cd ~/Desktop/esp/tiny_platform_mac
make run                     # 起容器（互動 zsh）

# 另開一個終端機
make agent                   # 起 micro-ROS agent（會自動放開 CH340 RTS 讓板子開機）
```

> 🚫 **不要跑裸的 `ros2 run micro_ros_agent`。** 除了 RTS 會把板子壓在 reset，
> 更關鍵的是啟動時序：韌體的 `error_loop()` 沒有退路，agent 沒先就位的話 ESP32 會
> **永久**卡在閃燈迴圈。詳見 `CLAUDE.md` 第 3.2 節。

### 確認連上了

```bash
make attach                  # 進容器
ros2 node list               # 應看到 /tiny/base
ros2 topic list              # 應看到 /tiny/cmd_vel /tiny/wheel_duty /tiny/wheel_actual_vel /tiny/wheel_ticks
ros2 topic hz /tiny/wheel_ticks   # 應約 50 Hz
```

> ⏱ **`node list` / `topic list`需要約 10 秒才會穩定**，剛起 agent 就查會是空的。
> 這不代表沒連上——**`ros2 topic hz /tiny/wheel_ticks` 是即時可信的檢查**，
> 它直接訂閱不靠 ROS graph。空清單先等 10 秒再查一次，別急著重跑 agent。
>
> 🔒 **為什麼所有 topic 都在 `/tiny/` 底下**：Tomcat 底盤的容器可能同時在這台主機上跑，
> 它用的是裸 `/cmd_vel`。沒有 namespace 的話，Tomcat 的手把會直接驅動這台車。
> （`ROS_DOMAIN_ID` 在這裡無效，原因見 `CLAUDE.md` 3.10。）

**看不到 `/tiny/base`** → 照 `CLAUDE.md` 第 7 節的診斷順序做，不要亂試。

### LED 對照

| LED | 意思 |
| --- | --- |
| 快閃（5 Hz） | 韌體在跑，但**找不到 agent**（或某個 `rcl_*` 失敗）→ 順序錯了 |
| 慢閃（1 Hz） | agent 已連上，**沒有命令**（看門狗已停車）→ 正常待機 |
| 恆亮 | agent 已連上，**正在收命令**→ 正在驅動 |
| **完全不亮** | **板子沒開機** → 走 `CLAUDE.md` 3.1 的 bootloop 定位流程 |

---

## 1.5 階段 A0：手轉輪子驗 encoder（**先做這個，馬達完全不動**）

這一步不需要馬達轉動，是最安全也最划算的一步——**一次回答四個問題**。

```bash
cd $TP_PROJ && python3 tools/encoder_check.py
```

然後**用手**依序慢慢轉每一顆輪子，看哪一欄在動：

```
FL*    +512 ( +0.98 rev) | FR       +0 ( +0.00 rev) | RL   +0 (...) | RR   +0 (...)
  ^ 星號 = 這一刻正在變動
```

| 問題 | 怎麼看 |
| --- | --- |
| **encoder 都活著嗎？** | 四欄都要能被轉動。永遠不動的那欄 → 沒接、沒供電、或缺上拉電阻 |
| **哪個 encoder 對到哪顆輪子？** | 轉前左輪，動的應該是 `FL` 欄。動到別欄就是接頭插錯位置 |
| **計數方向對嗎？** | 把輪子往「前進」方向滾，數字應該**增加**。減少 → `ENC_DIR[該輪] = -1` |
| **`COUNTS_PER_WHEEL_REV` 是多少？** | 在輪子做記號，**精確轉一整圈**，讀 `rev` 欄應接近 `1.00`（= 520 counts ±5%） |

> 讀到約 **0.5 rev**（260 counts）→ 掉了一相，查 B 相接線／上拉。
> 讀到約 **2.0 rev**（1040 counts）→ 減速比不是 10:1。
> 實際值填回 `firmware/tiny_open/config.h` 的 `COUNTS_PER_WHEEL_REV`。

四輪都通過再往下走。**這一步不過就不要通馬達電**——方向校正需要能相信 encoder 的讀數。

---

## 2. 階段 A：逐輪確認「哪一個接頭對到哪一顆輪子」

燒 `firmware/tiny_open`（`make flash SKETCH=tiny_open`，燒完要重跑 `make agent`）。

一次只推一顆輪子。`/tiny/wheel_duty` 是**繞過運動學的原始 duty**，
陣列順序固定 `[FL, FR, RL, RR]`：

```bash
# FL（前左）。持續發，因為看門狗 500 ms 會停車
ros2 topic pub --rate 20 /tiny/wheel_duty std_msgs/msg/Float32MultiArray "{data: [250, 0, 0, 0]}"
```

**看：真的是前左那顆在轉嗎？**

- ✅ 是 → 進行下一顆。
- ❌ 轉的是別顆 → 馬達接頭插錯位置。**把實際對應關係記下來告訴我**
  （例如「送 FL 的時候是後右在轉」），我會把 `config.h` 的 `MOT_*` 與 `ENC_*`
  四個陣列**一起**重排。⚠️ 只重排其中一組會讓 encoder 對到別顆馬達，比原本更難查。

四顆依序：

```bash
ros2 topic pub --rate 20 /tiny/wheel_duty std_msgs/msg/Float32MultiArray "{data: [250, 0, 0, 0]}"   # FL
ros2 topic pub --rate 20 /tiny/wheel_duty std_msgs/msg/Float32MultiArray "{data: [0, 250, 0, 0]}"   # FR
ros2 topic pub --rate 20 /tiny/wheel_duty std_msgs/msg/Float32MultiArray "{data: [0, 0, 250, 0]}"   # RL
ros2 topic pub --rate 20 /tiny/wheel_duty std_msgs/msg/Float32MultiArray "{data: [0, 0, 0, 250]}"   # RR
```

> **完全不動**：先把 250 加到 320（上限是 `DUTY_LIMIT` = 0.35 × 1023 ≈ 358）。
> 還是不動 → 量該通道的 TB6612 輸出電壓、檢查 STBY 是否真的被拉高（GPIO4 對 GND 應為 3.3V）。

---

## 3. 階段 B：馬達轉向（`MOTOR_DIR`）

沿用階段 A 的指令，這次看**方向**：輪子最上緣是不是往車頭跑？

| 結果 | 動作 |
| --- | --- |
| 往車頭（正轉） | ✅ `MOTOR_DIR[該輪]` 維持 `+1` |
| 往車尾（反轉） | ❌ 把 `config.h` 的 `MOTOR_DIR[該輪]` 改成 `-1` |

四顆都記錄完再一次改 `config.h`、重燒，不要改一顆燒一次。

> 馬達的兩條粗線接反只是轉向相反，**用 `MOTOR_DIR` 修就好，不必重焊**。

---

## 4. 階段 C：encoder 方向（`ENC_DIR`）

一邊推該輪正轉，一邊開另一個終端機看 tick：

```bash
ros2 topic echo /tiny/wheel_ticks
```

| 結果 | 動作 |
| --- | --- |
| 該輪的 tick **增加** | ✅ `ENC_DIR[該輪]` 維持 `+1` |
| 該輪的 tick **減少** | ❌ 改成 `-1` |
| 該輪的 tick **完全不動** | 🔴 encoder 沒讀到 —— 見下方 |

**tick 不動的排查順序：**

1. 量該輪 encoder 的 `VCC` 對 GND 是否有 3.3V。
2. 手轉輪子，用三用電表量 A 相對 GND 有沒有在 0V / 3.3V 之間跳。
3. 沒跳 → **極可能是缺上拉電阻**（`docs/WIRING.md` 5.2）。
   GPIO34/35/36/39 沒有內部上拉，open-drain 的 encoder 沒外接上拉就永遠讀 0。

改完 `MOTOR_DIR` 與 `ENC_DIR` → 重燒 → 重跑階段 A~C 確認四輪全部「正命令 → 正轉 → tick 增加」。

---

## 4.5 ✅ 階段 A~C 實測結果（2026-09-08）

用 `tools/wheel_sweep.py` 逐輪驅動 duty +180，人眼確認轉向：

| 輪 | 通道對位 | +duty 時滾的方向 | encoder 讀數 | → `MOTOR_DIR` | → `ENC_DIR` |
| --- | --- | --- | --- | --- | --- |
| FL | ✅ 正確 | 後（反了） | + | **−1** | **−1** |
| FR | ✅ 正確 | 後（反了） | + | **−1** | **−1** |
| RL | ✅ 正確 | 前 | **−** | +1 | **−1** |
| RR | ✅ 正確 | 前 | + | +1 | +1 |

修正後複驗：四輪皆 `+`（FL +4411 / FR +4464 / RL +4004 / RR +5054）。

**四個馬達接頭都插在正確的角落**，沒有交叉。

> ⚠️ `MOTOR_DIR` 與 `ENC_DIR` **不是兩個獨立的旋鈕**。翻 `MOTOR_DIR` 會讓輪子反轉，
> encoder 也跟著反轉，所以 `ENC_DIR` 必須一起翻。之後若哪顆又不對，
> **重新觀察一次再同時推導兩個值**，不要單獨去試其中一個——那很容易繞進死胡同。

### 順帶量到的：四輪速度個體差異

同 duty（180）、同時間窗，兩趟獨立測試的排名一致：

```
        第一趟(2s)   第二趟(3s)      相對
  RR      3294        5054         最快
  FR      2738        4464
  FL      3016        4411
  RL      2453        4004         最慢  (約比 RR 慢 21%)
```

這是真實的馬達個體差異，不是雜訊。**開環直走會畫弧就是這個原因**，
也是閉環韌體需要每輪獨立前饋（`FF_OFFSET` / `FF_GAIN`）的理由。

---

## 5. 階段 D：`COUNTS_PER_WHEEL_REV` 實測（**不要略過**）

規格推算是 `13 PPR × 4 × 10 = 520`，但**必須實測**。
理由見 `config.h` 的註解：這個常數同時縮放量測值與 setpoint，
**錯了的話迴圈依然完美跟隨目標**，里程計也用同一個倍率少報，兩個錯誤互相抵消——
任何自洽性檢查都抓不到。Tomcat 那台的猜測值錯了 2.5 倍，撐過了一整輪 bench 測試沒被發現。

### 5.1 手轉法（最直接，先做這個）

> 階段 A0 已經用 `tools/encoder_check.py` 做過這件事。這裡保留純 ROS 指令版，
> 給不想用工具、或想自己核對數字的時候用。

agent 在跑、**不下任何命令**（此時馬達是 coast，輪子可以自由轉動）：

```bash
# 記下起始值
ros2 topic echo /tiny/wheel_ticks --once
```

在輪子上做一個記號，**用手把它慢慢轉整整一圈**（記號回到原位），然後：

```bash
ros2 topic echo /tiny/wheel_ticks --once
```

`後 − 前` 應該等於 **520 ± 5%**（也就是 494 ~ 546）。

- 差很多 → 把實測值填進 `config.h` 的 `COUNTS_PER_WHEEL_REV`。
- 四輪都要驗（順便再確認一次 encoder 都活著）。
- 讀到約一半（260）→ 掉了一相，回去查 B 相接線／上拉。
- 讀到約兩倍（1040）→ 減速比不是 10:1。

### 5.2 捲尺法（落地後做，驗證的是不同的東西）

手轉法驗的是「一圈 = 幾個 count」。捲尺法驗的是
「count × 輪半徑 = 真實距離」，會一併抓到**輪徑量錯**與**打滑**。
兩個都要做，缺一不可。落地、直線走一段量到的距離，跟里程計比對。

---

## 6. 階段 E：量 `MAX_WHEEL_RAD_S`

到目前為止 `config.h` 的 `MAX_WHEEL_RAD_S = 100.0` 是**推算值不是量測值**
（MG513 系列 30:1 常見標示 330 rpm，外推到 10:1 約 990 rpm ≈ 104 rad/s）。
開環的 duty 映射完全依賴這個數字，所以要量。

⚠️ **車子必須架高、輪子離地。** 這一步會讓輪子全速轉。

1. 暫時把 `config.h` 的 `DUTY_LIMIT_FRAC` 從 `0.35f` 改成 `1.0f`，重燒。
2. 四輪一起全速：
   ```bash
   ros2 topic pub --rate 20 /tiny/wheel_duty std_msgs/msg/Float32MultiArray "{data: [1023, 1023, 1023, 1023]}"
   ```
3. 另一個終端機讀穩態值：
   ```bash
   ros2 topic echo /tiny/wheel_actual_vel
   ```
4. 取**四輪之中最小**的那個穩態值，乘 0.8 當作 `MAX_WHEEL_RAD_S`。
   取最小值是因為運動學要求四輪能同時達到同一個速度；取 0.8 是留給電池電壓下降與負載的餘裕。
5. `DUTY_LIMIT_FRAC` 改回一個你安心的值（建議先回 `0.5f`），重燒。

---

## 7. 階段 F：`/tiny/cmd_vel` 三軸驗證

四輪方向都對了，才驗運動學。**車子仍然架高。**

```bash
# 前進
ros2 topic pub --rate 20 /tiny/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"
# 左平移
ros2 topic pub --rate 20 /tiny/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.2}, angular: {z: 0.0}}"
# 逆時針自轉
ros2 topic pub --rate 20 /tiny/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}"
```

期望的四輪轉向（`+` = 正轉）：

| 命令 | FL | FR | RL | RR |
| --- | --- | --- | --- | --- |
| `+vx` 前進 | + | + | + | + |
| `+vy` 左平移 | **−** | + | + | **−** |
| `+wz` 逆時針 | **−** | + | **−** | + |

用 `ros2 topic echo /tiny/wheel_actual_vel` 對照正負號。

- 前進時有輪子是負的 → 回階段 B，`MOTOR_DIR` 沒調好。
- 前進全對、但左平移的正負號是 `+ − − +`（整組相反）→ **滾子方向裝反了**
  （麥克納姆輪從上方看滾子應該排成 "X"）。此時是機構問題，
  要嘛把左右輪對調安裝，要嘛在 `inverse_kinematics()` 把 `vy` 項的正負號整組翻過來。
  ⚠️ 翻程式碼之前先確認機構——裝反的麥克納姆輪在地面上會互相打架、磨損很快。

### 落地前最後一件事

三軸都對了，才把車放到地上，**先用最小的速度**試。手邊備好斷電開關。

---

## 7.5 ✅ 階段 D / E / F 實測結果（2026-09-08）

### 階段 D — `COUNTS_PER_WHEEL_REV`（手轉一圈 × 4）

```
FL 563    FR 581    RL 572    RR 581       平均 574.25
```

**規格推算的 520 錯了 +10.4%，已被排除。** 即使算進手轉對記號的 ±2.8% 誤差，
範圍也只到 558~590。實際減速比很可能是 **11:1** 而不是型號寫的 10:1
（13 PPR × 4 × 11 = 572，正落在量測範圍中央）。
`config.h` 採用量測值 **574**。

> 這正是本專案一直警告的陷阱。若沒做這一步，PID 會完美跟隨一個錯 10% 的目標、
> 里程計少報 10%，兩個錯誤互相抵消，**每一項自我檢查都會通過**。

### 階段 E — duty 階梯掃描（`tools/duty_sweep.py`，車架高）

| 輪 | 死區 (duty) | 滿檔轉速 | 擬合線 |
| --- | --- | --- | --- |
| FL | 60 | 99.51 rad/s | `duty = 21.6 + 9.91ω` |
| FR | 80 | 102.54 | `duty = 25.7 + 9.67ω` |
| RL | **120** | 100.16 | `duty = 37.8 + 9.89ω` |
| RR | 60 | 105.48 | `duty = 22.9 + 9.39ω` |

- 最高轉速差距 6%，**死區差距 2 倍**（RL 的靜摩擦明顯偏大）。
- `FF_GAIN` 9.39~9.91，與 Tomcat 那台的 8.91~9.64 幾乎同區間（同型馬達的交叉驗證）。
- 滿檔 99.5 rad/s = **3.98 m/s（14 km/h）**，對室內小車太快，
  `MAX_WHEEL_RAD_S` 設 **25.0**（= 1.0 m/s）當上限。

> ⚠️ **這一趟同時修掉一個設計問題**：原本 `MAX_WHEEL_RAD_S` 身兼二職——
> 既是「最高允許速度」又是「滿檔對應的速度」。把它降額成 0.8× 當安全上限，
> duty 映射就會默默多出 25% 誤差。現在拆成兩個常數，降額才真的只是降額。

### 階段 F — `/tiny/cmd_vel` 三軸（`tools/cmd_vel_check.py`）

| 命令 | FL | FR | RL | RR |
| --- | --- | --- | --- | --- |
| `+vx` 前進 | +2.5% | +1.3% | +8.4% | +4.3% |
| `+vy` 左平移 | −1.9% | +3.1% | +9.6% | +1.7% |
| `+wz` 逆時針 | −3.5% | +4.6% | −8.1% | +4.2% |

**符號全對，誤差全在 10% 以內。** 開環的每輪前饋映射有效。

> ⏳ **仍未驗證、且 encoder 看不到的一件事**：`+vy` 是不是真的讓車體往**左**平移。
> 那取決於麥克納姆滾子從上方看有沒有排成 "X"，是機構事實，只能用眼睛或落地實測。
> 落地第一次就要確認這件事——滾子裝反的話輪子會互相打架、磨損很快。

---

## 8. 完成後

階段 A~F 全過 → 把量到的數字（`MOTOR_DIR`、`ENC_DIR`、`COUNTS_PER_WHEEL_REV`、
`MAX_WHEEL_RAD_S`、`MIN_DUTY`）都寫回 `config.h`，然後才進階段 2：
`firmware/tiny_pid` 閉環韌體與手把遙控。

---

## 9. 症狀對照表

| 症狀 | 最可能的原因 | 先查 |
| --- | --- | --- |
| `ros2 node list` 空的 | agent 順序錯，或有兩個 agent | `CLAUDE.md` 第 7 節 |
| LED 完全不亮 | 板子沒開機（bootloop） | `CLAUDE.md` 3.1；確認 GPIO12 沒接東西 |
| 四輪全都不動 | STBY 沒拉高、或沒共地 | 量 GPIO4 = 3.3V？三處 GND 導通？ |
| 一輪不動、其他正常 | 該通道接線或 TB6612 該半邊 | 交換兩顆馬達的接頭，看故障跟著誰跑 |
| 某輪 tick 永遠 0 | encoder 缺上拉電阻 | `WIRING.md` 5.2 |
| 某輪只能單向轉 | 該束線接觸不良 | **拔掉重插**。Tomcat 遇過「單向轉」與 bootloop 同時出現，是同一束線鬆脫 |
| 怪症狀成雙成對出現 | 機構/接線，不是兩個軟體 bug | 先查線再查程式 |
