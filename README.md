
# 自行車輔助輪控制系統

## 專案概述
本專案為一套自行車輔助輪控制系統，整合紅外線遙控、磁簧開關（霍爾感測器）、速度感測與 MPU6050 姿態感測器，並搭配 OLED 顯示、RGB LED 狀態提示及音效反饋。系統採用多層決策邏輯，以提升自行車行駛時的穩定性與安全性，並提供自動與手動控制模式。

---

## 系統功能

### 1. 輔助輪控制

#### 手動模式
- **紅外線遙控**：使用紅外線遙控器切換輔助輪 UP / DOWN 狀態。  
- **磁簧開關**：偵測輔助輪上下限位，避免機構過行程。  
- **RGB LED 指示**：
  - DOWN：黃色  
  - UP：綠色  
- **音效提示**：輔助輪放下與收起時播放不同提示音。

#### 自動模式（速度控制）
- 速度 < 10 km/h → 自動放下輔助輪（磁簧確認下限）。  
- 速度 ≥ 10 km/h → 自動收起輔助輪（磁簧確認上限）。  
- **RGB LED 顯示**：
  - 低速：黃色  
  - 高速：綠色  

#### 傾斜警告保護
- 當傾斜角度 > 20° 且持續 0.5 秒：
  - 自動放下輔助輪（磁簧確認下限）  
  - RGB LED 顯示藍色  
  - 播放警告音  
- 當傾斜角度 < 12° 且持續 3 秒：
  - 自動收起輔助輪（磁簧確認上限）  
  - RGB LED 回復綠色  

#### 緊急停止（最高優先）
- 傾斜角度 > 35° 或電流過載 → 立即鎖定輔助輪下降（磁簧確認下限）、紅燈警報、播放緊急音效。  
- 當角度 < 12° 並確認安全後，解除緊急狀態。

---

### 2. 感測與控制模組
- **MPU6050**：取得加速度與角速度資料，計算即時傾斜角度。  
- **磁簧開關**：
  - 偵測輔助輪端點位置  
  - 判斷手動操作是否超過上下限  
- **速度感測模組**：透過霍爾脈衝計算即時行駛速度。  
- **紅外線接收器**：接收使用者遙控指令。  
- **INA226 電流監測**：偵測系統電流狀態，用於安全保護。

---

### 3. 顯示與互動
- **OLED 顯示器**：
  - 顯示系統狀態、輔助輪 UP/DOWN、即時速度、傾斜角度  
  - 提供啟動畫面與測試畫面  
- **RGB LED 狀態提示**：以顏色快速顯示系統目前狀態  
- **音效提示**：
  - 系統啟動提示音  
  - 輔助輪動作提示音  
  - 傾斜警告音  
  - 緊急警報音  

---

### 4. 測試功能
- OLED 顯示測試  
- 紅外線接收測試  
- MPU6050 姿態讀值測試  
- 磁簧開關與速度感測測試  
- RGB LED 顏色與音效確認  
- 測試結果同步顯示於 OLED 並以音效回饋

---

### 5. 系統設計重點
- 採用 **三層決策邏輯**：
  1. 安全層（最高優先）  
  2. 傾斜保護層  
  3. 操作控制層  
- 採用非阻塞式設計（以 `millis()` 為基礎），提升即時反應能力  
- 高風險狀態可直接中斷低優先權控制邏輯  

---

### 6. RGB LED 狀態指示
| 顏色 | 狀態說明 |
|------|-----------|
| 🟢 綠色 | 正常行駛，速度 ≥ 10 km/h，輔助輪 UP |
| 🟡 黃色 | 輔助輪 DOWN（低速或手動） |
| 🔵 藍色 | 傾斜警告（角度 > 20°） |
| 🔴 紅色 | 緊急狀態（角度 > 35° 或安全保護） |
| 🟣 紫色 | 保留狀態（未使用） |

---

### 7. 系統流程圖
```mermaid
graph TD
    Start([系統啟動]) --> Init[初始化設定<br/>輔助輪下降]
    Init --> InitHW[硬體初始化<br/>I2C, GPIO, OLED, MPU6050, INA226, 磁簧開關]
    InitHW --> Calib[IMU 校正<br/>陀螺儀零點偏移]
    Calib --> StartAnim[開機動畫]
    StartAnim --> StartSound[啟動音效]
    StartSound --> MainLoop{主迴圈}

    MainLoop --> ReadSensors[讀取感測器<br/>IMU, 電流, 速度, 磁簧開關]
    ReadSensors --> UpdateHistory[更新歷史數據<br/>速度/角度]
    UpdateHistory --> CheckIR{紅外線接收?}
    CheckIR -->|Yes| HandleIR[處理紅外線<br/>手動模式]
    CheckIR -->|No| CheckLimit

    CheckLimit{磁簧開關觸發?}
    CheckLimit -->|上限位| LimitUp[鎖定輔助輪上升]
    CheckLimit -->|下限位| LimitDown[鎖定輔助輪下降]
    CheckLimit -->|無觸發| CheckEmergency
    LimitUp --> CheckEmergency
    LimitDown --> CheckEmergency

    CheckEmergency{緊急停止?}
    CheckEmergency -->|是| EmergencyStop[冷卻模式<br/>停止馬達<br/>紅燈警報]
    CheckEmergency -->|否| CheckState

    CheckState{系統狀態}
    CheckState -->|RUNNING| RunningState[運行模式]
    CheckState -->|COOLING_DOWN| CoolingState[冷卻模式]

    RunningState --> TiltProtect{傾斜保護}
    TiltProtect -->|>35°| EmergencyTilt[緊急傾斜<br/>紅燈警報]
    TiltProtect -->|20~35°| TiltWarn{>0.5秒?}
    TiltProtect -->|<20°| TiltNormal

    TiltWarn -->|是| ActivateWarn[傾斜警告<br/>輔助輪下降<br/>藍燈]
    TiltWarn -->|否| CheckSpeed
    ActivateWarn --> TiltRecoverCheck{<12°? 持續3秒}
    TiltRecoverCheck -->|是| TiltRecover[解除警告<br/>輔助輪上升<br/>綠燈]
    TiltRecoverCheck -->|否| MaintainWarn[保持警告]
    TiltRecover --> CheckSpeed
    MaintainWarn --> CheckSpeed
    TiltNormal --> CheckSpeed

    CheckSpeed{速度控制<br/>非手動/警告}
    CheckSpeed -->|<10 km/h 且未到下限| AutoDown[自動下降輔助輪<br/>黃燈]
    CheckSpeed -->|≥10 km/h 且未到上限| AutoUp[自動上升輔助輪<br/>綠燈]
    CheckSpeed -->|保持現狀| KeepStatus

    AutoDown --> ExecuteControl
    AutoUp --> ExecuteControl
    KeepStatus --> ExecuteControl

    ExecuteControl[執行 PWM 控制<br/>上下移動<br/>檢查磁簧開關]
    ExecuteControl --> UpdateDisplay[更新 OLED<br/>顯示狀態/速度/角度]
    UpdateDisplay --> MainLoop