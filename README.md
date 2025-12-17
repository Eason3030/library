# 自行車輔助輪控制系統

## 專案概述
本專案為一套自行車輔助輪控制系統，整合紅外線遙控、霍爾感測器、速度感測與 MPU6050 姿態感測器，並搭配 OLED 顯示、RGB LED 狀態提示及音效反饋，透過多層決策邏輯提升自行車行駛時的穩定性與安全性。

---

## 系統功能

### 1. 輔助輪控制

#### 手動模式
- **紅外線遙控**：使用紅外線遙控器切換輔助輪 UP / DOWN 狀態。
- **霍爾感測器**：偵測端點與手動操作，避免機構過行程。
- **RGB LED 指示**：
  - DOWN：黃色  
  - UP：綠色
- **音效提示**：輔助輪放下與收起時播放不同提示音。

#### 自動模式（速度控制）
- 速度 **低於 10 km/h** 時，自動放下輔助輪。
- 速度 **高於或等於 10 km/h** 時，自動收起輔助輪。
- RGB LED 顯示：
  - 低速：黃色  
  - 高速：綠色

#### 傾斜警告保護
- 當傾斜角度 **超過 20° 且持續 0.5 秒**：
  - 自動放下輔助輪
  - RGB LED 顯示藍色
  - 播放警告音
- 當傾斜角度 **低於 12° 且持續 3 秒**：
  - 自動收起輔助輪
  - RGB LED 回復綠色

#### 緊急停止（最高優先）
- 當傾斜角度 **超過 35°** 或觸發安全保護：
  - 立即鎖定輔助輪為 DOWN
  - RGB LED 顯示紅色
  - 播放緊急警告音
- 當角度 **低於 12°** 並確認安全後，解除緊急狀態。

---

### 2. 感測與控制模組
- **MPU6050**：取得加速度與角速度資料，計算即時傾斜角度。
- **霍爾感測器**：
  - 輔助輪端點偵測
  - 手動操作狀態判斷
- **速度感測模組**：透過霍爾脈衝計算即時行駛速度。
- **紅外線接收器**：接收使用者遙控指令。
- **INA226 電流監測**：偵測系統電流狀態，用於安全保護。

---

### 3. 顯示與互動

#### OLED 顯示器
- 顯示內容包含：
  - 系統狀態
  - 輔助輪 UP / DOWN
  - 即時速度
  - 即時傾斜角度
- 提供啟動畫面與模組測試畫面。

#### RGB LED 狀態提示
- 以顏色快速顯示系統目前狀態，提升騎乘時辨識性。

#### 音效提示
- 系統啟動提示音
- 輔助輪動作提示音
- 傾斜警告音
- 緊急狀態警報音

---

### 4. 測試功能
- OLED 顯示測試
- 紅外線接收測試
- MPU6050 姿態讀值測試
- 霍爾感測器與速度感測測試
- RGB LED 顏色與音效確認  
- 測試結果會同步顯示於 OLED 並以音效回饋。

---

### 5. 系統設計重點
- 採用 **三層決策邏輯**：
  1. **安全層（最高優先）**
  2. **傾斜保護層**
  3. **操作控制層**
- 採用非阻塞式設計（以 `millis()` 為基礎）以提升即時反應能力。
- 所有高風險狀態皆可直接中斷低優先權控制邏輯。

---

🚦 RGB LED 狀態指示

| 顏色 | 狀態說明 |
|----|----|
| 🟢 綠色 | 正常行駛，速度 ≥ 10 km/h，輔助輪 UP |
| 🟡 黃色 | 輔助輪 DOWN（低速或手動） |
| 🔵 藍色 | 傾斜警告（角度 > 20°） |
| 🔴 紅色 | 緊急狀態（角度 > 35° 或安全保護） |
| 🟣 紫色 | 保留狀態（未使用） |

---

### 6. 未來優化方向
- 將音效與動畫全面改為非阻塞方式，進一步提升即時性。
- 增加資料紀錄功能，儲存速度與傾斜角度供後續分析。
- 優化 OLED 顯示配置，使資訊呈現更清楚直觀。

---

## 系統流程圖

```mermaid
graph TD
    Start([系統啟動]) --> Init[初始化設定]
    Init --> InitHW[硬體初始化<br/>- I2C, GPIO, 中斷<br/>- OLED 128x64<br/>- MPU6050, INA226]
    InitHW --> Calib[IMU 校正<br/>陀螺儀零點偏移]
    Calib --> StartAnim[開機動畫<br/>MEGA PRO v4.0]
    StartAnim --> StartSound[啟動音效]
    StartSound --> MainLoop{主迴圈}
    
    MainLoop --> ReadSensors[讀取感測器]
    ReadSensors --> ReadIMU[讀取 MPU6050<br/>計算傾斜角度]
    ReadIMU --> ReadCurrent[讀取電流感測器<br/>INA226]
    ReadCurrent --> ReadSpeed[計算速度<br/>霍爾中斷數據]
    
    ReadSpeed --> UpdateHistory[更新歷史數據<br/>速度/角度記錄]
    UpdateHistory --> CheckIR{紅外線<br/>接收?}
    
    CheckIR -->|Yes| HandleIR[處理紅外線指令<br/>切換輔助輪狀態<br/>進入手動模式]
    CheckIR -->|No| CheckHall
    HandleIR --> CheckHall
    
    CheckHall{霍爾限位<br/>觸發?}
    CheckHall -->|上限位| HallUp[鎖定輔助輪上升<br/>取消手動模式<br/>播放鎖定音效]
    CheckHall -->|下限位| HallDown[鎖定輔助輪下降<br/>取消手動模式<br/>播放鎖定音效]
    CheckHall -->|No| CheckEmergency
    HallUp --> CheckEmergency
    HallDown --> CheckEmergency
    
    CheckEmergency{緊急停止?}
    CheckEmergency -->|按鈕按下<br/>或電流過載| EmergencyStop[進入冷卻模式<br/>停止馬達/繼電器<br/>播放緊急警報]
    CheckEmergency -->|正常| CheckState
    EmergencyStop --> StateCheck
    
    CheckState{當前<br/>系統狀態?}
    
    StateCheck{系統狀態機}
    StateCheck -->|RUNNING| RunningState[運行模式]
    StateCheck -->|COOLING_DOWN| CoolingState[冷卻模式<br/>禁止所有動作<br/>RGB 紅燈]
    
    CoolingState --> CoolTimer{冷卻時間<br/>≥5秒?}
    CoolTimer -->|Yes| Recover[恢復運行<br/>重置手動/鎖定<br/>播放恢復音效]
    CoolTimer -->|No| KeepCool[保持冷卻]
    Recover --> TiltProtect
    KeepCool --> UpdateDisplay
    
    RunningState --> TiltProtect{傾斜保護檢查}
    
    TiltProtect -->|角度>35°| EmergencyTilt[緊急傾斜!<br/>進入冷卻模式<br/>緊急警報]
    TiltProtect -->|角度>20°| TiltWarn{持續>0.5秒?}
    TiltProtect -->|角度正常| TiltNormal
    
    EmergencyTilt --> UpdateDisplay
    
    TiltWarn -->|Yes| ActivateWarn[啟動傾斜警告<br/>強制輔助輪下降<br/>取消手動模式<br/>RGB 藍燈]
    TiltWarn -->|No| CheckSpeed
    
    ActivateWarn --> TiltRecoverCheck{角度<12°<br/>持續>3秒?}
    TiltRecoverCheck -->|Yes| TiltRecover[解除傾斜警告<br/>輔助輪上升<br/>播放恢復音效]
    TiltRecoverCheck -->|No| MaintainWarn[保持警告狀態]
    TiltRecover --> CheckSpeed
    MaintainWarn --> CheckSpeed
    
    TiltNormal --> CheckSpeed
    
    CheckSpeed{速度自動控制<br/>非手動/鎖定/警告}
    CheckSpeed -->|速度<10 km/h<br/>且輪子上升| AutoDown[自動下降輔助輪<br/>播放下降音效]
    CheckSpeed -->|速度≥10 km/h<br/>且輪子下降| AutoUp[自動上升輔助輪<br/>播放上升音效]
    CheckSpeed -->|維持現狀| KeepStatus[保持當前狀態]
    
    AutoDown --> ExecuteControl
    AutoUp --> ExecuteControl
    KeepStatus --> ExecuteControl
    
    ExecuteControl[執行控制邏輯]
    ExecuteControl --> PWMControl{PWM 週期控制}
    PWMControl -->|輔助輪下降| PWMOn[ON週期 1.8秒<br/>PWM 輸出<br/>根據電位器調速]
    PWMControl -->|輔助輪上升| PWMOff[OFF週期 1.8秒<br/>PWM 關閉]
    
    PWMOn --> RelayControl
    PWMOff --> RelayControl
    
    RelayControl[繼電器控制<br/>下降=ON / 上升=OFF]
    RelayControl --> RGBControl
    
    RGBControl{RGB 狀態燈}
    RGBControl -->|冷卻中| RGB_Red[紅燈]
    RGBControl -->|傾斜警告| RGB_Blue[藍燈]
    RGBControl -->|霍爾鎖定| RGB_Purple[紫燈]
    RGBControl -->|輔助輪下降| RGB_Yellow[黃燈]
    RGBControl -->|正常運行| RGB_Green[綠燈]
    
    RGB_Red --> CurrentAlarm
    RGB_Blue --> CurrentAlarm
    RGB_Purple --> CurrentAlarm
    RGB_Yellow --> CurrentAlarm
    RGB_Green --> CurrentAlarm
    
    CurrentAlarm{電流>1.5A?}
    CurrentAlarm -->|Yes| BuzzerAlarm[蜂鳴器警報<br/>1kHz 脈衝]
    CurrentAlarm -->|No| UpdateDisplay
    BuzzerAlarm --> UpdateDisplay
    
    UpdateDisplay[更新 OLED 顯示<br/>每100ms刷新]
    UpdateDisplay --> DisplayMode{顯示模式<br/>每5秒切換}
    
    DisplayMode -->|模式0| Dashboard[儀表板模式<br/>速度/角度大數字<br/>狀態條圖]
    DisplayMode -->|模式1| Detail[詳細數據模式<br/>所有參數列表]
    DisplayMode -->|模式2| Chart[歷史圖表模式<br/>速度/角度曲線]
    
    Dashboard --> SerialOutput
    Detail --> SerialOutput
    Chart --> SerialOutput
    
    SerialOutput[序列埠輸出<br/>每1秒除錯資訊]
    SerialOutput --> MainLoop
    
    style Start fill:#90EE90
    style EmergencyStop fill:#FF6B6B
    style EmergencyTilt fill:#FF6B6B
    style TiltWarn fill:#FFD93D
    style CoolingState fill:#FF6B6B
    style RGB_Red fill:#FF6B6B
    style RGB_Blue fill:#6BCF7F
    style RGB_Green fill:#90EE90
    style RGB_Yellow fill:#FFD93D
    style RGB_Purple fill:#C77DFF
    style MainLoop fill:#87CEEB