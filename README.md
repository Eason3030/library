
# 自行車輔助輪控制系統

## 專案概述
你的系統構想是一套完整的自行車輔助輪智能控制系統，核心目標是結合安全、可控與易於監測的功能。系統以三種主要感測資訊為偵測基礎：速度、傾斜角度與電流。速度感測可判斷車速，提供輔助輪自動收放或防跌倒的判斷依據；角度感測透過 MPU6050 或其他 IMU 取得車身傾斜資訊，用於偵測可能的傾倒風險；電流感測則可偵測線性制動器的負載狀態，防止馬達過載或異常耗電，保護硬體安全。

在控制端，你希望操作線性制動器能夠精準控制輔助輪的上升或下降。線性制動器以固定方向旋轉操作，確保機構簡單且可靠，並搭配 PWM 控制調節轉動速度，使系統能夠平滑升降，同時提供緊急斷電功能以應對意外狀況。使用者可以透過 IR 遙控器直接操作輔助輪上升或下降，例如家長在小孩快跌倒或需要完全升起輔助輪時，可即時介入。緊急停止按鈕則可隨時暫停整個系統或啟動重置流程，按下即生效，不需長按，確保快速應對意外。

顯示部分分為 OLED 與 RGB LED。OLED 顯示用於呈現精確資訊，包括即時速度、車身傾角與系統狀態，讓使用者或監護人能清楚掌握運行情況；RGB LED 則以不同顏色直觀顯示當前系統狀態，如自動控制、手動操作、安全警告或緊急停機，便於快速辨識。

軟體功能部分，I2C 通訊加入超時防護與自動重啟機制，搭配硬體看門狗，確保系統即使在通訊異常或感測器失效時，也能自我恢復，維持穩定運作。序列埠則用於除錯與診斷，輸出包括優先權、速度、角度、PWM 值、感測電流以及各種狀態訊息，方便開發與維護。整體系統的設計邏輯強調安全優先、即時控制與資訊透明，使自行車輔助輪在保護小孩安全的同時，也提供家長及使用者良好的操作體驗。
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
# 輔助輪控制系統 v10.1 - 完整流程圖

## 1. 系統整體狀態機

```mermaid
stateDiagram-v2
    [*] --> 上電啟動
    
    上電啟動 --> 檢查測試模式: 讀取按鈕
    
    檢查測試模式 --> INIT: 按鈕未按下
    檢查測試模式 --> TEST_MODE: 按鈕按住
    
    INIT --> 初始化流程
    
    初始化流程 --> IDLE: 初始化成功
    初始化流程 --> ERROR: 初始化失敗
    
    IDLE --> MOVING_UP: IR指令/自動觸發
    IDLE --> MOVING_DOWN: IR指令
    
    MOVING_UP --> IDLE: 到達上限位
    MOVING_UP --> ERROR: 超時10秒
    MOVING_UP --> MOVING_DOWN: IR中途切換
    
    MOVING_DOWN --> IDLE: 到達下限位
    MOVING_DOWN --> ERROR: 超時10秒
    MOVING_DOWN --> MOVING_UP: IR中途切換
    
    TEST_MODE --> MOVING_UP: IR指令
    TEST_MODE --> MOVING_DOWN: IR指令
    
    MOVING_UP --> TEST_MODE: 測試模式到達限位
    MOVING_DOWN --> TEST_MODE: 測試模式到達限位
    
    ERROR --> [*]: 需重啟系統
    
    note right of IDLE
        RGB: 青色
        繼電器: 關閉
        等待指令
    end note
    
    note right of MOVING_UP
        RGB: 綠色
        繼電器: 開啟
        監測上限位
    end note
    
    note right of MOVING_DOWN
        RGB: 紅色
        繼電器: 開啟
        監測下限位
    end note
    
    note right of ERROR
        RGB: 紅色閃爍
        繼電器: 關閉
        系統鎖定
    end note
```

## 2. 初始化流程詳細圖

```mermaid
flowchart TD
    Start([系統啟動]) --> CheckButton{檢查按鈕<br/>是否按住?}
    
    CheckButton -->|是| TestMode[進入測試模式<br/>跳過初始化]
    CheckButton -->|否| InitStart[INIT_START<br/>準備初始化]
    
    InitStart --> ReadSwitch[讀取磁簧狀態]
    ReadSwitch --> TurnOnRelay[開啟繼電器<br/>啟動馬達]
    TurnOnRelay --> StartTimer[啟動超時計時器<br/>10秒]
    StartTimer --> Searching[INIT_SEARCHING<br/>搜尋上限位]
    
    Searching --> CheckLower1{檢查下限位<br/>是否觸發?}
    CheckLower1 -->|是| AlreadyDown[已在下限位]
    AlreadyDown --> StopRelay1[關閉繼電器]
    StopRelay1 --> Complete1[初始化完成<br/>→ IDLE]
    
    CheckLower1 -->|否| CheckUpper{檢查上限位<br/>是否觸發?}
    CheckUpper -->|否| CheckTimeout1{是否超時?}
    CheckTimeout1 -->|是| Failed1[初始化失敗<br/>→ ERROR]
    CheckTimeout1 -->|否| Searching
    
    CheckUpper -->|是| UpperFound[找到上限位<br/>記錄事件]
    UpperFound --> MovingToLower[INIT_MOVING_TO_LOWER<br/>繼續移動至下限]
    
    MovingToLower --> CheckLower2{檢查下限位<br/>是否觸發?}
    CheckLower2 -->|是| ReachLower[到達下限位]
    ReachLower --> StopRelay2[關閉繼電器]
    StopRelay2 --> Delay[等待500ms<br/>穩定]
    Delay --> Complete2[初始化完成<br/>→ IDLE]
    
    CheckLower2 -->|否| CheckTimeout2{是否超時?}
    CheckTimeout2 -->|是| Failed2[初始化失敗<br/>→ ERROR]
    CheckTimeout2 -->|否| MovingToLower
    
    Complete1 --> PlaySound1[播放完成音效]
    Complete2 --> PlaySound2[播放完成音效]
    Failed1 --> PlayError1[播放錯誤音效]
    Failed2 --> PlayError2[播放錯誤音效]
    
    PlaySound1 --> End([進入待機狀態])
    PlaySound2 --> End
    PlayError1 --> ErrorState([進入錯誤狀態])
    PlayError2 --> ErrorState
    
    TestMode --> TestEnd([測試模式待機])
    
    style Start fill:#e1f5ff
    style End fill:#c8e6c9
    style ErrorState fill:#ffcdd2
    style TestEnd fill:#fff9c4
    style TurnOnRelay fill:#ffe0b2
    style StopRelay1 fill:#ffe0b2
    style StopRelay2 fill:#ffe0b2
```

## 3. IR 控制流程

```mermaid
flowchart TD
    IRStart([接收到IR信號]) --> DecodeIR[解碼IR數據]
    
    DecodeIR --> CheckNull{代碼是否<br/>為 0x0000?}
    CheckNull -->|是| IgnoreNull[忽略: 空碼]
    CheckNull -->|否| CheckRepeat{代碼是否<br/>為 0xFFFF?}
    
    CheckRepeat -->|是| IgnoreRepeat[忽略: 重複碼]
    CheckRepeat -->|否| CheckDebounce{距上次按下<br/>是否 < 800ms?}
    
    CheckDebounce -->|是| IgnoreDebounce[忽略: 防彈跳]
    CheckDebounce -->|否| ValidSignal[有效信號<br/>記錄時間]
    
    ValidSignal --> CheckState{檢查當前狀態}
    
    CheckState -->|INIT| ActionInit[提示音<br/>忽略指令]
    CheckState -->|ERROR| ActionError[錯誤音<br/>需重啟]
    CheckState -->|IDLE/TEST| CheckPosition{檢查磁簧位置}
    CheckState -->|MOVING_UP| ActionReverse1[停止上升<br/>切換下降]
    CheckState -->|MOVING_DOWN| ActionReverse2[停止下降<br/>切換上升]
    
    CheckPosition -->|在下限位| ActionUp[啟動上升]
    CheckPosition -->|在上限位| ActionDown[啟動下降]
    CheckPosition -->|在中間| ActionDefault[預設下降]
    CheckPosition -->|兩限位同時觸發| ActionStop[停止<br/>異常狀態]
    
    ActionUp --> ExecuteUp[執行上升動作]
    ActionDown --> ExecuteDown[執行下降動作]
    ActionDefault --> ExecuteDown
    ActionReverse1 --> ExecuteDown
    ActionReverse2 --> ExecuteUp
    
    ExecuteUp --> EndProcess([IR處理完成])
    ExecuteDown --> EndProcess
    ActionInit --> EndProcess
    ActionError --> EndProcess
    ActionStop --> EndProcess
    
    IgnoreNull --> EndProcess
    IgnoreRepeat --> EndProcess
    IgnoreDebounce --> EndProcess
    
    style ValidSignal fill:#c8e6c9
    style IgnoreNull fill:#ffcdd2
    style IgnoreRepeat fill:#ffcdd2
    style IgnoreDebounce fill:#ffcdd2
    style ExecuteUp fill:#a5d6a7
    style ExecuteDown fill:#ef9a9a
```

## 4. 上升動作流程

```mermaid
flowchart TD
    Start([啟動上升指令]) --> CheckState{檢查狀態<br/>是否為IDLE?}
    
    CheckState -->|否| Reject[拒絕執行<br/>非待機狀態]
    CheckState -->|是| LogStart[記錄日誌<br/>上升啟動]
    
    LogStart --> SetState[設定狀態<br/>→ MOVING_UP]
    SetState --> TurnOnRelay[開啟繼電器]
    TurnOnRelay --> StartTimer[啟動計時器]
    StartTimer --> PlaySound[播放上升音效]
    PlaySound --> LoopStart[進入上升循環]
    
    LoopStart --> CheckUpper{檢查上限位<br/>是否觸發?}
    
    CheckUpper -->|是| Success[到達上限位]
    Success --> TurnOffRelay1[關閉繼電器]
    TurnOffRelay1 --> StopTimer1[停止計時器]
    StopTimer1 --> SetIdle1[設定狀態<br/>→ IDLE/TEST]
    SetIdle1 --> PlayComplete1[播放完成音效]
    PlayComplete1 --> End1([上升完成])
    
    CheckUpper -->|否| CheckTimeout{計時器<br/>是否超時?}
    
    CheckTimeout -->|是| Timeout[超時錯誤]
    Timeout --> TurnOffRelay2[關閉繼電器]
    TurnOffRelay2 --> StopTimer2[停止計時器]
    StopTimer2 --> SetError[設定狀態<br/>→ ERROR]
    SetError --> PlayError[播放錯誤音效]
    PlayError --> End2([上升失敗])
    
    CheckTimeout -->|否| CheckIR{是否收到<br/>IR切換指令?}
    
    CheckIR -->|是| Cancel[取消上升]
    Cancel --> TurnOffRelay3[關閉繼電器]
    TurnOffRelay3 --> StopTimer3[停止計時器]
    StopTimer3 --> Delay[短暫延遲<br/>100ms]
    Delay --> SwitchDown[切換為下降]
    SwitchDown --> End3([切換完成])
    
    CheckIR -->|否| LoopStart
    
    Reject --> End4([指令拒絕])
    
    style TurnOnRelay fill:#a5d6a7
    style TurnOffRelay1 fill:#ef9a9a
    style TurnOffRelay2 fill:#ef9a9a
    style TurnOffRelay3 fill:#ef9a9a
    style Success fill:#c8e6c9
    style Timeout fill:#ffcdd2
    style End1 fill:#c8e6c9
    style End2 fill:#ffcdd2
```

## 5. 下降動作流程

```mermaid
flowchart TD
    Start([啟動下降指令]) --> CheckState{檢查狀態<br/>是否為IDLE?}
    
    CheckState -->|否| Reject[拒絕執行<br/>非待機狀態]
    CheckState -->|是| LogStart[記錄日誌<br/>下降啟動]
    
    LogStart --> SetState[設定狀態<br/>→ MOVING_DOWN]
    SetState --> TurnOnRelay[開啟繼電器]
    TurnOnRelay --> StartTimer[啟動計時器]
    StartTimer --> PlaySound[播放下降音效]
    PlaySound --> LoopStart[進入下降循環]
    
    LoopStart --> CheckLower{檢查下限位<br/>是否觸發?}
    
    CheckLower -->|是| Success[到達下限位]
    Success --> TurnOffRelay1[關閉繼電器]
    TurnOffRelay1 --> StopTimer1[停止計時器]
    StopTimer1 --> SetIdle1[設定狀態<br/>→ IDLE/TEST]
    SetIdle1 --> PlayComplete1[播放完成音效]
    PlayComplete1 --> End1([下降完成])
    
    CheckLower -->|否| CheckTimeout{計時器<br/>是否超時?}
    
    CheckTimeout -->|是| Timeout[超時錯誤]
    Timeout --> TurnOffRelay2[關閉繼電器]
    TurnOffRelay2 --> StopTimer2[停止計時器]
    StopTimer2 --> SetError[設定狀態<br/>→ ERROR]
    SetError --> PlayError[播放錯誤音效]
    PlayError --> End2([下降失敗])
    
    CheckTimeout -->|否| CheckIR{是否收到<br/>IR切換指令?}
    
    CheckIR -->|是| Cancel[取消下降]
    Cancel --> TurnOffRelay3[關閉繼電器]
    TurnOffRelay3 --> StopTimer3[停止計時器]
    StopTimer3 --> Delay[短暫延遲<br/>100ms]
    Delay --> SwitchUp[切換為上升]
    SwitchUp --> End3([切換完成])
    
    CheckIR -->|否| LoopStart
    
    Reject --> End4([指令拒絕])
    
    style TurnOnRelay fill:#a5d6a7
    style TurnOffRelay1 fill:#ef9a9a
    style TurnOffRelay2 fill:#ef9a9a
    style TurnOffRelay3 fill:#ef9a9a
    style Success fill:#c8e6c9
    style Timeout fill:#ffcdd2
    style End1 fill:#c8e6c9
    style End2 fill:#ffcdd2
```

## 6. 自動上升觸發流程

```mermaid
flowchart TD
    Start([主循環]) --> CheckState{當前狀態<br/>是否為IDLE?}
    
    CheckState -->|否| Skip1[跳過檢查]
    CheckState -->|是| CheckAuto{自動模式<br/>是否開啟?}
    
    CheckAuto -->|否| Skip2[跳過檢查]
    CheckAuto -->|是| CheckSpeed{速度是否<br/>≥ 15 km/h?}
    
    CheckSpeed -->|否| Skip3[跳過檢查]
    CheckSpeed -->|是| CheckPosition{是否在<br/>下限位?}
    
    CheckPosition -->|否| Skip4[跳過檢查]
    CheckPosition -->|是| Trigger[觸發自動上升]
    
    Trigger --> LogSpeed[記錄當前速度]
    LogSpeed --> StartUp[執行上升動作]
    StartUp --> End([自動上升啟動])
    
    Skip1 --> Continue([繼續主循環])
    Skip2 --> Continue
    Skip3 --> Continue
    Skip4 --> Continue
    
    style Trigger fill:#c8e6c9
    style StartUp fill:#a5d6a7
    style End fill:#c8e6c9
```

## 7. 按鈕控制流程

```mermaid
flowchart TD
    Start([讀取按鈕]) --> ReadState[讀取當前狀態]
    
    ReadState --> CheckPress{按下瞬間?}
    
    CheckPress -->|是| RecordStart[記錄按下時間<br/>重置長按標記]
    CheckPress -->|否| CheckHold{持續按住?}
    
    CheckHold -->|是| CheckDuration{按住時間<br/>≥ 2 秒?}
    CheckDuration -->|是且未處理| LongPress[長按觸發]
    CheckDuration -->|否| WaitMore[繼續等待]
    CheckDuration -->|已處理| WaitRelease[等待放開]
    
    LongPress --> ToggleAuto[切換自動上升模式]
    ToggleAuto --> CheckAutoState{自動模式<br/>新狀態?}
    
    CheckAutoState -->|開啟| PlayHigh[播放高音]
    CheckAutoState -->|關閉| PlayLow[播放低音]
    
    PlayHigh --> LogAuto1[記錄: 自動模式開啟]
    PlayLow --> LogAuto2[記錄: 自動模式關閉]
    
    LogAuto1 --> MarkHandled1[標記已處理]
    LogAuto2 --> MarkHandled2[標記已處理]
    
    CheckHold -->|否| CheckRelease{放開瞬間?}
    
    CheckRelease -->|否| End1([按鈕處理完成])
    CheckRelease -->|是| CalcDuration[計算按下時長]
    
    CalcDuration --> CheckShort{時長 < 2 秒?}
    
    CheckShort -->|是| ShortPress[短按觸發]
    CheckShort -->|否| LongRelease[長按放開]
    
    ShortPress --> SwitchPage[切換OLED頁面]
    SwitchPage --> PlayBeep[播放短音]
    PlayBeep --> LogPage[記錄頁面切換]
    
    LogPage --> End2([按鈕處理完成])
    LongRelease --> End3([按鈕處理完成])
    RecordStart --> End4([按鈕處理完成])
    WaitMore --> End5([按鈕處理完成])
    WaitRelease --> End6([按鈕處理完成])
    MarkHandled1 --> End7([按鈕處理完成])
    MarkHandled2 --> End8([按鈕處理完成])
    
    style LongPress fill:#fff9c4
    style ShortPress fill:#c8e6c9
    style ToggleAuto fill:#ffe0b2
```

## 8. 主循環結構

```mermaid
flowchart TD
    Start([Loop 開始]) --> ReadSensors[讀取所有感測器]
    
    ReadSensors --> UpdateSwitches[更新磁簧開關狀態]
    UpdateSwitches --> UpdateSpeed[更新速度測量]
    UpdateSpeed --> HandleInput[處理輸入]
    
    HandleInput --> HandleIR[處理 IR 指令]
    HandleIR --> HandleButton[處理按鈕]
    HandleButton --> CheckErrors[檢查異常狀態]
    
    CheckErrors --> CheckBothSwitches{兩磁簧<br/>同時觸發?}
    
    CheckBothSwitches -->|是| EmergencyStop[緊急停止<br/>→ ERROR]
    CheckBothSwitches -->|否| StateMachine[狀態機處理]
    
    StateMachine --> CheckCurrentState{檢查當前狀態}
    
    CheckCurrentState -->|INIT| ProcessInit[執行初始化流程]
    CheckCurrentState -->|IDLE| CheckAutoUp[檢查自動上升條件]
    CheckCurrentState -->|TEST_MODE| CheckAutoUp
    CheckCurrentState -->|MOVING_DOWN| ProcessDown[處理下降動作]
    CheckCurrentState -->|MOVING_UP| ProcessUp[處理上升動作]
    CheckCurrentState -->|ERROR| DoNothing[無動作<br/>等待重啟]
    
    ProcessInit --> UpdateOutputs[更新輸出]
    CheckAutoUp --> UpdateOutputs
    ProcessDown --> UpdateOutputs
    ProcessUp --> UpdateOutputs
    DoNothing --> UpdateOutputs
    EmergencyStop --> UpdateOutputs
    
    UpdateOutputs --> UpdateRGB[更新 RGB LED]
    UpdateRGB --> UpdateDisplay[更新 OLED 顯示]
    UpdateDisplay --> PrintStatus[輸出狀態到串口]
    
    PrintStatus --> LoopEnd([Loop 結束])
    LoopEnd --> Start
    
    style Start fill:#e1f5ff
    style EmergencyStop fill:#ffcdd2
    style UpdateOutputs fill:#c8e6c9
    style LoopEnd fill:#e1f5ff
```

## 9. 錯誤處理流程

```mermaid
flowchart TD
    Start([偵測到錯誤]) --> CheckType{錯誤類型?}
    
    CheckType -->|兩磁簧同時觸發| Error1[BOTH_SWITCHES_ERROR]
    CheckType -->|上升超時| Error2[TIMEOUT_ERROR]
    CheckType -->|下降超時| Error3[TIMEOUT_ERROR]
    CheckType -->|初始化超時| Error4[INIT_TIMEOUT_ERROR]
    
    Error1 --> CommonActions[共同處理]
    Error2 --> CommonActions
    Error3 --> CommonActions
    Error4 --> CommonActions
    
    CommonActions --> StopRelay[關閉繼電器]
    StopRelay --> StopTimer[停止計時器]
    StopTimer --> SetErrorState[設定狀態<br/>→ ERROR_STATE]
    SetErrorState --> RecordType[記錄錯誤類型]
    RecordType --> PlayError[播放錯誤音效]
    
    PlayError --> LogError[輸出錯誤日誌]
    LogError --> LockSystem[鎖定系統]
    
    LockSystem --> ErrorLoop[錯誤循環]
    ErrorLoop --> FlashRed[紅色LED閃爍]
    FlashRed --> ShowError[OLED顯示錯誤]
    ShowError --> WaitReset{等待重啟}
    
    WaitReset -->|IR按下| IgnoreIR[忽略IR<br/>播放錯誤音]
    WaitReset -->|按鈕按下| IgnoreButton[忽略按鈕]
    WaitReset -->|系統重啟| SystemReset[系統重新啟動]
    
    IgnoreIR --> ErrorLoop
    IgnoreButton --> ErrorLoop
    SystemReset --> End([錯誤處理結束])
    
    style Error1 fill:#ffcdd2
    style Error2 fill:#ffcdd2
    style Error3 fill:#ffcdd2
    style Error4 fill:#ffcdd2
    style LockSystem fill:#f44336
    style SystemReset fill:#c8e6c9
```

## 10. 速度測量流程

```mermaid
flowchart TD
    Start([霍爾感測器中斷]) --> ReadTime[讀取當前時間<br/>micros]
    
    ReadTime --> CalcInterval[計算脈衝間隔<br/>= 當前時間 - 上次時間]
    
    CalcInterval --> CheckMin{間隔是否<br/>≥ 40ms?}
    
    CheckMin -->|否| IgnorePulse[忽略脈衝<br/>防止雜訊]
    CheckMin -->|是| ValidPulse[有效脈衝]
    
    ValidPulse --> SaveInterval[儲存脈衝間隔]
    SaveInterval --> SaveTime[更新上次觸發時間]
    SaveTime --> SetFlag[設定新數據標記]
    
    SetFlag --> EndInterrupt([中斷處理完成])
    IgnorePulse --> EndInterrupt
    
    EndInterrupt -.-> MainLoop([主循環處理])
    
    MainLoop --> CheckFlag{是否有<br/>新數據?}
    
    CheckFlag -->|否| CheckTimeout{是否超時<br/>> 2 秒?}
    CheckFlag -->|是| CalcSpeed[計算速度]
    
    CalcSpeed --> Formula[速度 = 周長 / 時間 × 3.6]
    Formula --> Validate{速度是否<br/>合理?<br/>0~50 km/h}
    
    Validate -->|是| UpdateSpeed[更新當前速度]
    Validate -->|否| DiscardSpeed[捨棄異常數據]
    
    UpdateSpeed --> ClearFlag[清除新數據標記]
    ClearFlag --> MarkTime[標記更新時間]
    
    CheckTimeout -->|是| ZeroSpeed[速度歸零]
    CheckTimeout -->|否| KeepSpeed[保持當前速度]
    
    MarkTime --> EndMain([速度處理完成])
    DiscardSpeed --> EndMain
    ZeroSpeed --> EndMain
    KeepSpeed --> EndMain
    
    style ValidPulse fill:#c8e6c9
    style IgnorePulse fill:#ffcdd2
    style UpdateSpeed fill:#c8e6c9
    style DiscardSpeed fill:#ffcdd2
```

---

## 圖表說明

### 顏色標示：
- 🟦 **藍色**：啟動/開始節點
- 🟩 **綠色**：成功/正常執行
- 🟥 **紅色**：錯誤/失敗/停止
- 🟨 **黃色**：警告/特殊狀態
- 🟧 **橘色**：繼電器操作

### 主要流程：
1. **狀態機**：系統整體運作邏輯
2. **初始化**：自動校正找基準位置
3. **IR控制**：三層過濾機制
4. **上升/下降**：包含超時保護
5. **自動上升**：速度觸發條件
6. **按鈕**：短按/長按不同功能
7. **主循環**：每次循環的完整流程
8. **錯誤處理**：異常情況處理
9. **速度測量**：霍爾感測器中斷處理

### 關鍵保護機制：
- ✅ 10秒超時保護
- ✅ 磁簧防彈跳 50ms
- ✅ IR防彈跳 800ms
- ✅ 兩磁簧同時觸發偵測
- ✅ 速度合理性檢查
- ✅ 錯誤狀態鎖定
