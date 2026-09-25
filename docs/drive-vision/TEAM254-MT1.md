# Team 254 MT1 定位移植與編譯前確認

本輪將單顆 `limelight-rear` 的場地定位，由「MT2 X/Y＋MT1 航向」改成 **Team 254 的 MT1 優先＋單 Tag 歷史航向備援**。原始碼與測試案例已備妥；Java 編譯、JUnit、模擬與部署均尚未執行。

參考固定版本：[Team254/FRC-2025-Public `ae1aa582`](https://github.com/Team254/FRC-2025-Public/blob/ae1aa582b1cadb8462e6cb90718880d11a8f42b1/src/main/java/com/team254/frc2025/subsystems/vision/VisionSubsystem.java)。移植其每台相機的選擇規則與數學計算，使用本機單相機 IO；沒有新增不存在的第二台相機，也沒有雙相機加權平均。

## 執行流程

![MT1 與歷史航向備援](diagrams/05-megatag-inputs.png)

[SVG](diagrams/05-megatag-inputs.svg) · [Mermaid](diagrams/05-megatag-inputs.mmd)

1. `VisionIOHardwareLimelight` 只讀 `botpose_wpiblue`（MT1）、`stddevs`、三維姿態與標籤明細。每輪清除舊欄位再更新。
2. `VisionSubsystem` 先檢查資料格式、時間戳與平移標準差。
3. `processMegatagPoseEstimate()` 執行 254 的主分支篩選，通過就保留 MT1 的 X/Y/yaw。
4. `fuseWithGyro()` 同時產生單 Tag 備援候選。使用影像拍攝當下的歷史航向，把 MT1 推算的 robot-to-tag 向量轉回場地，再由已知 Tag 位置反推機器人 X/Y。
5. 主分支優先；主分支失敗才用備援。每個時間戳最多送出一次 `RobotState.updateMegatagEstimate()`。
6. 既有 callback 轉交 `DriveSubsystem → DriveIOHardware → CTRE.addVisionMeasurement()`。這輪沒有改動底盤接收端；FPGA 秒到 CTRE 時基仍只在 Drive IO 轉換。

「一張 Tag」不代表一定用備援。單 Tag 的 MT1 若通過主分支，仍優先使用 MT1 的位置與航向。兩張以上 Tag 不走 gyro 備援。

## 與目前版本相比，會改到哪些流程

| 流程 | 修改前 | 本輪原始碼 |
|---|---|---|
| 相機定位輸入 | MT1 與 MT2 都必須存在 | 只需要 MT1；不讀取 MT2 |
| 對相機輸出 | 每輪傳底盤 yaw/yaw rate，啟動設定外部 IMU 模式 | 移除這兩項 MT2 專用輸出；保留安裝姿態及追蹤 priorityid=21 |
| 定位姿態 | MT2 的 X/Y 配 MT1 yaw | 主分支直接用 MT1 X/Y/yaw；備援用歷史 yaw 重算 X/Y |
| 單 Tag | 無 MT1-only 備援 | MT1 優先，失敗可選歷史航向備援 |
| 多 Tag | 仍要求兩演算法資料齊全 | 通過主分支即接受，不走備援 |
| 平移權重 | 依 MT2 距離平方與 Tag 數 | 主分支取 max(MT1 σx,σy)/quality；備援直接取 max(σx,σy) |
| 航向權重 | 固定 σyaw=10 rad | 主分支取 MT1 的 yaw 標準差/quality；備援設為 1e6，幾乎不更新 yaw |
| 篩選條件 | MT1/MT2 時差、場界、距離、所有量測的角速度檢查 | 移除雙通道時差／MT2 距離與場界檢查；採下表 254 主分支與備援門檻 |
| 模擬 | 同時寫 MT1/MT2 topic | 只寫 MT1；保留原始拍攝時間、正確 raw Tag 清單及相對追蹤資料 |
| 記錄 | 混合量測與雙通道記錄 | 記錄 MEGATAG／GYRO／NONE，以及各候選拒絕原因 |

使用 MT1 yaw 標準差會改變定位器修正航向的強度；它也可能間接影響使用融合姿態的場地座標駕駛、heading hold 或自動對位回授。相關控制命令的 PID、速度上限與按鍵綁定未修改。

## 與 254 相同的主分支門檻

| 檢查 | 條件 |
|---|---|
| 單 Tag ambiguity | ≤0.19 |
| 單 Tag 平均面積 | ≥1.0% |
| 單 Tag yaw | 面積 <2.0% 時，與歷史 yaw 最短角度差 ≤5° |
| MT1 距場地原點 | 平移向量長度 ≥1 m；不是 Tag 距離或前次定位誤差 |
| MT1 高度 | abs(Z) ≤0.2 m |
| exclusiveTag | 若已設定，本次 Tag ID 清單至少包含指定 ID |
| 歷史姿態 | `getFieldToRobot(timestamp)` 不為空 |
| quality | 多 Tag 為 1；單 Tag 為 1−ambiguity，用於縮放標準差 |

多 Tag 跳過單 Tag 的 ambiguity／面積／yaw 額外檢查；仍檢查原點距離、Z、exclusiveTag 與歷史姿態。主分支沒有高速旋轉拒絕門檻，與 254 一致。

## 與 254 相同的備援選擇與計算

只有一張 Tag 才可產生備援候選，要求歷史姿態存在、Tag 位於目前 Reef 場地圖，以及拍攝前 0.3 秒的角速度檢查通過（5 rad/s）。

```text
robotToTag = 已知場地 Tag 姿態 relativeTo MT1 機器人姿態
posteriorXY = 已知 Tag XY − rotate(robotToTag XY, 歷史 yaw)
posteriorYaw = 歷史 yaw
stddevs = [max(σx,σy), max(σx,σy), 1e6]
```

這個歷史 yaw 是底盤定位器輸出的姿態角度，可包含先前視覺融合，不是直接讀原始 IMU yaw。

**備援不重做主分支的 ambiguity、面積、yaw 差、原點距離、Z 門檻或 exclusiveTag 檢查。** 因此單 Tag 可能在這些主分支檢查失敗後仍被備援接受；這是刻意依照 254 的控制流移植。資料型別／有限值等共同保護仍適用。

## 本機保留的差異與修正

這是 254 核心演算法的單相機移植，不是將整個 254 專案逐字覆蓋：

- 相機仍是一顆 `limelight-rear`，在底盤中心正上方 0.54 m，朝車頭，pitch/yaw/roll=0。沿用 `com.team11855`、2026 依賴與目前 2025 Reefscape 場地圖。
- 保留 □／△ 對 21 號的相對位置觀測。`tv` 用於局部追蹤；場地定位看 MT1 的 tagCount，所以只看到其他 Tag 時仍可定位。這與 254 用 `tv` 作為每台相機入口條件不同。
- 保留資料年齡 ≤0.50 s、未來容許 0.05 s；拒絕不合法／重複時間戳。
- 增加有效 ID／Tag 數一致性、空陣列、非有限姿態與標準差保護。MT1 x/y 標準差須為正且有限；主分支的 yaw 標準差也需有效。未知或零標準差不會直接當成最高可信度。
- 保留已修正的 `MegatagPoseEstimate`：Helper 延遲毫秒轉秒、空 raw fiducial 過濾，以及 quality 的正確 Struct 大小／schema。
- 254 的角速度 helper 可能回傳帶負號的最大幅度值；此處比較前取 `abs`，正反向高速旋轉都會拒絕備援。`RobotState` 本身未改動。
- 主分支沿用 254 直接使用 `stddevs[5]` 的寫法，没有自行另加角度單位轉換；這些數值仍需與實際 Limelight 韌體輸出核對。
- 保留本機 Photon 模擬的真實拍攝時間；不退回 254 模擬批次寫入時重新計時的方式。

既有 `RobotState` 歷史 buffer 對超出範圍的查詢會返回端點，因此「查到非空姿態」不等於嚴格證明時間在緩衝區間內。本輪沒有重設這個歷史資料結構。

## 原始碼與測試檔案

下列主程式檔案位於 `src/main/java/com/team11855/frc2026/`：

| 檔案 | 變更 |
|---|---|
| `Constants.java` | 加入 254 門檻，移除 MT2／混合定位專用常數 |
| `subsystems/vision/VisionIO.java` | 移除 MT2 狀態與朝向輸出介面；保留單相機及局部追蹤 |
| `subsystems/vision/VisionIOHardwareLimelight.java` | 只讀 MT1；移除 MT2 讀取、朝向輸出與 IMU 模式設定 |
| `subsystems/vision/VisionSubsystem.java` | MT1 主分支、歷史航向備援、單次回送、候選與拒絕原因記錄 |
| `subsystems/vision/VisionIOSimPhoton.java` | 只產生 MT1 定位 topic；保留相對追蹤 |
| `src/test/.../vision/MegatagVisionTest.java` | 改為 MT1 主分支、備援幾何、門檻、正反向角速度、單次回送與 MT1-only IO 案例 |
| `src/test/.../vision/SingleCameraVisionTest.java` | 更新 MT1 權重與 fixture，保留安裝／失去目標／相對追蹤獨立性案例 |
| `docs/drive-vision/`、根目錄 README | 更新目前流程、確認清單與第 3／5 張圖 |

`Robot`、`RobotContainer`、`RobotState`、Drive IO、Tuner、CAN、PS5 綁定、追蹤命令、vendor 版本與 Helper 沒有程式變更。沒有 commit 或 push。

## 驗證狀態與等待確認的範圍

已執行的檢查只限原始碼閱讀、格式整理、文件圖表產生、`git diff --check` 與 `python3 docs/drive-vision/static_check.py`。這些不編譯或執行機器人程式。JUnit 案例已更新但尚未執行，不宣稱已通過。

依使用者提供的規則：「程式下進去編譯前要跟我完整確認會改到哪些流程才可以下」，待確認上述變更後，才執行：

```sh
./gradlew spotlessCheck test build -PteamNumber=11855
```

此命令會檢查格式、編譯 Java、執行測試、建立產物，並可能下載依賴、產生 AutoLogged 與 BuildConstants。若全專案格式檢查遇到本輪未改的舊檔，先報告問題，不自動格式化整份專案。本次待確認範圍不含 `deploy`、桌面模擬、Git commit 或 GitHub push。
