# MT1／MT2 讀取與編譯前確認

目前 `offseason` 已改成參考專案的 Helper 方式：**MT2 提供 X、Y，MT1 提供朝向，組成一筆量測後交給底盤融合**。這是原始碼修改完成的狀態，尚未由本輪工作編譯、執行測試或部署。先前固定追蹤 21 號的未提交修改也保留。

## 參照的實際程式

採用你連結中的 [VisionIOLimelightHelper.java](https://github.com/PepsiLee/2026CompetitionRobot-Offseason-main/blob/cbbfe56336a8d2b683d77063c47d82a935fd275d/src/main/java/frc/robot/subsystems/vision/VisionIOLimelightHelper.java)，以及 [Vision.java](https://github.com/PepsiLee/2026CompetitionRobot-Offseason-main/blob/cbbfe56336a8d2b683d77063c47d82a935fd275d/src/main/java/frc/robot/subsystems/vision/Vision.java) 的距離權重。參考版本固定在 `cbbfe56336a8d2b683d77063c47d82a935fd275d`。同目錄的 `VisionIOLimelight.java` 只訂閱 MT2；本輪使用的是同時讀兩組資料的 Helper 方式。

保留自己的 `VisionIO → VisionSubsystem → RobotState → DriveIOHardware` 介面、`com.team11855` 套件與 library 版本。參考專案的 2026 場地、其他機構、相機安裝參數及 CAN 設定沒有帶入。

## 資料接線

![MT1 與 MT2 接線](diagrams/05-megatag-inputs.png)

[SVG](diagrams/05-megatag-inputs.svg) · [Mermaid](diagrams/05-megatag-inputs.mmd)

1. `RobotState` 提供底盤在**藍方原點座標系**的朝向。實機角速度來自 Pigeon，模擬來自輪組回報的角速度。`VisionSubsystem.periodic()` 每輪先呼叫 `io.setRobotOrientation()`，再讀相機；角度以度、角速度以度／秒送出。
2. 啟動時設定 `imumode_set=0`，MT2 使用外部底盤朝向，不依賴相機內建 IMU。每輪用 `SetRobotOrientation()` 發送 `robot_orientation_set`。MT2 的朝向需求及藍方原點慣例依 [Limelight 官方說明](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)。
3. `getBotPoseEstimate_wpiBlue()` 讀 `botpose_wpiblue`（MT1）；`getBotPoseEstimate_wpiBlue_MegaTag2()` 讀 `botpose_orb_wpiblue`（MT2）。兩份原始 pose、tagCount、時間戳分開保存。
4. `VisionSubsystem` 檢查兩份量測，再組合 `Pose2d(MT2.translation, MT1.rotation)`。只回送一筆，採用 MT2 的拍攝時間與 tagCount。
5. 時間戳是「NT 更新時間 − 相機延遲」，全程保持 FPGA 秒。既有 `DriveIOHardware.addVisionMeasurement()` 才呼叫 `Utils.fpgaToCurrentTime()`，只轉換一次。沒有照抄參考 raw adapter 中提早轉 CTRE 的時間處理。
6. `RobotState` callback 把量測交給 CTRE 底盤估測器，融合輪組里程計、陀螺儀及視覺。

MT2 算出的位置依賴送入的底盤朝向；啟動／Options 重設的方向必須符合真實場地方向。這不是看見標籤就自動重新校正所有陀螺儀設定。Limelight 也必須使用和程式相符的 AprilTag 地圖；本專案目前仍是 2025 Reefscape。

## 前後行為

| 流程 | 修改前 | 修改後 |
|---|---|---|
| 場地定位讀取 | 只實際讀取 MT1；MT2 欄位未填入 | 每輪讀 MT1 與 MT2，先發送底盤朝向 |
| 估計選擇 | MT1 通過舊品質門檻就用；部分單標籤情況嘗試 gyro 備援 | 兩組皆有效時，用 MT2 X/Y + MT1 朝向；沒有 MT1 單獨備援 |
| 資料失效 | 部分欄位可能保留舊值 | 每輪清空後更新兩通道；殘留 NT 值仍用原時間判斷過期 |
| 篩選 | 面積、ambiguity、Z、原點距離、歷史朝向等舊門檻 | 下表的新時間、場界、距離、角速度檢查；保留 exclusiveTag 限制 |
| 融合權重 | Limelight stddevs 加品質倍率 | 與參考專案相同，平移依距離與 tagCount 設定；朝向標準差 10 rad，讓陀螺儀主導朝向 |
| □／△ | 相對座標追蹤 21 號 | 操作與演算法保持；不依賴 MT1／MT2 場地量測是否被接受 |
| 模擬 | MT2 topic 與 MT1 完全相同、平均距離為 0 | 兩通道共用真實拍攝時間；距離與 raw tags 對應解算目標；模擬 MT2 朝向取底盤歷史 |

Photon 只用來驗證資料流與控制邏輯；MT2 模擬通道使用 Photon 位置加底盤朝向，**不是 Limelight 專有 MT2 演算法的重現**，不能用它證明實際定位精度。

| 新篩選項目 | 條件／預設 |
|---|---|
| 雙通道 | MT1、MT2 都存在且 tagCount > 0；缺任何一組就不更新場地定位 |
| 數值 | 拍攝時間有限且 > 0、延遲有限且 ≥ 0、pose 無 NaN／Infinity |
| 資料年齡 | 兩通道各自 ≤ 0.50 秒；向未來超前最多 0.05 秒 |
| 通道時間差 | MT1 與 MT2 相差最多 0.05 秒；這是容許差，沒有假定 NT 兩個 topic 能原子同步 |
| 重複影格 | MT1、MT2 的時間都必須晚於各自上次接受值；同一量測不重複融合 |
| 場地 | MT2 X/Y 需在場地長寬邊界外 0.5 公尺以內；不再拒絕原點附近合法位置 |
| 距離 | MT2 平均 tag 距離 > 0 且 ≤ 6 公尺 |
| 旋轉 | 現在與拍攝附近歷史的絕對角速度 ≤ 5 rad/s（約 286°/s），正反方向都檢查 |
| exclusiveTag | 若其他程式明確設定，兩通道的 ID 清單都必須包含指定 ID；按鍵指定 21 並不設定此篩選 |
| 平移標準差 | 多標籤 `0.20 + 0.05 × 距離²`；單標籤 `0.50 + 0.12 × 距離²`，單位公尺 |

時間／距離／場界／角速度門檻在 [Constants.java](../../src/main/java/com/team11855/frc2026/Constants.java) 的 `VisionConstants`。距離權重是 `VisionSubsystem` 的公式；這些是起始設定，尚未用實機記錄調校。MT1 朝向被低權重融合，並非完全不參與定位。

## 鏡頭及按鍵

仍是單顆 **limelight-rear**，朝車頭、位在底盤中心正上方 **0.54 m**，pitch/yaw/roll = 0°。安裝位置仍在 `Constants.VisionConstants` 的五個 `kCamera...` 欄位修改。

□ 按一次只轉向 21；△ 按住轉向並維持底盤中心至 21 號的水平距離 1 m。它們使用 `tv / tid / targetpose_cameraspace`，所以缺少 MT2 不會單獨阻擋按鍵。看不到有效 21 就停止，不盲目搜尋。沒有把場地定位標籤過濾成只有 21。

先前「搖桿可開車但按鍵沒反應」仍缺即時紀錄，**本次不能宣稱已排除**。定位排查請看 `Vision/RejectionReason`、`Vision/AcceptedThisCycle`、`Vision/Camera/MT1`、`Vision/Camera/MT2`；按鍵仍看 `AprilTagTracking/Active`、`Status`、`ObservedTagId`。實機 NT4 的前綴是 `/AdvantageKit/RealOutputs/`。

## 本輪實際檔案

以下除 Helper、測試與文件外，都在 `src/main/java/com/team11855/frc2026/`。

| 檔案 | 修改 |
|---|---|
| `Constants.java` | 將舊 MT1 篩選常數換成雙通道門檻、朝向標準差；保留固定 21 |
| `RobotState.java` | 提供目前底盤 yaw rate，讀既有量測，不改 odometry callback |
| `subsystems/vision/VisionIO.java` | 增加朝向輸出介面、heartbeat 與 MT2 平均距離 |
| `subsystems/vision/VisionIOHardwareLimelight.java` | 外部朝向模式、讀 MT1/MT2、分別清空／更新 |
| `subsystems/vision/VisionSubsystem.java` | 單次組合融合、新檢查、拒絕原因與兩通道記錄；保留局部 getter |
| `subsystems/vision/MegatagPoseEstimate.java` | 處理空 raw fiducials、延遲 ms→s，修正 quality 序列化長度／schema |
| `subsystems/vision/VisionIOSimPhoton.java` | 兩組 topic 保留拍攝時間，補真實平均距離及解算 tag 清單，MT2 模擬朝向取歷史 |
| `src/main/java/com/team11855/lib/limelight/LimelightHelpers.java` | 增加短 pose 陣列／非法 tagCount 的本地解析保護 |
| `src/test/java/com/team11855/frc2026/subsystems/vision/SingleCameraVisionTest.java` | 更新單相機 fixture、時鐘及新權重預期，保留局部／全場獨立性測試 |
| 同目錄新增 `MegatagVisionTest.java` | 雙通道組合、單次 callback、度數、缺資料、時間、角速度、距離、場界、解析、struct 回歸案例 |
| `docs/drive-vision/` | 本文、README、REVIEW、歷史文件提示、圖表與靜態報告 |

另有本輪前已存在的固定 21 修改：`commands/AprilTagTrackingCommand.java`、`AprilTagTrackingCommandTest.java` 及追蹤文件／第 4 圖；一併包含在後續驗證範圍。`RobotContainer`、`Robot`、Drive 控制、Tuner、CANivore、PS5 綁定、vendor 版本本輪沒改。`BuildConstants.java` 的工作檔與原有 staged 內容均保留；不提交、不推送、不改 `main`。

## 驗證與待確認指令

已執行的範圍只有 `python3 docs/drive-vision/static_check.py`、`git diff --check` 與原始碼／API 對照。57 個 JUnit 案例是**待執行**，不是已通過；沒有執行 Java 編譯或模擬。

依你提供的 AGENTS 規則：「程式下進去編譯前要跟我完整確認會改到哪些流程才可以下」，確認上表後才執行：

```sh
./gradlew spotlessCheck test build -PteamNumber=0 -x createVersionFile
./gradlew simulateJavaRelease -PteamNumber=0 -x createVersionFile
```

第一行會檢查格式、編譯並跑 57 項 JUnit、建立 jar；必要的排版修正限定本次修改檔案。第二行啟動桌面模擬檢查定位回送、□／△／Options、失去 21、距離前進／後退、Disable／Auto→Teleop 及既有停止機制。Gradle 可能下載依賴並重生 AutoLogged 檔案；`-x createVersionFile` 用於保留你的現有 BuildConstants，這是驗證用產物，不作為正式部署版本。沒有 `deploy`、commit 或 push。

桌面模擬無法驗證 Limelight 韌體、曝光／校正、實際地圖及網路延遲；實機需在你允許的測試安排中再核對。此次沒有連上或更新機器人。
