# 修改完成後的編譯確認清單

> 最新定位原始碼已改為 MT2 位置＋MT1 朝向，完整檔案清單、流程差異及待確認指令見 [MEGATAG-IO.md](MEGATAG-IO.md)。下方保留前階段的確認紀錄；其中「MT1／gyro 備援保留」只描述當時版本。

最新增量：□／△ 固定指定 21 號；相機優先 ID、場地定位與局部追蹤分流、模擬及測試變更見 [本輪確認](APRILTAG-TRACKING.md#本輪指定-21-號的增量變更)。此增量尚未編譯或部署。

本輪新增 □ 對準／△ 持有 1 公尺跟隨，完整流程、檔案與待確認指令見 [APRILTAG-TRACKING.md](APRILTAG-TRACKING.md)。

前一階段單顆 Limelight 變更、實際檔案與待確認指令見 [SINGLE-CAMERA.md](SINGLE-CAMERA.md)。本輪尚未編譯、測試或模擬；下方較早階段的紀錄保留供參考。

目前 Java 套件與專案資料夾已依 [更名說明](NAMESPACE-MIGRATION.md) 更新；本次更名後尚未編譯。下方保留底盤整合的完整流程確認。

分支：`offseason`。基準：`main` 的 `79645a4b0193d84b179d11654fbe499b42f2e52d`。此分支依使用者指示提交並推送至 GitHub；編譯、測試與模擬仍需另行確認，不部署機器人。

最新 Tuner 硬體與模擬參數接入見 [TUNER-INTEGRATION.md](TUNER-INTEGRATION.md)，此項也包含在編譯前確認範圍。

Library 更新詳情與本次增量檔案見 [LIBRARY-UPGRADE.md](LIBRARY-UPGRADE.md)。下表包含前一階段 Drive／Vision 精簡，新增的 2026 遷移影響另列於該文件。

**目前沒有執行 Java 編譯、JUnit、Gradle、機器人程式或模擬。** 下列是實際修改與待確認的驗證工作；不是測試通過報告。

## 會改到的流程

| 流程 | 原本 | 此分支 |
|---|---|---|
| 啟動／容器 | 建立全部機構、modal controls、狀態機與 Auto chooser | 只建立 Drive、Vision；RobotState 連接兩者；直接建立 port 0 的 CommandPS5Controller |
| 手動移動 | Xbox／自訂映射 → ControlBoard → Drive 命令 | PS5 左 Y／左 X／右 X → 原本的輸入曲線 → 獨立 Drive 命令 |
| 朝向維持 | 依機構狀態可能轉向 Reef／Barge／Processor | 保留放開右搖桿後維持當前朝向；比賽專用轉向全部移除 |
| Options | 原 controlboard 控制 | 僅 Teleop Enabled 且連線時生效；保留 X/Y，朝向改藍方 0°／紅方 180°，用 pending heading 跨 default command 重啟，避免等待非同步定位回報 |
| 輸入不可用 | 沒有此獨立 gate | 非 Teleop Enabled 或 PS5 斷線：讀軸前立即 stop，清除朝向目標 |
| Autonomous | 依 chooser 生成含得分／取物的路徑組合 | getAutonomousCommand() 預設 Commands.none()，不移動；通用 AutoBuilder、跟路徑、尋路、定點對位可供後續呼叫 |
| 尋路 warmup | vendor PathPlanner 與賽事專用 warmup | 使用實際底盤所接的 local fork warmup；輸出 consumer 為空，不控制馬達；保留三份 navgrid |
| Disabled | 機構重設、比賽 Auto 再生成及診斷 | 取消 Auto、停止底盤，保留記錄同步／drive CAN bus 診斷與通用尋路設定 |
| Auto → Teleop | 比賽狀態重設＋底盤接管 | 取消 Auto → stop → PS5 default 接手；100 Hz thread 與手動接管共用鎖 |
| Test／模式離開 | cancelAll 或機構專用清理 | Test 入口 cancelAll＋stop；Auto、Teleop、Test 出口停止底盤 |
| 路徑完成／中斷 | 中斷可能以 null 交接，最後速度持續 | 中斷、Disabled、零終速都立即停止；正常非零終速路徑保持無零速脈衝交接 |
| 重設定位 | AutoBuilder callback 空白 | callback 接到 resetOdometry；先停止路徑／馬達，再重設底盤 pose |
| 模擬 | 完整機構、取物、拋射與得分模擬 | 只保留底盤物理與相機模擬；SimulatedDriveState 共用底盤真值姿態歷史；由 DriveIOSim 的 4 ms loop 更新物理，採使用者的模組幾何 |
| 記錄／Replay | 完整機器人的資料與 replay 入口 | 刪除機構記錄、保留 Drive／Vision／RobotState／PathPlanner；保留 replay 入口，未重新設計 replay IO |
| Dashboard | 含機構、得分選項、舊操作模式 | 精簡為 Drive／Vision、定位、路徑及對應診斷項目；只做 JSON 與 key 靜態核對，尚未實際匯入 UI |

輸入曲線維持原值：平移 `-sign(axis) * abs(axis)^1.5`，旋轉 `-sign(axis) * axis^2`。平移死區比例 5%、旋轉輸入死區及 heading P=5/I=0/D=0 保留。依使用者 Tuner 設定，最大平移改為 5 m/s，手動最大角速度改為 0.4 × 2π ≈ 2.513 rad/s，附中文單位註解。

## 主要程式介面

- `DriveMaintainingHeadingCommand` 接受 Drive、RobotState、三個軸 supplier，另加 `BooleanSupplier driverControlEnabled`；命令不認識 PS5 或 RobotContainer，也不接機構狀態。
- `DriveSubsystem.stop()` 清路徑、停／重設計時器、立即送零速度。`setControl()` 表示手動接管；`applyRequest()` 持有 Drive requirement，結束會停止。
- 100 Hz trajectory loop、trajectory accept、手動接管、stop、resetOdometry 共用 `controlLock`，防止停止完成後被舊 tick 覆寫。
- 原 `Consumer<PathPlannerTrajectory>` 保留。`null` 僅表示正常非零終速交接；停止應使用 `stop()` 或停止 sentinel。
- `SimulatedDriveState` 取代 `SimulatedRobotState`，沒有 container、機構或遊戲物件依賴。
- 刪除八個 subsystem 目錄、controlboard、比賽 auto／factory／viz、機構通用基底與未被保留程式引用的工具。
- 刪除未再使用的 Phoenix 5 與 Choreo vendor descriptor。2026 library 更新已另行授權完成設定與原始碼修改；官方 PathPlanner vendor 保留為 2026.1.2，本地 fork 現在獨立使用 2025 場地翻轉工具。

完整實際檔案清單見 [CHANGES.md](CHANGES.md)。程式連線、圖表與開發規則見 [README.md](README.md)。

## 已完成的靜態確認

- 檢查 source 只剩 drive／vision subsystem，且沒有被移除套件的 import／型別引用。
- 檢查保留 Java 的本地 import 目標、JSON 可解析性、文件連結及 `git diff --check`。
- DriveIOHardware、VisionSubsystem、VisionIOHardwareLimelight、Main、navgrid 與相機校正保留；CompTunerConstants 已依使用者檔案更新，Prac／Sim 的舊參數檔及 MAC 自動切換已移除。
- 原 `docs/mentor-analysis` 的文件、圖表、來源快照以 SHA-256 核對保留。
- CAN bus 與三份 dashboard 診斷 topic 已改為 `canivore`；執行原始碼沒有舊 `drivebase-climber` 引用。

可重跑靜態檢查：`python3 docs/drive-vision/static_check.py`。結果見 [static-check.json](static-check.json)。它只做文字／檔案結構檢查，不能取代 Java 編譯、型別解析或實機測試。

## 確認後才會執行的指令

使用 Java 17 與專案鎖定的 WPILib 2026.2.1／2026 vendor 依賴。已補上 `.wpilib/wpilib_preferences.json`，供 VS Code 辨識 2026 Java 專案，並設定使用者確認的隊號 **11855**。以下只為本機建置與模擬傳入 `-PteamNumber=0`，不改寫 preferences 中的實際隊號。Gradle 首次執行可能需要下載對應依賴，並產生 BuildConstants／AutoLogged 類別與 build 快取。

```sh
./gradlew spotlessCheck test build -PteamNumber=0
./gradlew simulateJavaRelease -PteamNumber=0
```

第一行檢查格式、編譯、執行 JUnit、建立產物。若格式檢查失敗，修正本次變更檔案，不對整個 upstream codebase 大規模重排。第二行啟動桌面模擬，執行下表的互動驗收。Replay 僅在有適當紀錄檔時另外使用 `./gradlew simulateJavaRelease -PteamNumber=0 -Preplay` 驗證讀取入口；缺紀錄檔時標註未驗證，不捏造通過。

不執行 `deploy`、不連線啟動實體機器人，也不推送 Git 遠端。

## 待執行的驗收

| 範圍 | 測試／驗收情境 | 目前狀態 |
|---|---|---|
| PS5 原生軸 | PS5ControllerSim 軸符號、曲線與全行程速度比例 | JUnit 已撰寫，未執行 |
| 手動命令 | 禁止輸入時不讀軸、朝向維持、紅方平移、旋轉後回中重新鎖向 | JUnit 已撰寫，未執行 |
| Options | RobotState 仍是舊 yaw 時，下次 default start 使用指定新朝向 | JUnit 已撰寫，未執行 |
| 控制權 | 手動接管取消路徑、command end stop、停止後新路徑可啟動 | JUnit 已撰寫，未執行 |
| 100 Hz 同步 | 舊 tick 執行中同時 stop，stop 返回後不再被覆寫 | JUnit 已撰寫，未執行 |
| 路徑交接 | null 保留非零終速、重設定位前先停 | JUnit 已撰寫，未執行 |
| 共享模擬姿態 | 初始無資料、最新姿態／插值、舊資料過期 | JUnit 已撰寫，未執行 |
| 2026 相容性 | 非有限時間／空軌跡停止、短路徑保留兩端、2025 waypoint 翻轉、場地 reset 不生成物件 | JUnit 已撰寫，未執行 |
| Tuner 接入 | 模擬與路徑幾何順序一致、模擬不覆寫實機校正與 PID、滿搖桿使用 m/s 與 rad/s | JUnit 已撰寫，未執行 |
| 整合生命週期 | Auto→Teleop、Disable→Enable、Test cancelAll、PS5 拔除／重接 | 待模擬驗收 |
| 尋路／視覺 | 路徑中斷、兩段非零終速銜接、Auto 起點重設、Vision 回送定位 | 待模擬驗收 |
| 實體 PS5 | NI DS port 0、左／右搖桿與 Options 映射 | 本次不部署，未實機驗證 |

## 保留的上游限制

底盤模組使用本次提供的 MK5i R2／Kraken X60 參數。使用者同意重量、保險桿尺寸、慣量等未提供數值先沿用原預設；相機校正、2025 場地與 Vision 演算法亦保留。這些預設不代表新機器人的實測值。

Replay 仍沿用上游以 `RobotBase.isSimulation()` 建立 DriveIOSim／Photon 的模式；live simulation 的 RobotState 更新與日誌輸入回填並未在此次重構中重設，因此不能宣稱可確定性重現所有視覺／定位狀態。

通用 AutoAlign 的原有控制演算法未重寫；它是定點對位，不提供避障。所有路徑／對位命令仍需有效的目標、單位及 Drive requirement。
