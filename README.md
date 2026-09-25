# FRC-2026-Systemcore-Public-main

本分支 `offseason` 由 Team 254 的 [2025 Undertow 程式](https://www.team254.com/first/2025/) 精簡而來。原始完整機器人版本保留在 `main` 分支（提交 `79645a4`）；本機另保留 `codex/original-2025` 備份。

| 分支 | 用途 |
|---|---|
| `main` | Team 254 原始完整機台程式，全部機構與 ControlBoard，WPILib 2025 |
| `offseason` | 本隊 Drive＋Vision、PS5、WPILib 2026 與 Tuner 底盤設定 |

本隊 Java 程式使用 `com.team11855.frc2026`，共用程式使用 `com.team11855.lib`。專案資料夾與套件名稱的對應及驗證狀態見 [更名說明](docs/drive-vision/NAMESPACE-MIGRATION.md)。

只保留 Drive、Vision 兩個 subsystem，以及通用 PathPlanner 路徑、AprilTag 定位、AdvantageKit 記錄與底盤模擬。視覺定位採 Team 254 的 MT1 優先＋單 Tag 歷史航向備援，流程與編譯前確認見 [MT1 移植說明](docs/drive-vision/TEAM254-MT1.md)。PS5 控制器透過 WPILib 的 `CommandPS5Controller` 直接接入；舊 controlboard 與機構得分流程已移除。

詳見 [底盤／視覺使用說明與三張架構圖](docs/drive-vision/README.md)。[原始完整機器人的 mentor 分析](docs/mentor-analysis/README.md) 保留為歷史參考，其中機構與按鍵不適用此分支。

## VS Code 專案識別

請開啟包含 `build.gradle` 的專案根目錄。本分支包含 [.wpilib/wpilib_preferences.json](.wpilib/wpilib_preferences.json)，供 WPILib 擴充套件辨識為 2026 Java 專案；使用者確認的隊號為 **11855**。後續若需更改，使用 `WPILib: Set Team Number` 更新。

若先前出現 `Cannot simulate code since this is not a WPILib project`，補齊設定後可執行 `Developer: Reload Window` 再開啟 `WPILib: Open Project Information` 確認辨識狀態。`WPILib: Simulate Robot Code` 會先編譯，仍須依下方驗證規則確認後執行。

## 操作

將 PS5 控制器配置在 Driver Station 的 USB port **0**。

| 輸入 | 功能 |
|---|---|
| 左搖桿 | 場地座標平移，依紅藍方轉換駕駛方向 |
| 右搖桿左右 | 旋轉；放開後維持目前朝向 |
| Options | 保留 X/Y，重設為藍方 0°／紅方 180° |

只有 Teleop Enabled 且控制器連線時接受手動控制。Auto 預設不移動。

模擬預設 Keyboard0：W/S 前後、A/D 橫移、Q/E 旋轉、R 對應 Options。原廠 PS5 在 NI Driver Station 與桌面模擬的軸映射可能不同；模擬應先確認軸 0/1/2 分別為左 X／左 Y／右 X。

## 版本與驗證

已將依賴設定更新至 GradleRIO／WPILib 2026.2.1 與 2026 vendor 組合，保留 Java 17；MapleSim 使用官方已發布的 0.4.0-beta。版本表、來源及實際流程影響見 [Library 更新清單](docs/drive-vision/LIBRARY-UPGRADE.md)。底盤已改用使用者提供的 MK5i R2／Kraken X60 設定，接在 `canivore` CANivore；實機、路徑與模擬共用模組幾何。整合位置、參數表及仍沿用的預設值見 [Tuner 接入說明](docs/drive-vision/TUNER-INTEGRATION.md)。相機校正與 2025 AprilTag／尋路場地資料保留原設定。

目前修改後的 Java 建置、測試及模擬**尚未執行**。依本工作區規則，先審閱 [變更與編譯確認清單](docs/drive-vision/REVIEW.md)，取得使用者確認後才執行該文件列出的指令。程式驗證不部署機器人；GitHub 推送依使用者指示另行處理。

## 授權

- Team 254 code：MIT — [LICENSE](LICENSE)
- WPILib：BSD — [WPILib-License.md](WPILib-License.md)
- PathPlanner：MIT — [PathPlanner-License.md](PathPlanner-License.md)
