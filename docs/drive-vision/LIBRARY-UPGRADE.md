# Drive／Vision：2026 Library 更新與編譯前確認

**階段紀錄：本文件記錄 Library 升級當時的差異；其後已依使用者要求更新底盤硬體參數。現行 CAN bus、幾何、速度與模擬週期以 [TUNER-INTEGRATION.md](TUNER-INTEGRATION.md) 為準。**

日期：2026-09-19。分支：`main`。已修改依賴設定與相容性原始碼，**尚未執行 Gradle、Java 編譯、JUnit、機器人模擬或部署**，因此此文件不是建置通過證明。

使用者已同意保留 MapleSim、採用官方 2026 beta。此次升級接續 Drive／Vision 精簡；完整前後流程見 [REVIEW.md](REVIEW.md)，全部工作樹檔案見 [CHANGES.md](CHANGES.md)。

## 鎖定的版本與來源

| 元件 | 更新前 | 本次設定 | 來源／選擇理由 |
|---|---|---|---|
| GradleRIO／WPILib | 2025.3.2 | **2026.2.1** | [WPILib 正式 release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.2.1)，未採用 2027 alpha |
| Phoenix 6 | 25.4.0 | **26.3.0** | [CTRE release](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v26.3.0)、[2026 descriptor](https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json) |
| AdvantageKit | 4.1.2 | **26.0.2** | [release 與官方 descriptor](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v26.0.2)；AutoLog processor 隨 descriptor 同版 |
| PathPlanner vendor | 2025.2.7 | **2026.1.2** | [release](https://github.com/mjansen4857/pathplanner/releases/tag/v2026.1.2)、[descriptor](https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json)；本地 fork 的處理見下節 |
| PhotonLib／PhotonTargeting | v2025.3.1 | **v2026.3.4** | [固定版本 descriptor](https://github.com/PhotonVision/photonvision/releases/download/v2026.3.4/photonlib-v2026.3.4.json)；不採用目前 latest URL 指向的 v2027.0.0-alpha-2 |
| MapleSim | 0.3.9 | **0.4.0-beta** | [官方已發布版本的 descriptor](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/4837df158cfd0c257c47b45777d563369544e624/docs/vendordep/maple-sim.json)、[Maven metadata](https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/repos/releases/org/ironmaple/maplesim-java/maven-metadata.xml) |
| LimelightHelpers 原始碼 | 1.11 | **1.14** | [官方來源快照](https://github.com/LimelightVision/limelightlib-wpijava/blob/919aa4b573a2d0bb6ecefb1af1313a1bcb44b045/LimelightHelpers.java)；調整 package 至 `com.team11855.lib.limelight` |
| WPILibNewCommands | 2025 年描述檔 | **2026 年描述檔** | `version: 1.0.0` 是描述檔標記；實際 Java 版本 `wpilib` 跟隨 GradleRIO，不能改成獨立的套件版本 |

Java 17、Gradle wrapper 8.11、gversion、Spotless、JUnit 版本保留；此組 Java／wrapper 也用於 [AdvantageKit 26.0.2 官方範本](https://github.com/Mechanical-Advantage/AdvantageKit/tree/v26.0.2/template_projects/template)。dyn4j 保留 MapleSim 官方描述檔要求的 5.0.2。Java package 名稱 `frc2025` 保留，它代表這份 2025 機器人專案，並不決定 WPILib 版本。

**MapleSim 發布差異：** 查核時官方 latest 描述檔寫的是 `0.4.0-beta-obstacles-fix`，但其 JAR、POM、sources JAR 均回傳 HTTP 404。Maven metadata 最新已發布版本是 `0.4.0-beta`，其 JAR、POM、sources JAR 可取得；因此使用官方歷史描述檔中的此版本。沒有自行編譯 MapleSim 或使用第三方 fork。未來更新 vendordep 前應再次核對實際 artifact，不能只看 latest 描述檔文字。

## 本次增量會影響的流程

| 流程 | 更新前 | 更新後／保留條件 |
|---|---|---|
| 建置與啟動 | 2025 WPILib、JNI、vendor、AutoLog | 使用 2026 組合；settings 的本機 WPILib Maven 年份改 2026。需要重新編譯與生成 AutoLogged／BuildConstants。 |
| PS5 手動底盤 | 程式轉換紅藍方輸入，CTRE request 使用預設 perspective | CTRE request 明確指定 BlueAlliance；heading target 也明確使用同一場地座標，避免重複做駕駛方向轉換。USB 0、軸符號、曲線、死區、速度、PID、Options 操作不變。 |
| 馬達／定位 IO | Phoenix 25 API 與原硬體設定 | Phoenix 26 API；原 CAN bus、CAN ID、encoder offset、反轉、gear ratio、TunerConstants 保留。實機韌體相容性須另行確認。 |
| Limelight 定位 | helpers 1.11 解讀 NetworkTables | helpers 1.14；保留相機姿態、校正、MegaTag 資料路徑、RobotState → Drive 融合。官方新版對 raw fiducial 長度不符時回傳空陣列；沒有新增相機設定或 OS 更新操作。 |
| Photon 相機模擬 | Photon 2025 結果欄位 | Photon 2026.3.4，改用公開 getters 讀取 targets／area／camera transform。保留相機 FOV、延遲、FPS、安裝位置與 2025 AprilTag 子集合。 |
| MapleSim 物理模擬 | 0.3.9 的預設 2025 場地 | 0.4.0-beta 預設是 2026，因此先安裝 `DriveSimulationArena`：僅載入 2025 障礙物，不註冊得分模擬、不生成遊戲物件、不發布比賽得分。場地 reset 也維持此行為。 |
| 模擬時間／共享姿態 | DriveIOSim 5 ms loop，SimulatedDriveState 交給 Photon | 保留同一個物理更新入口與姿態資料流；沒有增加另一個 periodic 物理更新。 |
| 跟路徑／尋路失敗 | 非有限 trajectory 時間可能一直等待或繼續舊輸出 | FollowPath／Pathfinding 在接受前辨識非有限總時間並立刻停止；Drive consumer 再擋空軌跡及非有限總時間。合法路徑中斷停止與非零終速交接仍依原契約。 |
| 短路徑生成 | 非常短的路徑可能只剩一個端點 | 保留起點，另外加入終點；避免 trajectory 讀取第二個 point 時越界。 |
| 軌跡計算與資訊 | 上游較舊的加速度／差速反向／路徑名稱行為 | 合併 2026 上游的模組加速度絕對值、差速 heading 修正、清除 trajectory 時清除 path name。Swerve 的客製限制與 controller consumer 保留。 |
| 場地翻轉 | Waypoint 使用 vendor FlippingUtil，其餘使用 local fork | Waypoint 改用 local fork，所有既有路徑仍使用 2025 場地尺寸，避免 vendor 2026 的尺寸混入。三份 navgrid 原檔保留。 |
| Disabled／Auto／Teleop／Test | 前一階段已完成取消命令與 Drive stop | 此次版本更新沿用該流程與同步鎖；Auto 仍預設不移動。異常路徑增加立即停止條件。 |
| 記錄／Replay | Drive／Vision／路徑記錄，既有 replay 限制 | AdvantageKit 26 記錄同樣的主要資料；Replay IO 沒有重設計，不能宣稱確定性重現。 |

## 本地 PathPlanner fork 的界線

`com.team11855.lib.pathplanner` 是專案內的客製原始碼，更新 vendor JSON 不會更新它。本次依 [官方 2025.2.2 → 2026.1.2 差異](https://github.com/mjansen4857/pathplanner/compare/v2025.2.2...v2026.1.2) 選擇性合併上述相容性／路徑修正，保留 100 Hz consumer、獨立 X/Y 加速度限制、尋路快取與三份網格；**它不是官方 2026.1.2 的完整逐檔複本**。

此外，Choreo `.traj` 讀取器接受的格式上限從 1 調整為 3，與上游一致；不重新加入 Choreo vendor 或賽事 Auto。上游 2026.1.2 的 `.path`／`.auto` 讀取器仍檢查 `2025.X` 格式，這與 library 發布年份不同，不能直接把檔案內 version 改成 2026。

本地 LocalADStar 已在修改 start／goal／problem 時清除 `newPathAvailable`，因此上游同類修正不需重複套用。官方新增的測試用全域 reset API 與 alert 分組沒有移植。官方 PathPlanner vendor 保留，但現有底盤路徑入口一律使用 `com.team11855.lib.pathplanner`，勿混用另一套 AutoBuilder。

## 已做的靜態核對與未執行的驗證

- 六份 vendor JSON 的版本、年份、Java／JNI／C++ 依賴一致；WPILibNewCommands 與 dyn4j 按官方描述檔處理。
- 下載並閱讀實際 Phoenix 26.3.0、Photon 2026.3.4、MapleSim 0.4.0-beta 的 sources JAR；核對此專案使用的 swerve request、sim constructor、camera 與 arena 介面。
- 比對底盤硬體設定、相機校正、navgrid 與升級前 SHA-256；保留原始完整機器人 mentor 文件與既有架構圖。
- `python3 docs/drive-vision/static_check.py` 檢查本地引用、只剩 Drive／Vision、依賴描述檔、文件連結與 `git diff --check`。結果見 [static-check.json](static-check.json)。
- Library 升級階段共 **18 個 JUnit test cases，尚未執行**。該階段新增：空／非有限軌跡停止、短路徑兩端、2025 場地翻轉、MapleSim reset 不生成遊戲物件。

這些核對不等於 Java 型別解析、annotation processor、JNI 載入或整合測試已通過。Limelight 原始碼保留供應商格式；核准後若 Spotless 回報問題，再針對本次修改檔案格式化並重查差異。

## 編譯前請確認的具體範圍

依使用者提供的 AGENTS.md 規則：「程式下進去編譯前要跟我完整確認會改到哪些流程才可以下」，完成上列修改後停在此確認點。

確認後預計使用 Java 17 執行：

```sh
./gradlew spotlessCheck test build -PteamNumber=0
./gradlew simulateJavaRelease -PteamNumber=0
```

專案缺少 `.wpilib/wpilib_preferences.json`；`-PteamNumber=0` 僅供這兩個本機驗證指令通過 Gradle 設定，不設定真實隊號、不部署。該參數由 [GradleRIO FRCExtension](https://github.com/wpilibsuite/GradleRIO/blob/v2026.2.1/src/main/java/edu/wpi/first/gradlerio/deploy/FRCExtension.java) 讀取。首次執行可能下載 Gradle／Maven／JNI 依賴並建立快取。

模擬驗收包括 PS5 操作／斷線、Options、各模式停止、Auto → Teleop 後無舊路徑覆寫、兩段合法非零終速銜接、異常路徑立即停止、Vision 融合、2025 場地與共享姿態。未來實機執行前需確認 roboRIO／Driver Station 的 2026 環境與 [Phoenix 2026 韌體相容性](https://v6.docs.ctr-electronics.com/en/stable/docs/yearly-changes/yearly-changelog.html)；LimelightHelpers 1.14 的官方最低需求為 LLOS 2026.0。本次沒有更新裝置韌體、刷相機 OS、部署、commit 或 push。

## 本次 Library 更新的實際檔案

下列清單以升級前工作樹的 SHA-256 快照為基準產生，與前一階段刪除機構的清單分開。

<!-- UPGRADE_FILES -->

| 狀態 | 檔案 |
|---|---|
| M | [README.md](<../../README.md>) |
| M | [build.gradle](<../../build.gradle>) |
| M | [docs/drive-vision/CHANGES.md](<../../docs/drive-vision/CHANGES.md>) |
| A | [docs/drive-vision/LIBRARY-UPGRADE.md](<../../docs/drive-vision/LIBRARY-UPGRADE.md>) |
| M | [docs/drive-vision/README.md](<../../docs/drive-vision/README.md>) |
| M | [docs/drive-vision/REVIEW.md](<../../docs/drive-vision/REVIEW.md>) |
| M | [docs/drive-vision/static-check.json](<../../docs/drive-vision/static-check.json>) |
| M | [docs/drive-vision/static_check.py](<../../docs/drive-vision/static_check.py>) |
| M | [settings.gradle](<../../settings.gradle>) |
| M | [src/main/java/com/team11855/frc2026/commands/DriveMaintainingHeadingCommand.java](<../../src/main/java/com/team11855/frc2026/commands/DriveMaintainingHeadingCommand.java>) |
| A | [src/main/java/com/team11855/frc2026/simulation/DriveSimulationArena.java](<../../src/main/java/com/team11855/frc2026/simulation/DriveSimulationArena.java>) |
| M | [src/main/java/com/team11855/frc2026/subsystems/drive/DriveIOSim.java](<../../src/main/java/com/team11855/frc2026/subsystems/drive/DriveIOSim.java>) |
| M | [src/main/java/com/team11855/frc2026/subsystems/drive/DriveSubsystem.java](<../../src/main/java/com/team11855/frc2026/subsystems/drive/DriveSubsystem.java>) |
| M | [src/main/java/com/team11855/frc2026/subsystems/vision/VisionIOSimPhoton.java](<../../src/main/java/com/team11855/frc2026/subsystems/vision/VisionIOSimPhoton.java>) |
| M | [src/main/java/com/team11855/lib/limelight/LimelightHelpers.java](<../../src/main/java/com/team11855/lib/limelight/LimelightHelpers.java>) |
| M | [src/main/java/com/team11855/lib/pathplanner/commands/FollowPathCommand.java](<../../src/main/java/com/team11855/lib/pathplanner/commands/FollowPathCommand.java>) |
| M | [src/main/java/com/team11855/lib/pathplanner/commands/PathPlannerAuto.java](<../../src/main/java/com/team11855/lib/pathplanner/commands/PathPlannerAuto.java>) |
| M | [src/main/java/com/team11855/lib/pathplanner/commands/PathfindingCommand.java](<../../src/main/java/com/team11855/lib/pathplanner/commands/PathfindingCommand.java>) |
| M | [src/main/java/com/team11855/lib/pathplanner/path/PathPlannerPath.java](<../../src/main/java/com/team11855/lib/pathplanner/path/PathPlannerPath.java>) |
| M | [src/main/java/com/team11855/lib/pathplanner/path/Waypoint.java](<../../src/main/java/com/team11855/lib/pathplanner/path/Waypoint.java>) |
| M | [src/main/java/com/team11855/lib/pathplanner/trajectory/PathPlannerTrajectory.java](<../../src/main/java/com/team11855/lib/pathplanner/trajectory/PathPlannerTrajectory.java>) |
| A | [src/test/java/com/team11855/frc2026/simulation/DriveSimulationArenaTest.java](<../../src/test/java/com/team11855/frc2026/simulation/DriveSimulationArenaTest.java>) |
| M | [src/test/java/com/team11855/frc2026/subsystems/drive/DriveSubsystemControlTest.java](<../../src/test/java/com/team11855/frc2026/subsystems/drive/DriveSubsystemControlTest.java>) |
| A | [src/test/java/com/team11855/lib/pathplanner/path/PathPlannerCompatibilityTest.java](<../../src/test/java/com/team11855/lib/pathplanner/path/PathPlannerCompatibilityTest.java>) |
| M | [vendordeps/AdvantageKit.json](<../../vendordeps/AdvantageKit.json>) |
| D | `vendordeps/PathplannerLib-2025.2.7.json` |
| A | [vendordeps/PathplannerLib.json](<../../vendordeps/PathplannerLib.json>) |
| D | `vendordeps/Phoenix6-frc2025-latest.json` |
| A | [vendordeps/Phoenix6-frc2026-latest.json](<../../vendordeps/Phoenix6-frc2026-latest.json>) |
| M | [vendordeps/WPILibNewCommands.json](<../../vendordeps/WPILibNewCommands.json>) |
| M | [vendordeps/maple-sim.json](<../../vendordeps/maple-sim.json>) |
| M | [vendordeps/photonlib.json](<../../vendordeps/photonlib.json>) |
