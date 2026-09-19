# 專案與 Java 套件更名

分支：`offseason`。更名前提交：`d52d6c6`。本次只更改專案名稱、Java 套件及相關引用。

| 項目 | 更名前 | 更名後 |
|---|---|---|
| 專案資料夾／Gradle 專案名稱 | `FRC-2025-Public-main` | `FRC-2026-Systemcore-Public-main` |
| 機器人程式 | `com.team254.frc2025` | `com.team11855.frc2026` |
| 共用程式 | `com.team254.lib` | `com.team11855.lib` |
| Java 啟動入口 | `com.team254.frc2025.Main` | `com.team11855.frc2026.Main` |

Java 套件片段不能以數字開頭，因此採用 `team11855`。年份套件依建議同步改成 `frc2026`，不代表將場地切換為 2026 賽季，也不改變目標控制器平台。

## 已同步的位置

- `src/main/java` 與 `src/test/java` 的資料夾、package、一般及 static imports、完整類別名稱。
- Gradle 的 `ROBOT_MAIN_CLASS`、gversion `classPackage`、產生之 `BuildConstants.java` 的格式排除與 Git 忽略路徑。
- 現行使用說明、架構圖和靜態檢查腳本。文件中的本機絕對連結改為相對連結，方便搬動專案。
- `.wpilib/wpilib_preferences.json` 保留 Java／2026／隊號 11855。

`docs/mentor-analysis` 的 206 份原始分析與快照保留原文，原 Team 254 套件名稱與上游來源網址屬歷史資料；授權與原作者資訊亦保留。先前的 `static-check.json` 是整合階段歷史紀錄，本次結果另存於 `namespace-check.json`。

## 流程影響與編譯前確認

| 流程 | 影響 |
|---|---|
| 啟動／打包 | Main 完整名稱更新，Gradle manifest 與版本資訊產生位置同步 |
| Teleop、Auto、Disabled、Test | 只更新引用；PS5、停止、路徑交接邏輯不變 |
| Drive／Vision／模擬 | 只更新套件；Tuner 數值、CANivore、PID、物理週期和視覺融合不變 |
| 路徑規劃 | 本地 API 改從 `com.team11855.lib.pathplanner` 匯入；演算法與場地不變 |
| 開發工具 | VS Code／Codex 請開啟新的資料夾；舊 build 產物不代表新套件已編譯 |

更名後先比對每個 Java 檔案：扣除套件替換後，內容須與更名前完全相同。另檢查 package 與實際路徑一致、imports 可對應、本地連結存在、原始分析雜湊不變，以及原始版本提交 `79645a4` 未改變（保存在 `main` 與本機 `codex/original-2025` 分支）。

依使用者提供的 AGENTS.md：「程式下進去編譯前要跟我完整確認會改到哪些流程才可以下」。本輪只執行靜態檢查與文件圖表產生；確認上表後，預計在新資料夾執行：

```sh
./gradlew spotlessCheck test build -PteamNumber=11855
./gradlew simulateJavaRelease -PteamNumber=11855
```

本次更名紀錄僅涵蓋靜態查核，尚未完成由本任務執行的 Java 編譯、JUnit 或模擬驗證。Git 提交與推送不代表建置已通過；本次不部署機器人。
