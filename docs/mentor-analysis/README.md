# FRC 2025 軟體 Mentor 手冊

這份手冊把你說的 framework 拆成「架構、控制流程、資料回授、裝置接線、開發規範」來解釋，以 Team 254 Undertow 的本地 2025 程式為依據。

**建議先開 [完整閱讀版 index.html](index.html)**：包含6章詳解、7張流程圖、24個狀態／424條邊的互動矩陣、168個Java檔案的搜尋與雙向型別引用，以及可點行號的原始碼快照。整個資料夾可離線閱讀，沒有CDN依賴。

## 內容入口

|內容|閱讀重點|
|---|---|
|[01 架構與生命週期](01-架構與生命週期.md)|Main到Robot、Container組裝順序、50/100/250Hz、Disabled/Auto/Teleop、RobotState、REAL/SIM/REPLAY|
|[02 操作與Superstructure](02-操作與Superstructure.md)|三種模式逐鍵表、command/default/requirement、24種機構狀態、424條允許轉移、Coral追蹤、Factories、完整操作範例|
|[03 機構與硬體IO](03-機構與硬體IO.md)|每個機構與底層IO、CAN bus/ID/S1/S2、方向/單位/控制模式、歸零與保護、模擬差異|
|[04 底盤視覺與自動](04-底盤視覺與自動.md)|四個swerve模組、手動heading、两種對位、PathPlanner各層、Auto字串、雙相機融合、SIM真值與已知疑點|
|[05 開發規範與他隊參考](05-開發規範與他隊參考.md)|規則、流程確認表、P0/P1/P2改善、6328/1678/3061固定commit的實作比較與來源|
|[06 共用工具與資源](06-共用工具與資源.md)|lib/util、Chezy sequence/repeat、自訂語義、navgrid、相機設定、依賴與部署資源|
|[07 全檔案連接索引](07-全檔案連接索引.md)|全部168個Java檔案、接到的型別、反向引用、方法宣告候選；搜尋請用HTML版|
|[08 圖表總覽](08-圖表總覽.md)|可直接閱覽、分享的PNG/SVG與可重產DOT來源|
|[狀態轉移 CSV](data/state-transitions.csv)|424条邊、成本、預設成本與來源，可匯入表格軟體|
|[狀態轉移 JSON](data/state-transitions.json)|24 states、424 edges、19 self loops、114預設成本、2個過時成本key|

## 作為 mentor，我建議先掌握的事

1. **兩套尋路不能混用概念。** Superstructure的A*決定機構姿態轉移；LocalADStar決定底盤在場上怎麼走。
2. **控制、量測與顯示要分清。** RobotState主要共享資料；CTRE負責此架構的底盤估測；Logger/Viz負責觀測。圖上看起來正確不等於真機到位。
3. **按鍵必須連同模式看。** 同一按鍵在CORAL、ALGAECLIMB、CORALMANUAL作用不同。
4. **Command結束不等於安全輸出已定義。** timeout、中斷、Disable、default接手、恢復限位/障礙圖都要有明確契約。
5. **Replay入口不等於全機可確定性重播。** Drive與Vision目前有繞過統一inputs的資料流；MapleSim還會把控制位姿覆寫為物理真值。
6. **移植前先驗證輸入、轉移與保護。** 包括Auto字串、transition取消flag、A*成本假設、CANrange失效策略、機构軟硬限位與單位。報告已區分目前啟用的流程、未被呼叫的API與待驗證風險。

## 範圍、可信度與限制

- 基準日期：2026-09-19；本地commit：`79645a4b0193d84b179d11654fbe499b42f2e52d`。
- 原始碼：168個Java檔、31,686行（含註解/空行）；76個robot檔、92個lib檔。
- 語意詳解以本地程式為準；外隊來源分成直接閱讀的原始碼、README宣稱與本報告建議，並附固定版本連結。
- 逐檔索引是**型別引用靜態解析**，不是完整Java AST、method-level call graph或實際執行trace。動態派發、回呼、反射及部分巢狀類別仍需人工追蹤。
- 424条allowed edges代表程式列出的可轉移關係，不代表機械碰撞都已驗證；runtime mode/game-piece blocking另有條件。
- 裝置圖只呈現程式中的bus/ID/通道，無法證明實體電線、供電、termination、韌體或安裝方向。
- 已做來源行號/本地連結/狀態資料/圖檔的靜態檢查，並人工查看生成圖表。瀏覽器檢視本地HTML受到環境限制，未完成實際瀏覽器互動驗證；不宣稱互動介面已經過完整瀏覽器測試。
- 沒有執行Gradle、Java編譯、robot simulation、測試程式或deploy；**機器人原始碼與設定沒有更改**。文件產生器僅處理文字、圖與HTML。

你提供的工作規則是：「程式下進去編譯前要跟我完整確認會改到哪些流程才可以下」。因此後續若要實作報告中的修正，應先拿第05章確認表列出具體diff、按鍵／Auto／機構／IO／取消／Disabled／SIM／Replay影響與驗收方式，再取得你的確認。

## 文件維護

`generate_index.py`讀Java文字產生引用索引與hash，`render_docs.mjs`把Markdown與DOT渲染為離線閱讀版及原始碼快照。Node套件路徑可由`FRC_DOC_NODE_MODULES`指定；需要既有marked與@viz-js/viz。狀態JSON/CSV為本次靜態分析資料，修改enum/成本後需重新核對並更新，不能只跑索引產生器就視為狀態表同步完成。

保留原專案的Team254／WPILib／PathPlanner授權文件；source資料夾是本地Java的唯讀HTML副本，包含原始註解與版權資訊。
