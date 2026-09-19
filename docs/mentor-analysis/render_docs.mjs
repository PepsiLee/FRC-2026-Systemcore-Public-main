/** Documentation renderer only. No Java/Gradle execution. */
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const here=path.dirname(fileURLToPath(import.meta.url));
const root=path.resolve(here,'../..');
const deps=process.env.FRC_DOC_NODE_MODULES || '/Users/pepsi/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules';
const {marked}=await import(path.join(deps,'marked/lib/marked.esm.js'));
const {instance}=await import(path.join(deps,'@viz-js/viz/dist/viz.js'));
const viz=await instance();
for(const name of fs.readdirSync(path.join(here,'diagrams')).filter(x=>x.endsWith('.dot'))){
 const dot=fs.readFileSync(path.join(here,'diagrams',name),'utf8');
 const result=viz.renderString(dot,{format:'svg',engine:'dot'});
 fs.writeFileSync(path.join(here,'diagrams',name.replace('.dot','.svg')),result);
}
const esc=s=>String(s).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;');
const inventory=JSON.parse(fs.readFileSync(path.join(here,'data/source-index.json'),'utf8'));
const sourceDir=path.join(here,'source');fs.mkdirSync(sourceDir,{recursive:true});
const sourceSlug=p=>p.replaceAll('/','__')+'.html';
const css=`:root{color-scheme:light;--ink:#182d42;--muted:#52677b;--line:#d7e3ef;--blue:#1262a3}*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:#f6f9fc;color:var(--ink);font:16px/1.75 -apple-system,BlinkMacSystemFont,"PingFang TC","Noto Sans TC",sans-serif}a{color:var(--blue)}header{padding:40px 36px 26px;background:#102b46;color:#fff}header p{max-width:940px;color:#c5d9eb;margin:.6em 0}h1{font-size:32px;line-height:1.3;margin:.3em 0}h2{font-size:25px;margin:1.7em 0 .6em}h3{font-size:20px;margin:1.3em 0 .6em}p{margin:.65em 0}main{max-width:1500px;margin:auto;padding:24px 32px 80px}nav{display:flex;flex-wrap:wrap;gap:8px;padding:16px 32px;background:#e9f1f8;border-bottom:1px solid var(--line)}nav a{padding:7px 12px;text-decoration:none;border:1px solid #bad0e2;border-radius:6px;background:white}section.chapter{background:white;padding:26px 32px;margin:24px 0;border:1px solid var(--line);border-radius:10px;overflow-wrap:anywhere}section.chapter>h1{font-size:29px;color:#194e79}table{width:100%;border-collapse:collapse;margin:16px 0;font-size:14px}th,td{padding:10px 12px;border:1px solid var(--line);text-align:left;vertical-align:top}th{background:#eaf2f8}tr:nth-child(even){background:#f7fafc}.table-wrap{overflow-x:auto}code{font:13px/1.6 ui-monospace,SFMono-Regular,monospace;background:#edf2f7;padding:2px 4px;border-radius:3px}pre{overflow:auto;background:#edf2f7;padding:16px;border-radius:6px;white-space:pre-wrap}pre code{background:transparent;padding:0}.diagram{margin:24px 0;border:1px solid var(--line);padding:12px;background:#fff;border-radius:8px}.diagram img{display:block;width:100%;height:auto}.diagram figcaption{padding:10px 8px;color:var(--muted)}summary{cursor:pointer;font-weight:600;padding:10px 0}details{border-top:1px solid var(--line);margin-top:12px}.badge{display:inline-block;margin:4px 6px 4px 0;padding:4px 12px;border-radius:20px;background:#e8f4f0;color:#196450;font-size:14px}.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap}input,select,button{font:inherit;padding:9px 12px;border:1px solid #afc4d7;border-radius:6px;background:white;color:var(--ink)}input{min-width:250px;flex:1}button{cursor:pointer}.file-card{padding:14px 0;border-bottom:1px solid var(--line)}.file-card h3{margin:0}.small{font-size:14px;color:var(--muted)}.two{display:grid;grid-template-columns:1fr 1fr;gap:24px}.source-code{background:#fff;padding:24px;overflow-x:auto}.source-line{display:block;white-space:pre;font:13px/1.5 ui-monospace,SFMono-Regular,monospace;min-height:19px}.source-line:target{background:#fff0bb}.ln{display:inline-block;width:56px;padding-right:12px;text-align:right;text-decoration:none;color:#66819a;user-select:none}.matrix{font-size:11px;table-layout:fixed;min-width:960px}.matrix td,.matrix th{padding:4px;text-align:center}.matrix th:first-child{width:44px}.matrix td.edge{background:#d8eee8;color:#125340}.matrix td.default{background:#f8e8c7;color:#7d4f00}.matrix button{border:0;background:transparent;padding:1px;font-size:11px;width:100%;border-radius:0}.notice{padding:14px 18px;background:#fff5df;border-left:4px solid #c38b26}.state-key{columns:2;font-size:13px}.muted{color:var(--muted)}@media(max-width:800px){header,main,nav{padding-left:16px;padding-right:16px}section.chapter{padding:18px 16px}h1{font-size:26px}.two{grid-template-columns:1fr}.state-key{columns:1}table{font-size:13px}input{min-width:180px}.diagram{padding:4px}}@media print{body{background:white}header{background:white;color:#142d42}header p{color:#52677b}nav,.controls{display:none}main{padding:0}section.chapter{border:0;break-before:page}.diagram{break-inside:avoid}a{color:#123f66}details{display:block}}`;
for(const r of inventory.files){
 const src=fs.readFileSync(path.join(root,r.path),'utf8');
 const html=`<!doctype html><html lang="zh-Hant"><meta charset="utf-8"><title>${esc(r.name)} 原始碼</title><style>${css}</style><header><a style="color:white" href="../index.html#files">← 回逐檔索引</a><h1>${esc(r.name)}</h1><p>${esc(r.path)} · 本地快照 · 唯讀來源</p></header><div class="source-code">${src.split('\n').map((l,i)=>`<span class="source-line" id="L${i+1}"><a class="ln" href="#L${i+1}">${i+1}</a>${esc(l)}</span>`).join('')}</div></html>`;
 fs.writeFileSync(path.join(sourceDir,sourceSlug(r.path)),html);
}
function linkReplace(html){
 html=html.replaceAll(root+'/', '../../');
 return html.replace(/href="(?:\.\.\/\.\.\/)?(src\/[^"#]+?)(?::(\d+)|#L(\d+))?"/g,(all,p,line,hash)=>{
  if(p.endsWith('.java'))return `href="source/${sourceSlug(p)}${line||hash?'#L'+(line||hash):''}"`;
  return all;
 });
}
let sections='';
const chapters=fs.readdirSync(here).filter(x=>/^0[1-6]-.*\.md$/.test(x)).sort();
for(const name of chapters){
 let html=linkReplace(marked.parse(fs.readFileSync(path.join(here,name),'utf8')));
 html=html.replaceAll('<table>','<div class="table-wrap"><table>').replaceAll('</table>','</table></div>');
 sections+=`<section class="chapter" id="chapter-${name.slice(0,2)}">${html}</section>`;
}
const graphNames=fs.readdirSync(path.join(here,'diagrams')).filter(x=>x.endsWith('.svg')).sort();
const captions=['整體架構：先分清組裝、意圖、協調、控制、IO與共享資料。','週期與執行緒：相機在主週期讀，底盤odometry和controller有獨立執行脈絡。','機構命令鏈：狀態機提出合法轉移，Factories把轉移變成命令。','視覺融合：MT1與gyro fallback的驗證條件不完全相同。','自動流程：規劃、機構到位和得分判斷互相配合。','硬體拓撲：CAN ID必須連同bus辨識；CANrange在drivebase-climber。','Replay現況：有入口不表示所有inputs都能確定性重播。'];
let charts=graphNames.map((n,i)=>`<figure class="diagram"><img src="diagrams/${n}" alt="${esc(captions[i]||n)}"><figcaption>${i+1}. ${esc(captions[i]||n)} <a href="diagrams/${n}">開啟向量圖</a> · <a href="diagrams/${n.replace('.svg','.dot')}">圖表來源</a></figcaption></figure>`).join('');
const statePath=path.join(here,'data/state-transitions.json');
const state=fs.existsSync(statePath)?JSON.parse(fs.readFileSync(statePath,'utf8')):null;
const payload=JSON.stringify({inventory,state}).replaceAll('<','\\u003c');
const html=`<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>FRC 2025 · Mentor 架構手冊</title><style>${css}</style></head><body><header><div class="small" style="color:#aad1ef">TEAM 254 / UNDERTOW / 2025 SNAPSHOT</div><h1>從手把到馬達：FRC 軟體架構手冊</h1><p>生命週期、按鍵與模式、機構與硬體、底盤與視覺、自動流程、共用工具、開發規範，以及每個 Java 檔案的連接入口。</p><span class="badge">168 Java 檔</span><span class="badge">31,686 行靜態分析範圍</span><span class="badge">只新增文件 · 未編譯／部署</span><p class="small" style="color:#c5d9eb">分析日期 2026-09-19 · 本地 commit 79645a4b0193 · 架構事實、靜態疑點與待實機驗證事項分開標示</p></header><nav><a href="#read">閱讀方式</a><a href="#charts">7 張流程圖</a><a href="#states">狀態轉移</a>${chapters.map(n=>`<a href="#chapter-${n.slice(0,2)}">${esc(n.replace('.md',''))}</a>`).join('')}<a href="#files">168 檔案索引</a></nav><main><section class="chapter" id="read"><h2>如何使用</h2><p>先看總圖，再沿著一個操作追「輸入 → 命令 → 子系統 → IO → 回授」。章節中的程式連結可直接打開本快照的唯讀原始碼行號。</p><p>全文可用瀏覽器搜尋；逐檔索引可按類名、路徑、方法名篩選。狀態矩陣可查看所有允許邊，再選狀態查看進出連接與成本。</p><div class="notice">程式宣告的裝置拓撲不能取代實體接線圖。此報告没有執行機器人程式，靜態問題候選尚未用編譯／SIM／實機重現。任何後續改碼、編譯或下程式前，先依使用者要求完整確認受影響流程。</div></section><section class="chapter" id="charts"><h2>架構與資料流程圖</h2>${charts}</section><section class="chapter" id="states"><h2>Superstructure 完整轉移矩陣</h2><p>橫列為來源、直欄為目標；綠色有成本資料，黃色使用預設成本。空白是不允許的直接轉移。顯示到小數三位，詳情保留原值。自迴圈在資料中存在，實際相同目標的命令處理另看第02章。</p><div id="state-view"></div></section>${sections}<section class="chapter" id="files"><h2>全 Java 檔案連接索引</h2><p>這是靜態型別引用與方法宣告候選，並非完整 runtime call graph。回呼、介面動態派發等需配合前文章節。未找到引用不代表可刪除。</p><div class="controls"><label for="file-search">類名／路徑／方法</label><input id="file-search" type="search" placeholder="例如 Claw、setControl、Pathfinding"><label for="file-layer">範圍</label><select id="file-layer"><option value="">全部</option><option value="frc2025">機器人程式</option><option value="lib">共用庫</option></select></div><p id="file-count" aria-live="polite"></p><div id="file-results"></div></section></main><script>const DOC_DATA=${payload};</script><script src="explorer.js"></script></body></html>`;
fs.writeFileSync(path.join(here,'index.html'),html);
console.log('Rendered '+chapters.length+' chapters, '+graphNames.length+' SVG diagrams, '+inventory.files.length+' source pages. State data: '+!!state);
