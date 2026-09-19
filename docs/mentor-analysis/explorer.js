(function(){
'use strict';
const esc=s=>String(s).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;');
const src=(p,line)=>'source/'+p.replaceAll('/','__')+'.html'+(line?'#L'+line:'');
const files=DOC_DATA.inventory.files;
const search=document.getElementById('file-search');
const layer=document.getElementById('file-layer');
const results=document.getElementById('file-results');
const count=document.getElementById('file-count');
function renderFiles(){
 const q=search.value.trim().toLowerCase();
 const matched=files.filter(f=>(!layer.value||f.path.includes('/'+layer.value+'/'))&&(!q||(f.path+' '+f.methods.map(m=>m.signature).join(' ')).toLowerCase().includes(q)));
 count.textContent='顯示 '+matched.length+' / '+files.length+' 個 Java 檔案';
 results.innerHTML=matched.map(f=>`<article class="file-card"><h3><a href="${src(f.path)}">${esc(f.name)}</a></h3><div class="small">${esc(f.path)} · ${f.lines} 行 · ${f.methods.length} 個方法宣告候選</div><details><summary>查看連接與方法</summary><div class="two"><div><strong>本檔接到哪些型別</strong><ul>${f.references.map(r=>`<li><a href="${src(r.path)}">${esc(r.name)}</a> — 本檔引用行 ${r.lines.map(n=>`<a href="${src(f.path,n)}">${n}</a>`).join(', ')}</li>`).join('')||'<li>規則未找到；不代表無動態連接。</li>'}</ul></div><div><strong>哪些檔案引用本檔</strong><ul>${f.referenced_by.map(r=>`<li><a href="${src(r.path)}">${esc(r.name)}</a> — ${r.lines.map(n=>`<a href="${src(r.path,n)}">${n}</a>`).join(', ')}</li>`).join('')||'<li>規則未找到；可能由框架或外部呼叫。</li>'}</ul></div></div><strong>方法宣告候選</strong><ul>${f.methods.map(m=>`<li><a href="${src(f.path,m.line)}">L${m.line}</a> <code>${esc(m.signature)}</code></li>`).join('')}</ul></details></article>`).join('');
}
search.addEventListener('input',renderFiles);layer.addEventListener('change',renderFiles);renderFiles();
// Normalize the documented export fields; no Java is executed.
const data=DOC_DATA.state;
const host=document.getElementById('state-view');
if(!data){host.innerHTML='<p>轉移資料尚未輸出，請讀第02章。</p>';return;}
const states=(data.states||[]).map(s=>typeof s==='string'?{name:s}:s);
const names=states.map(s=>s.name||s.state||s.id);
const edges=data.edges||data.transitions||[];
const from=e=>e.from||e.source||e.fromState;
const to=e=>e.to||e.target||e.toState;
const cost=e=>Number(e.cost??e.costSeconds??e.cost_seconds??e.estimatedCost??1);
const isDefault=e=>e.isDefaultCost??e.isDefault??e.defaultCost??e.uses_default_cost??e.usesDefaultCost??e.costSource==='default';
const mapped=new Map(edges.map(e=>[from(e)+'|'+to(e),e]));
const matrix='<div class="table-wrap"><table class="matrix"><thead><tr><th>from↓ / to→</th>'+names.map((n,i)=>`<th title="${esc(n)}">${i+1}</th>`).join('')+'</tr></thead><tbody>'+names.map((n,i)=>'<tr><th title="'+esc(n)+'">'+(i+1)+'</th>'+names.map((t,j)=>{const e=mapped.get(n+'|'+t);return e?`<td class="${isDefault(e)?'default':'edge'}"><button data-state="${i}" title="${esc(n)} → ${esc(t)}: ${cost(e)}s">${cost(e).toFixed(3)}</button></td>`:'<td>·</td>';}).join('')+'</tr>').join('')+'</tbody></table></div>';
host.innerHTML=`<p><span class="badge">${names.length} states</span><span class="badge">${edges.length} allowed edges</span><span class="badge">${edges.filter(isDefault).length} default costs</span></p><div class="state-key">${names.map((n,i)=>`<div>${i+1}. ${esc(n)}</div>`).join('')}</div>${matrix}<div class="controls"><label for="state-select">檢查一個狀態</label><select id="state-select">${names.map((n,i)=>`<option value="${i}">${i+1}. ${esc(n)}</option>`).join('')}</select></div><div id="state-detail" aria-live="polite"></div><p class="small">成本為轉移估計值，並非即時測量；A* 的最短路保證與 default cost限制見第02章。<a href="data/state-transitions.json">完整 JSON</a></p>`;
const select=document.getElementById('state-select');
const detail=document.getElementById('state-detail');
function updateState(){
 const name=names[Number(select.value)];
 function table(rows,dir){return '<table><tr><th>'+dir+'</th><th>成本(s)</th><th>來源</th></tr>'+rows.map(e=>`<tr><td>${esc(dir==='可直接到達'?to(e):from(e))}</td><td>${cost(e)}</td><td>${isDefault(e)?'預設1.0':'成本檔'}</td></tr>`).join('')+'</table>';}
 detail.innerHTML='<h3>'+esc(name)+'</h3><div class="two"><div>'+table(edges.filter(e=>from(e)===name),'可直接到達')+'</div><div>'+table(edges.filter(e=>to(e)===name),'可從哪裡進入')+'</div></div>';
}
select.addEventListener('change',updateState);
host.querySelectorAll('[data-state]').forEach(b=>b.addEventListener('click',()=>{select.value=b.dataset.state;updateState();select.scrollIntoView({block:'center'});}));
updateState();
})();
