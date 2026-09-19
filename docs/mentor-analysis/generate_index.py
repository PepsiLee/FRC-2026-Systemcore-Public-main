"""Documentation-only lexical index. Does not run/compile Java or Gradle."""
from pathlib import Path
import re,json,hashlib
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
files=sorted((ROOT/'src/main/java').rglob('*.java'))
records=[]
# Remove comments and literals, preserving newlines to keep source references accurate.
pat=re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',re.S)
def cleaned(s): return pat.sub(lambda m: ''.join('\n' if c=='\n' else ' ' for c in m.group()),s)
for f in files:
 s=f.read_text(); c=cleaned(s); pkg=re.search(r'package\s+([\w.]+)\s*;',c).group(1)
 imports=re.findall(r'import\s+(?:static\s+)?([\w.*]+)\s*;',c)
 methods=[]
 # Declaration candidates only: no claim to full Java AST or method-level call resolution.
 mp=re.compile(r'^\s*(?:public|protected|private)\s+(?:(?:static|final|synchronized|abstract|default)\s+)*(?:[\w<>?,.\[\] ]+\s+)?(\w+)\s*\(([^;{}]*?)\)\s*(?:throws\s+[^{};]+)?[;{]',re.M)
 for m in mp.finditer(c):
  start=m.start()+len(m.group(0))-len(m.group(0).lstrip())
  line=c[:start].count('\n')+1
  sig=' '.join(s[start:m.end()].split())
  methods.append({'name':m.group(1),'line':line,'signature':sig.rstrip('{;').strip()})
 records.append({'path':str(f.relative_to(ROOT)),'name':f.stem,'package':pkg,'lines':len(s.splitlines()),'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'imports':imports,'methods':methods,'clean':c})
byfq={r['package']+'.'+r['name']:r for r in records}
for r in records:
 candidates={}
 for imp in r['imports']:
  for fq,t in byfq.items():
   if imp==fq or imp.startswith(fq+'.') or (imp.endswith('.*') and t['package']==imp[:-2]): candidates[t['path']]=t
 for t in records:
  if t['package']==r['package']: candidates[t['path']]=t
 refs=[]
 for t in candidates.values():
  if t['path']==r['path']:continue
  hits=[r['clean'][:m.start()].count('\n')+1 for m in re.finditer(r'\b'+re.escape(t['name'])+r'\b',r['clean'])]
  # exclude import lines; retains construction, inheritance, parameter types and class calls
  hits=[n for n in hits if not r['clean'].splitlines()[n-1].strip().startswith('import ')]
  if hits:refs.append({'path':t['path'],'name':t['name'],'lines':sorted(set(hits))})
 r['references']=sorted(refs,key=lambda x:x['path'])
for r in records:
 r['referenced_by']=[{'path':p['path'],'name':p['name'],'lines':next(t['lines'] for t in p['references'] if t['path']==r['path'])} for p in records if any(t['path']==r['path'] for t in p['references'])]
 r.pop('clean')
(OUT/'data/source-index.json').write_text(json.dumps({'date':'2026-09-19','commit':'79645a4b0193d84b179d11654fbe499b42f2e52d','files':records},ensure_ascii=False,indent=2))
lines=['# 07｜全 Java 檔案與連接索引','','此表涵蓋本地168個Java檔。語意流程看第01～06章；本章是可重產的**靜態型別引用索引**，不是完整runtime call graph。只解析import/同package型別與宣告候選；反射、method reference、interface動態派發、callback、wildcard/nested type等可能漏記或同名誤配。沒有引用表示「此規則未找到」，不等於可刪除。單一型別的多行引用保留在JSON與HTML檢視器。方法列為宣告候選，不是逐方法執行證明。','','索引來源：`generate_index.py`；資料：`data/source-index.json`。建議使用`index.html`的檔案搜尋與引用詳情；其中每個Java檔有可點行號的唯讀HTML快照。','']
for r in records:
 lines += [f"## {r['name']}",'',f"來源：[{r['path']}](../../{r['path']})｜{r['lines']} 行｜`{r['package']}`",'']
 def links(refs,reverse=False):
  return '、'.join(f"[{x['name']}:{x['lines'][0]}](../../{x['path'] if reverse else r['path']}:{x['lines'][0]})" for x in refs) or '未找到本地型別引用（仍可能由框架或動態派發使用）'
 lines += ['接到的本地型別（點擊查看本檔引用行）：'+links(r['references']),'','被哪些本地檔引用（點擊查看對方引用行）：'+links(r['referenced_by'],True),'']
 if r['methods']:
  lines+=['公開／保護／私有方法宣告候選：','']
  for m in r['methods']: lines.append(f"- [{m['name']} · L{m['line']}](../../{r['path']}:{m['line']}) — `{m['signature']}`")
  lines+=['']
(OUT/'07-全檔案連接索引.md').write_text('\n'.join(lines))
print(f"Indexed {len(records)} Java files, {sum(r['lines'] for r in records)} lines, {sum(len(r['methods']) for r in records)} declaration candidates")
