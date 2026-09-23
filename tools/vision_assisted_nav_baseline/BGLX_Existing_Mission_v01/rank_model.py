import argparse,base64,json,time,urllib.request
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--model',default='qwen2.5vl:7b');a=p.parse_args()
report=json.loads((a.folder/'candidates.json').read_text());ids=[c['id'] for c in report['candidates'] if 1.0 <= float(c.get('width_at_scan_height_m', 0)) <= 2.1]
result={'model':a.model,'ranking':[],'advisory_only':True};start=time.monotonic()
try:
 if not ids:raise ValueError('No candidates')
 prompt='Rank the labelled openings '+', '.join(ids)+' for further inspection by a delivery trike. Mention visible obstructions. Do not certify clearance. Reply JSON only: {"ranking":[],"reason":"..."}. Use only supplied IDs; omit uncertain choices.'
 payload={'model':a.model,'stream':False,'format':{
    'type':'object',
    'properties':{
        'ranking':{
            'type':'array',
            'items':{'type':'string','enum':ids},
            'maxItems':len(ids)
        },
        'reason':{'type':'string'}
    },
    'required':['ranking','reason'],
    'additionalProperties':False
},'messages':[{'role':'user','content':prompt,'images':[base64.b64encode((a.folder/'candidates.jpg').read_bytes()).decode()]}],'options':{'temperature':0,'num_predict':200}}
 req=urllib.request.Request('http://127.0.0.1:11434/api/chat',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
 with urllib.request.urlopen(req,timeout=90) as response:raw=json.load(response)
 text=raw.get('message',{}).get('content','');result['raw']=text;parsed=json.loads(text);rank=parsed.get('ranking',[])
 if not isinstance(rank,list) or any(not isinstance(x,str) or x not in ids for x in rank) or len(rank)!=len(set(rank)):raise ValueError('Invalid candidate IDs')
 result.update(ranking=rank,reason=parsed.get('reason',''))
except Exception as e:result['fallback_reason']=str(e)
result['seconds']=time.monotonic()-start;(a.folder/'model_rank.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
