"""Run the real Lua controller with mocked CC events and redstone outputs."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / '.test-deps'))
from lupa.lua52 import LuaRuntime

source = Path(__file__).with_name('engine.lua').read_text(encoding='utf-8')

def run(events, fail=False):
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.execute('''
      output={top=9}; snapshots={}; events={}; cursor=0
      keys={e=1,r=2,f=3,w=4,s=5,d=6,a=7,c=8,space=9,q=10}
      redstone={setAnalogOutput=function(side,value)
        assert(value>=0 and value<=15 and value%1==0)
        output[side]=value
        assert(not ((output.front or 0)>0 and (output.back or 0)>0))
        assert(not ((output.left or 0)>0 and (output.right or 0)>0))
      end}
      term={clear=function() end,setCursorPos=function() end}
      print=function() end
      os={pullEventRaw=function()
        local copy={}; for k,v in pairs(output) do copy[k]=v end
        snapshots[#snapshots+1]=copy
        cursor=cursor+1
        if not events[cursor] then error('TEST_EVENT_EXHAUSTED') end
        return table.unpack(events[cursor])
      end}
    ''')
    for i, event in enumerate(events, 1):
        lua.globals().events[i] = lua.table_from(event)
    if fail:
        lua.execute("local old=term.clear; local n=0; term.clear=function() n=n+1; if n==4 then error('INJECTED') end; old() end")
    try:
        lua.execute(source)
        assert not fail
    except Exception as exc:
        assert fail and 'INJECTED' in str(exc), str(exc)
    out = dict(lua.globals().output)
    assert out['top'] == 9
    assert all(out[s] == 0 for s in ('bottom','front','back','left','right'))
    return [dict(t) for _,t in lua.globals().snapshots.items()]

def k(code, held=False): return ('key',code,held)

s=run([k(2),k(1),k(2),k(4),k(6),k(8),k(9),k(2),k(10)])
assert s[1]['bottom']==0 and s[3]['bottom']==1
assert s[5]['front']==1 and s[5]['right']==1
assert s[6]['bottom']==1 and s[6]['front']==s[6]['right']==0
assert s[8]['bottom']==0
run([k(1),k(2),k(1),k(2),k(10)])
s=run([k(1)]+[k(2)]*20+[k(3)]*20+[k(4)]*20+[k(5)]*40+[k(6)]*20+[k(7)]*40+[('terminate',)])
assert max(t['bottom'] for t in s)==15
assert max(t['front'] for t in s)==15 and max(t['back'] for t in s)==15
assert max(t['left'] for t in s)==15 and max(t['right'] for t in s)==15
s=run([k(1),k(2,True),('term_resize',),k(10)])
assert all(t['bottom']==0 for t in s)
run([k(1),k(2),k(4),k(10)],fail=True)
print('PASS: startup, arming, limits, axes, centering, repeat suppression, stop, quit, terminate, resize, error cleanup, unused top preserved (Lua 5.2).')
