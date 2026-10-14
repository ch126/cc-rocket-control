"""Whitelist-only mock: inspection cannot call setters, including on failures."""
from pathlib import Path
import test_wired as t

source=Path(__file__).with_name('inspect.lua').read_text(encoding='utf-8')
for fault in ('none','mass','missing','disk'):
    lua=t.LuaRuntime(unpack_returned_tuples=True)
    g=lua.globals(); g.fault=fault
    lua.execute('''
      lines={}; reads={}; files={['/rocket-inspect-0001.txt']='preserve me'}
      print=function(s) lines[#lines+1]=s end
      sublevel={}
      for _,m in ipairs({'isInPlotGrid','getUniqueId','getMass','getCenterOfMass',
        'getInertiaTensor','getInverseInertiaTensor','getLogicalPose',
        'getLinearVelocity','getAngularVelocity'}) do
        sublevel[m]=function()
          if m=='getMass' then assert(fault~='mass','mass failed'); return 100 end
          if m=='getCenterOfMass' then return {x=1,y=2,z=3} end
          return {}
        end
      end
      local allowed={getThrust=true,getPower=true,getVectorX=true,getVectorY=true,
        getTargetVectorX=true,getTargetVectorY=true,getMaxThrustOutputPn=true,isCustomThrustOutputActive=true}
      peripheral={hasType=function(n) return not (fault=='missing' and n:match('_8$')) end,
        call=function(n,m) assert(allowed[m],'unexpected write/API: '..m); reads[#reads+1]=m; return 0 end}
      textutils={serialize=function(r) assert(#r.engines==9); return 'report' end}
      fs={exists=function(p) return files[p]~=nil end,
        getFreeSpace=function() return fault=='disk' and 0 or 100000 end,
        open=function(p,m) assert(m=='w' and not files[p]); return {
          write=function(s) files[p]=s end,close=function() end} end}
    ''')
    lua.execute(source)
    assert g.files['/rocket-inspect-0001.txt']=='preserve me'
    assert bool(g.files['/rocket-inspect-0002.txt'])==(fault!='disk')
    assert len(g.reads)==(64 if fault=='missing' else 72)
print('PASS: inspection getter whitelist, mass failure, missing engine, unique report and low-disk preservation.')
