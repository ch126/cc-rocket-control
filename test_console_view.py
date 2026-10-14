"""Terminal layout bounds and exact mouse target mapping."""
from pathlib import Path
import test_wired as t
lua=t.LuaRuntime(unpack_returned_tuples=True)
lua.globals().view=lua.execute(Path(__file__).with_name('console_view.lua').read_text(encoding='utf-8'))
lua.execute('''
local colors={black=1,white=2,cyan=3,orange=4,red=5,yellow=6,gray=7}
for _,size in ipairs({{51,19},{30,16},{80,25},{20,10}}) do
  local width,height=size[1],size[2]; local x,y,screen=1,1,{}
  local term={getSize=function() return width,height end,
    setBackgroundColor=function() end,setTextColor=function() end,
    clear=function() screen={} end,
    setCursorPos=function(a,b) assert(a>=1 and a<=width and b>=1 and b<=height); x,y=a,b end,
    write=function(s) assert(x+#s-1<=width); screen[y]=(screen[y] or "")..s; x=x+#s end}
  local buttons=view.draw(term,colors,{live=false,message="read only"})
  if width>=30 then
    assert(#buttons==9)
    assert(view.hit(buttons,1,8)=="pad")
    assert(view.hit(buttons,width,8)=="arm")
    assert(view.hit(buttons,1,12)=="stop")
    assert(view.hit(buttons,width,12)=="exit")
    assert(view.hit(buttons,width,11)==nil and view.hit(buttons,0,8)==nil)
    assert(screen[1]:find("READ ONLY") and screen[14]:find("disabled"))
  else assert(#buttons==0 and screen[2]:find("too small")) end
end
''')
print('PASS console layout: 51x19, 30x16, 80x25, small-screen fallback, button bounds and empty-space rejection.')
