-- Terminal rendering and mouse hit-testing only; never controls engines.
local M={version=1}
function M.draw(term,colors,state)
  local width,height=term.getSize(); local buttons={}
  term.setBackgroundColor(colors.black); term.setTextColor(colors.white); term.clear()
  local function line(y,text,color)
    if y>height then return end
    term.setCursorPos(1,y); term.setTextColor(color or colors.white); term.write(text:sub(1,width))
  end
  line(1,"ROCKET | "..(state.live and state.mode or "READ ONLY"),colors.cyan)
  if width<30 or height<16 then
    line(2,"Screen too small: need 30x16",colors.orange)
    line(3,"SPACE stops | Q exits",colors.red)
    return buttons
  end
  line(2,"RETURN > COAST > BRAKE > HOVER")
  local s=state.sample
  if s then
    line(3,("Y %.2f Vy %+.2f Tilt %.1f"):format(s.y,s.vy,math.deg(s.tilt)))
    line(4,("Read %.3fs | links %s"):format(s.acquisitionSeconds,s.allNineReady and "ACK OK" or "NOT READY"))
  else line(3,"Telemetry unavailable",colors.orange) end
  local out=state.output
  line(5,"Phase: "..(out and out.plan and out.plan.phase or "--"))
  local kn=out and out.commands and out.commands[1].kn or 0
  line(6,("Main request %.1f%% | pad %s"):format(kn/733*100,state.pad and "SET" or "--"))
  local split=math.floor(width/2)
  local function button(row,col,label,action,danger)
    local x=col==1 and 1 or split+1; local w=col==1 and split or width-split
    term.setCursorPos(x,row)
    term.setBackgroundColor(danger and colors.red or (colors.gray or colors.black))
    term.setTextColor(colors.white)
    term.write((label..string.rep(" ",w)):sub(1,w))
    term.setBackgroundColor(colors.black)
    buttons[#buttons+1]={x=x,y=row,width=w,action=action}
  end
  button(8,1,"J PAD (supported)","pad"); button(8,2,"E ARM ZERO","arm")
  button(9,1,"R +1%","up"); button(9,2,"F -1%","down")
  button(10,1,"V HOVER HERE","hover"); button(10,2,"D DESCENT RETURN","descent")
  button(11,1,"B BOOSTBACK RETURN","boostback")
  button(12,1,"SPACE CUTOFF","stop",true); button(12,2,"Q EXIT","exit",true)
  line(13,"No upper stage / tower hardware",colors.orange)
  line(14,state.live and "Cutoff is NOT a safe landing" or "READ ONLY: controls disabled",colors.orange)
  line(height,state.message or "",colors.yellow)
  return buttons
end
function M.hit(buttons,x,y)
  for _,b in ipairs(buttons) do
    if y==b.y and x>=b.x and x<b.x+b.width then return b.action end
  end
end
return M
