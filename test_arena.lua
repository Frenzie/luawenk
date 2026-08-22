#!/usr/bin/env luajit
-- test_arena.lua
-- End-to-end test of wenk.lua's gesture engine, driven against a FIFO with
-- synthetic evdev streams (no real touchpad needed). Verifies the cdata-arena
-- rework produces IDENTICAL gesture outcomes to the pre-refactor code.
--
-- Usage:
--   luajit test_arena.lua                 # test the worktree wenk.lua (arena)
--   luajit test_arena.lua /path/to/wenk.lua

local ffi = require("ffi")
ffi.cdef[[int setenv(const char *name, const char *value, int overwrite);]]

local EV_SYN, EV_ABS = 0, 3
local SYN_REPORT = 0
local ABS_MT_SLOT = 0x2f
local ABS_MT_POSITION_X = 0x35
local ABS_MT_POSITION_Y = 0x36
local ABS_MT_TRACKING_ID = 0x39

local FIFO = "/tmp/wenk_test_ev"
local CFG = "/tmp/wenk_test_config"
local MARK = "/tmp/wenk_marker_"

local function ev(usec, typ, code, val)
  local e = ffi.new("struct input_event")
  e.time.tv_sec = 1
  e.time.tv_usec = usec
  e.type = typ
  e.code = code
  e.value = val
  return ffi.string(e, ffi.sizeof("struct input_event"))
end

-- load wenk.lua with the run-as-script guard stripped, returning its exports
local function load_wenk(path)
  local f = io.open(path, "rb")
  assert(f, "cannot open "..path)
  local src = f:read("*a")
  f:close()
  local idx = src:find("if pcall(debug.getlocal", 1, true)
  assert(idx, "guard not found in "..path)
  src = src:sub(1, idx - 1)
  src = src .. "return { create_device = create_device }\n"
  local chunk, err = loadstring(src, "@"..path)
  assert(chunk, err)
  local ok, mod = pcall(chunk)
  assert(ok and mod and mod.create_device, "failed to load module: "..tostring(mod))
  return mod
end

-- ================= event stream builders (Type B, MT slots) =================
local function new_stream()
  return { n = 0, parts = {} }
end
local function push(st, usec, typ, code, val)
  st.parts[#st.parts+1] = ev(usec, typ, code, val)
  st.n = st.n + 1
end
local function sync(st, usec) push(st, usec, EV_SYN, SYN_REPORT, 0) end

local function fingers_down(st, t, n, x, y)
  for f=0,n-1 do
    push(st, t, EV_ABS, ABS_MT_SLOT, f)
    push(st, t, EV_ABS, ABS_MT_TRACKING_ID, 100 + f)
    push(st, t, EV_ABS, ABS_MT_POSITION_X, x)
    push(st, t, EV_ABS, ABS_MT_POSITION_Y, y)
    sync(st, t)
    t = t + 10000
  end
  return t
end

local function fingers_move(st, t, n, from, to, step)
  for x=from,to,step do
    for f=0,n-1 do
      push(st, t, EV_ABS, ABS_MT_SLOT, f)
      push(st, t, EV_ABS, ABS_MT_POSITION_X, x)
      push(st, t, EV_ABS, ABS_MT_POSITION_Y, 400)
      sync(st, t)
    end
    t = t + 10000
  end
  return t
end

local function fingers_lift(st, t, n)
  for f=0,n-1 do
    push(st, t, EV_ABS, ABS_MT_SLOT, f)
    push(st, t, EV_ABS, ABS_MT_TRACKING_ID, 0xFFFFFFFF)
    sync(st, t)
    t = t + 10000
  end
  return t
end

-- ================= scenario: 3-finger swipe left/right, 2-finger swipe right, 3-finger tap
local function scenario_swipe(n, left)  local st = new_stream()
  local t = 100000
  t = fingers_down(st, t, n, 600, 400)
  t = fingers_move(st, t, n, 600, left and 240 or 960, left and -20 or 20)
  t = fingers_lift(st, t, n)
  return table.concat(st.parts)
end

local function scenario_tap(n)
  local st = new_stream()
  local t = 200000
  t = fingers_down(st, t, n, 500, 400)
  -- tiny movement, under threshold
  for f=0,n-1 do
    push(st, t, EV_ABS, ABS_MT_SLOT, f)
    push(st, t, EV_ABS, ABS_MT_POSITION_X, 502)
    push(st, t, EV_ABS, ABS_MT_POSITION_Y, 401)
    sync(st, t)
    t = t + 10000
  end
  t = t + 30000   -- debounce gap before lift
  t = fingers_lift(st, t, n)
  return table.concat(st.parts)
end

-- Pinch: two fingers down at distinct x, moved linearly to target x over
-- 'frames' frames with integer-exact symmetric steps (total x-cum cancels so
-- the swipe-eligibility guard never kills pinch). Spreading grows the max
-- pairwise distance => positive dis-cum delta => direction 'i'; squeezing
-- shrinks it => 'o'.
local function pinch_stream(ax0, ax1, tx0, tx1, frames)
  local st = new_stream()
  local t = 400000
  for f=0,1 do
    push(st, t, EV_ABS, ABS_MT_SLOT, f)
    push(st, t, EV_ABS, ABS_MT_TRACKING_ID, 100 + f)
    push(st, t, EV_ABS, ABS_MT_POSITION_X, f == 0 and ax0 or ax1)
    push(st, t, EV_ABS, ABS_MT_POSITION_Y, 400)
    sync(st, t)
    t = t + 10000
  end
  local step0 = (tx0 - ax0) / frames
  local step1 = (tx1 - ax1) / frames
  for k=1,frames do
    for f=0,1 do
      push(st, t, EV_ABS, ABS_MT_SLOT, f)
      local base = (f == 0) and ax0 or ax1
      local step = (f == 0) and step0 or step1
      push(st, t, EV_ABS, ABS_MT_POSITION_X, math.floor(base + step * k))
      push(st, t, EV_ABS, ABS_MT_POSITION_Y, 400)
      sync(st, t)
    end
    t = t + 10000
  end
  t = t + 30000   -- debounce gap before lift
  t = fingers_lift(st, t, 2)
  return table.concat(st.parts)
end

local function scenario_pinch_spread()
  -- 500/500 -> 320/680: distance 0 -> 360, ~24px per frame; crosses
  -- PINCH_THRESHOLD(100) cumulative around frame 5.
  return pinch_stream(500, 500, 320, 680, 15)
end

local function scenario_pinch_squeeze()
  -- 320/680 -> 500/500: distance 360 -> 0, negative deltas => direction 'o'.
  return pinch_stream(320, 680, 500, 500, 15)
end

-- Type A (no ABS_MT_SLOT): tracking-id + axis events with SYN_MT_REPORT separators
local function typeA_down(st, t, n, x, y)
  for f=0,n-1 do
    push(st, t, EV_ABS, ABS_MT_TRACKING_ID, 200 + f)
    push(st, t, EV_ABS, ABS_MT_POSITION_X, x)
    push(st, t, EV_ABS, ABS_MT_POSITION_Y, y)
    push(st, t, EV_SYN, 0x02, 0)  -- SYN_MT_REPORT
    t = t + 10000
  end
  return t
end

local function typeA_lift(st, t, n)
  for f=0,n-1 do
    push(st, t, EV_ABS, ABS_MT_TRACKING_ID, 0xFFFFFFFF)
    push(st, t, EV_SYN, 0x02, 0)
    t = t + 10000
  end
  return t
end

local function scenario_tap_typeA()
  local st = new_stream()
  local t = 300000
  t = typeA_down(st, t, 3, 500, 400)
  t = t + 30000   -- debounce gap
  t = typeA_lift(st, t, 3)
  return table.concat(st.parts)
end

-- ================= write stream to fifo, drain, check markers
local function write_fifo(data)
  local f = io.open(FIFO, "wb")
  assert(f, "cannot open fifo for writing")
  f:write(data)
  f:close()
end

local function clear_markers()
  os.execute("rm -f "..MARK.."* ")
end

local function marker(name) return MARK..name end

local function file_exists(p)
  local f = io.open(p, "rb")
  if f then f:close(); return true end
  return false
end

local function run_scenario(mod, name, data, expect, forbid)
  clear_markers()
  os.execute("rm -f "..FIFO)
  os.execute("mkfifo "..FIFO)
  local dev = mod.create_device(FIFO, 1, "touchpad")
  assert(dev, "create_device failed")
  write_fifo(data)
  for _=1,20 do dev.read_available() end
  dev.close()
  os.execute("rm -f "..FIFO)
  for _,m in ipairs(expect) do
    if not file_exists(marker(m)) then error(string.format("%s: expected marker %s missing", name, m)) end
  end
  for _,m in ipairs(forbid) do
    if file_exists(marker(m)) then error(string.format("%s: unexpected marker %s present", name, m)) end
  end
  io.stderr:write(string.format("  %-22s OK\n", name))
end

local function write_config()
  local f = io.open(CFG, "wb")
  f:write([[
{
  'pinch_deadzone_enabled': 'False',
  'touchpad': {
    'pinch_deadzone_enabled': 'False',
    'swipe': {
      '2': {
        'l': { 'start': ['touch /tmp/wenk_marker_2l'] },
        'r': { 'start': ['touch /tmp/wenk_marker_2r'] }
      },
      '3': {
        'l': { 'start': ['touch /tmp/wenk_marker_3l'] },
        'r': { 'start': ['touch /tmp/wenk_marker_3r'] },
        't': ['touch /tmp/wenk_marker_3t']
      }
    },
    'pinch': {
      '2': {
        'i': { 'start': ['touch /tmp/wenk_marker_pinch_i'] },
        'o': { 'start': ['touch /tmp/wenk_marker_pinch_o'] }
      }
    }
  }
}
]])
  f:close()
end

local function main(path)
  write_config()
  ffi.C.setenv("GESTURES_CONFIG", CFG, 1)
  local mod = load_wenk(path)
  io.stderr:write("testing "..path.."\n")

  run_scenario(mod, "3f swipe left ", scenario_swipe(3, true),  {"3l"}, {"3r","3t"})
  run_scenario(mod, "3f swipe right", scenario_swipe(3, false), {"3r"}, {"3l","3t"})
  run_scenario(mod, "3f tap        ", scenario_tap(3),        {"3t"}, {"3l","3r"})
  run_scenario(mod, "3f tap (typeA)", scenario_tap_typeA(),   {"3t"}, {"3l","3r"})
  run_scenario(mod, "2f pinch out ", scenario_pinch_spread(), {"pinch_i"}, {"pinch_o","2l","2r","3t"})
  run_scenario(mod, "2f pinch in  ", scenario_pinch_squeeze(),{"pinch_o"}, {"pinch_i","2l","2r","3t"})

  os.execute("rm -f "..FIFO)
  os.execute("rm -f "..CFG)
  clear_markers()
  io.stderr:write("PASS: all gesture scenarios match expected outcomes.\n")
end

local path = arg and arg[1] or "wenk.lua"
main(path)