#!/usr/bin/env luajit
-- LuaJIT port of gestures <https://github.com/natask/gestures>
-- Performance is much better than in Python.
-- Reads /dev/input/event*, classifies gestures, dispatches configured commands.

local ffi = require("ffi")
local bit = require("bit")

-- ================== FFI DECLS ==================
ffi.cdef[[
typedef long ssize_t;
typedef unsigned long size_t;
typedef int pid_t;
typedef long time_t;

struct timeval { time_t tv_sec; time_t tv_usec; };
struct input_event {
  struct timeval time;
  unsigned short type;
  unsigned short code;
  unsigned int value;
};

struct pollfd {
  int fd;
  short events;
  short revents;
};

int open(const char *pathname, int flags, ...);
int close(int fd);
ssize_t read(int fd, void *buf, size_t count);
int ioctl(int fd, unsigned long request, ...);
int poll(struct pollfd *fds, unsigned long nfds, int timeout);
unsigned int sleep(unsigned int seconds);
int usleep(unsigned int usec);

pid_t fork(void);
int execvp(const char *file, char *const argv[]);
pid_t waitpid(pid_t pid, int *status, int options);

char *strerror(int errnum);
]];

-- Packed per-finger state (cdata arena): avoids per-event hash lookups and
-- per-gesture-start/end table churn (resets become memset-speed ffi.fill).
ffi.cdef[[
typedef struct {
  double x, y;
  double last_x, last_y;
  bool active;
  bool first;
  bool x_updated, y_updated;
} wenk_point_t;

typedef struct {
  double xcum, ycum;
  int32_t moved;
} wenk_acc_t;

typedef struct {
  double xcum, ycum, discum;
  int32_t moved;
} wenk_total_t;
]]

-- ================== CONSTANTS ==================
-- Linux input constants
local EV_SYN = 0x00
local EV_ABS = 0x03
-- SYN codes
local SYN_REPORT     = 0x00
local SYN_MT_REPORT  = 0x02

local ABS_MT_SLOT        = 0x2f
local ABS_MT_POSITION_X  = 0x35
local ABS_MT_POSITION_Y  = 0x36
local ABS_MT_TRACKING_ID = 0x39
-- Single-touch fallbacks
local ABS_X = 0x00
local ABS_Y = 0x01

-- Poll flags
local POLLIN   = 0x001
local POLLERR  = 0x008
local POLLHUP  = 0x010
local POLLNVAL = 0x020

-- Open flags
local O_RDONLY   = 0
local O_NONBLOCK = 0x800

-- Gesture thresholds (match Python defaults)
local DECISION = 90
local PINCH_DECISION = 160

local ANGLE_X = 20
local ANGLE_Y = 20
local TANX = math.tan(ANGLE_X * math.pi/180)
local TANY = math.tan((90-ANGLE_Y) * math.pi/180)

local DEBOUNCE = 0.02
local THRESHOLD_SQUARED = 10
local PINCH_THRESHOLD = 100

local REP_THRES = 0.2
local REP_DAG_DEF = 250
local REP_DEF = 150
local PINCH_REP_DEF = 40
local WNOHANG = 1

local EINTR = 4
local EAGAIN = 11

-- Upper bound for tracked MT slots. Kernel slot ids and the synthetic Type A
-- ids stay far below this; events referencing higher slots are dropped with a
-- one-time warning.
local MAX_SLOTS = 64

-- Main loop tuning
local RESCAN_MS = 1000
local MAX_DEVICES = 16
local OPEN_RETRY_S = 5 -- cooldown before re-attempting a failed device open

local DEADZONE_SQUARED = 1000

local TOUCHPAD_CALIBRATION = 1
local TOUCHSCREEN_CALIBRATION = 2

-- Debug controls
local DEBUG = false
local STOP = false
local TYPE_FILTER = {} -- e.g. {"angle","slot","gesture"}
local RUNNING = true
local BENCHMARK = false
local BENCH_ITERS = 50000

-- ================== UTIL ==================
local function should_log(tag)
  if not DEBUG then return false end
  if (#TYPE_FILTER == 0) then return true end
  for _,t in ipairs(TYPE_FILTER) do
    if t == tag then return true end
  end
  return false
end

local function dprint(tag, ...)
  if not should_log(tag) then return end
  local args = {...}
  for i=1,#args do
    if type(args[i]) ~= "string" then args[i] = tostring(args[i]) end
  end
  io.stderr:write(tag .. ": " .. table.concat(args, " ") .. "\n")
end

local function dprintf(tag, fmt, ...)
  if not should_log(tag) then return end
  io.stderr:write(tag .. ": " .. string.format(fmt, ...) .. "\n")
end

-- Raw event debug (tag = "raw")
local function dprint_raw(evtype, code, val)
  dprintf("raw", "type=0x%02x code=0x%02x val=%d", evtype, code, val)
end

local function split_args(cmd)
  -- simple shlex-like splitter (handles quoted spaces with either quote type)
  -- returns a list of arguments without the surrounding quotes
  local args, i, len = {}, 1, #cmd
  while i <= len do
    -- skip whitespace
    while i <= len and cmd:sub(i,i):match("%s") do i = i + 1 end
    if i > len then break end
    local ch = cmd:sub(i,i)
    if ch == '"' or ch == "'" then
      -- quoted token
      local quote = ch
      local j = i + 1
      while j <= len do
        local c = cmd:sub(j,j)
        if c == '\\' and j < len then
          -- skip escaped char (including escaped quote)
          j = j + 2
        elseif c == quote then
          break
        else
          j = j + 1
        end
      end
      -- if we hit end without closing quote, log debug and take until end
      if j > len then
        dprint("parse", "unterminated quote in arg: %s", cmd)
        j = len
      end
      table.insert(args, cmd:sub(i+1, j-1))
      i = j + 1
    else
      -- unquoted token (handle simple backslash-escaped space)
      local buf = ""
      local j = i
      while j <= len and not cmd:sub(j,j):match("%s") do
        local c = cmd:sub(j,j)
        if c == '\\' and j < len and cmd:sub(j+1,j+1):match("%s") then
          -- escape space or other whitespace
          buf = buf .. cmd:sub(j+1,j+1)
          j = j + 2
        else
          buf = buf .. c
          j = j + 1
        end
      end
      table.insert(args, buf)
      i = j
    end
  end
  return args
end

local function ev_time_seconds(ev)
  return tonumber(ev.time.tv_sec) + tonumber(ev.time.tv_usec) / 1e6
end

-- ================== CONFIG =================-
local function boolish(v, default)
  if type(v) == "boolean" then return v end
  if type(v) == "string" then
    if v == "True" or v == "true" then return true end
    if v == "False" or v == "false" then return false end
  end
  return default
end

-- Native config parsing of relaxed gesture config syntax:
-- * single quotes for strings
-- * # comments to end-of-line (ignored when inside quotes)
-- * optional trailing commas before } or ]
-- * stringified booleans 'True'/'False'
-- We transform to strict JSON then decode with inlined json decoder.
local function read_file_all(path)
  local f = io.open(path, 'r'); if not f then return nil, 'cannot open '..path end
  local d = f:read('*a'); f:close(); return d
end

-- Direct relaxed-config parser producing Lua tables (no JSON intermediary)
local function parse_relaxed_config(src)
  local i, len = 1, #src
  local function peek() return src:sub(i,i) end
  local function nextc() local c = src:sub(i,i); i = i + 1; return c end
  local function skip_ws_and_comments()
    while i <= len do
      local c = peek()
      if c == '' then return end
      if c == '#' then
        while i <= len and peek() ~= '\n' do i = i + 1 end
      elseif c:match('%s') then
        i = i + 1
      else
        break
      end
    end
  end
  local function parse_string()
    local quote = nextc() -- ' or "
    local buf = {}
    while i <= len do
      local c = nextc()
      if c == '' then error("unterminated string") end
      if c == '\\' then
        local esc = nextc()
        if esc == 'n' then buf[#buf+1] = '\n'
        elseif esc == 'r' then buf[#buf+1] = '\r'
        elseif esc == 't' then buf[#buf+1] = '\t'
        elseif esc == '"' then buf[#buf+1] = '"'
        elseif esc == "'" then buf[#buf+1] = "'"
        elseif esc == '\\' then buf[#buf+1] = '\\'
        else buf[#buf+1] = esc end
      elseif c == quote then
        return table.concat(buf)
      else
        buf[#buf+1] = c
      end
    end
    error("unterminated string")
  end
  local function parse_number()
    local start = i
    while i <= len and src:sub(i,i):match('[%d%.eE+-]') do i = i + 1 end
    local numstr = src:sub(start, i-1)
    local n = tonumber(numstr)
    if n == nil then error("invalid number: "..numstr) end
    return n
  end
  local function parse_literal()
    if src:sub(i,i+3) == 'true' then i = i + 4; return true end
    if src:sub(i,i+4) == 'false' then i = i + 5; return false end
    if src:sub(i,i+3) == 'null' then i = i + 4; return nil end
    -- Allow capitalized string booleans; treat as raw strings so boolish() can handle later
    if src:sub(i,i+3) == 'True' then i = i + 4; return 'True' end
    if src:sub(i,i+4) == 'False' then i = i + 5; return 'False' end
    error('unexpected literal at '..i)
  end
  local parse_value
  local function parse_array()
    local arr = {}
    nextc() -- consume [
    skip_ws_and_comments()
    if peek() == ']' then nextc(); return arr end
    local idx = 1
    while true do
      arr[idx] = parse_value(); idx = idx + 1
      skip_ws_and_comments()
      local c = peek()
      if c == ']' then nextc(); break end
      if c == ',' then
        nextc(); skip_ws_and_comments()
        -- allow trailing comma before ]
        if peek() == ']' then nextc(); break end
      else
        error('expected , or ] at '..i)
      end
    end
    return arr
  end
  local function parse_object()
    local obj = {}
    nextc() -- consume {
    skip_ws_and_comments()
    if peek() == '}' then nextc(); return obj end
    while true do
      local c = peek()
      if c ~= '"' and c ~= "'" then error('expected string key at '..i) end
      local key = parse_string()
      skip_ws_and_comments()
      if nextc() ~= ':' then error('expected : after key at '..i) end
      skip_ws_and_comments()
      obj[key] = parse_value()
      skip_ws_and_comments()
      local d = peek()
      if d == '}' then nextc(); break end
      if d == ',' then
        nextc(); skip_ws_and_comments()
        -- allow trailing comma
        if peek() == '}' then nextc(); break end
      else
        error('expected , or } at '..i)
      end
    end
    return obj
  end
  parse_value = function()
    skip_ws_and_comments()
    local c = peek()
    if c == '"' or c == "'" then return parse_string()
    elseif c == '{' then return parse_object()
    elseif c == '[' then return parse_array()
    elseif c == '-' or c:match('%d') then return parse_number()
    elseif c:match('[A-Za-z]') then return parse_literal()
    else
      error('unexpected char '..c..' at '..i)
    end
  end
  local result = parse_value()
  skip_ws_and_comments()
  return result
end

local function native_load_config()
  local cfg_path = os.getenv('GESTURES_CONFIG') or ((os.getenv('HOME') or '')..'/.config/gestures.conf')
  local raw, err = read_file_all(cfg_path)
  if not raw then return nil, err end
  local ok, decoded = pcall(parse_relaxed_config, raw)
  if not ok then return nil, 'parse error: '..tostring(decoded) end
  return decoded
end

local function get_config_table()
  -- Try native first
  local tbl, err = native_load_config()
  if tbl then
    dprint('gesture', 'Config loaded natively.')
    return tbl
  end
  dprint('gesture', 'Native parse failed:', err or 'unknown')
  return {}
end

local function load_config_for_device(device_key)
  local all = get_config_table()
  if type(all) ~= 'table' then
    dprint('gesture','Config structure invalid, using built-in defaults')
    return {
      pinch_deadzone_enabled = false,
      swipe = { ["3"] = { l = { start = {"xdotool key alt+Left"} }, r = { start = {"xdotool key alt+Right"} } } },
      pinch = { ["2"] = { i = { start = {"echo pinch-in"} }, o = { start = {"echo pinch-out"} } } }
    }, false
  end
  local global_dead = boolish(all["pinch_deadzone_enabled"], false)
  local dev = all[device_key] or {}
  local dev_dead = boolish(dev["pinch_deadzone_enabled"], global_dead)
  dev["pinch_deadzone_enabled"] = dev_dead
  return dev, dev_dead
end

-- ================== GESTURE STATE ==================
---@class GestureType
---@field type string
---@field fingers string
---@field direction string
---@field event string
---@field update_direction string|nil

---@class GestureState
---@field type? GestureType
---@field acc ffi.cdata* -- wenk_acc_t[MAX_SLOTS]: per-slot {xcum, ycum, moved}
---@field sactive ffi.cdata* -- bool[MAX_SLOTS]: session slot membership (survives gesture resets)
---@field total ffi.cdata* -- wenk_total_t
---@field pinch boolean
---@field gesture_queue table
---@field rep_start number
---@field debounce number
---@field last_command_is_gesture_end boolean
---@field pinch_deadzone_enabled boolean
---@field started boolean
---@field max_fingers number
---@field nslots number

---@param pinched_deadzone_enabled boolean
---@return GestureState
local function newGestureState(pinched_deadzone_enabled)
  return {
    type = nil,                                -- table: {type, fingers, direction, event, update_direction}
    acc = ffi.new("wenk_acc_t[?]", MAX_SLOTS), -- per-slot movement accumulators
    sactive = ffi.new("bool[?]", MAX_SLOTS),   -- membership; preserved across resets like the old key set
    total = ffi.new("wenk_total_t"),
    pinch = true,                              -- still eligible to become pinch
    gesture_queue = {},                        -- list of argv arrays
    rep_start = 0,
    debounce = 0,
    last_command_is_gesture_end = false,
    pinch_deadzone_enabled = pinched_deadzone_enabled,
    started = false,
    max_fingers = 0,
    nslots = 0                                 -- maintained incrementally
  }
end

-- ============ Incremental max-pairwise-distance monitor ============
-- Exact delta of sqrt(max pairwise distance) between the committed snapshot
-- (updated slot at its last-processed coords ox,oy) and the after-snapshot
-- (slot at nx,ny). Tracks the argmax pair so a move NOT touching that pair is
-- O(n): only distances from the moved point can change. Falls back to an
-- exact O(n^2) rescan when the move touches the current pair or membership
-- changed (add/remove). Coordinates are read from last_x/last_y only.
local function mon_new()
  return { max2 = 0.0, pi = -1, pj = -1, valid = false }
end

local function mon_rebuild(mon, pts, n)
  local max2, pi, pj = 0.0, -1, -1
  local cnt = 0
  for i=0,MAX_SLOTS-1 do
    local a = pts[i]
    if a.active then
      cnt = cnt + 1
      local ax, ay = a.last_x, a.last_y
      for j=i+1,MAX_SLOTS-1 do
        local b = pts[j]
        if b.active then
          local dx = ax - b.last_x
          local dy = ay - b.last_y
          local d2 = dx*dx + dy*dy
          if d2 > max2 then max2, pi, pj = d2, i, j end
        end
      end
    end
  end
  mon.max2, mon.pi, mon.pj = max2, pi, pj
  mon.valid = n >= 2 and cnt >= 2
end

local function mon_move(mon, pts, nact, slot, nx, ny)
  if not mon.valid then return 0.0 end
  local before2 = mon.max2
  if mon.pi ~= slot and mon.pj ~= slot then
    -- Max pair survives this move: after-max2 = max(before2, best distance
    -- from the new position to any other committed point).
    local involve2, argj = 0.0, -1
    local seen = 0
    local limit = nact - 1
    for k=0,MAX_SLOTS-1 do
      if seen >= limit then break end
      local p = pts[k]
      if p.active and k ~= slot then
        local dx = nx - p.last_x
        local dy = ny - p.last_y
        local d2 = dx*dx + dy*dy
        if d2 > involve2 then involve2, argj = d2, k end
        seen = seen + 1
      end
    end
    if involve2 > before2 then
      mon.max2, mon.pi, mon.pj = involve2, slot, argj
      return math.sqrt(involve2) - math.sqrt(before2)
    end
    return 0.0 -- pair unchanged and no longer distance: provably zero
  end
  -- Move touches the tracked max pair: exact O(n^2) recompute.
  local after2, ai, aj = 0.0, -1, -1
  for i=0,MAX_SLOTS-1 do
    local a = pts[i]
    if a.active then
      local ax, ay = a.last_x, a.last_y
      if i == slot then ax, ay = nx, ny end
      for j=i+1,MAX_SLOTS-1 do
        local b = pts[j]
        if b.active then
          local bx, by = b.last_x, b.last_y
          if j == slot then bx, by = nx, ny end
          local dx = ax - bx
          local dy = ay - by
          local d2 = dx*dx + dy*dy
          if d2 > after2 then after2, ai, aj = d2, i, j end
        end
      end
    end
  end
  mon.max2, mon.pi, mon.pj = after2, ai, aj
  return math.sqrt(after2) - math.sqrt(before2)
end

local function out_of_deadzone(gs)
  local all_out = true
  local sactive, acc = gs.sactive, gs.acc
  for slot=0,MAX_SLOTS-1 do
    if sactive[slot] then
      local sl = acc[slot]
      local mv = sl.xcum*sl.xcum + sl.ycum*sl.ycum
      dprint("slot", "slot", slot, "mv", tostring(mv))
      all_out = all_out and (mv > DEADZONE_SQUARED)
    end
  end
  return all_out
end

-- ================== COMMAND QUEUE ==================
local function enqueue(gs, dev_conf, evsec)
  local g = gs.type
  if not g then return end
  -- Type-guarded config walk (same semantics as the old pcall'd closure,
  -- without allocating a closure per gesture event).
  local node = dev_conf[g.type]
  if type(node) ~= "table" then node = nil end
  if node then
    node = node[g.fingers]
    if type(node) ~= "table" then node = nil end
  end
  local target
  if node then
    node = node[g.direction]
    if type(node) == "table" then
      if g.direction == "t" then
        target = node
      elseif g.event == "update" then
        local ev_tbl = node[g.event]
        if type(ev_tbl) == "table" then target = ev_tbl[g.update_direction] end
      else
        target = node[g.event]
      end
    end
  end
  if not target then
    dprint("gesture", "Gesture recognized but not configured.")
    return
  end
  for _,cmd in ipairs(target) do
    if type(cmd) == 'string' then
      local trimmed = cmd:match('^%s*(.-)%s*$')
      if trimmed ~= '' then
        local argv = split_args(trimmed)
        if #argv > 0 and argv[1] ~= '' then
          gs.gesture_queue[#gs.gesture_queue+1] = argv
        end
      end
    end
  end
  if evsec then gs.debounce = evsec end
end

local function reap_children_nonblocking()
  while true do
    local pid = ffi.C.waitpid(-1, nil, WNOHANG)
    if pid <= 0 then break end
  end
end

local function exec_queue(gs, async)
  if async then
    reap_children_nonblocking()
  end

  if #gs.gesture_queue == 0 then return end
  for _,argv in ipairs(gs.gesture_queue) do
    if not STOP then
      local pid = ffi.C.fork()
      if pid == 0 then
        -- child: build argv vector with stable buffers
        local argc = #argv
        local cargv = ffi.new("char *[?]", argc + 1)
        for i,a in ipairs(argv) do
          local buf = ffi.new("char[?]", #a+1)
          ffi.copy(buf, a)
          cargv[i-1] = buf
        end
        cargv[argc] = nil
        ffi.C.execvp(argv[1], cargv)
        os.exit(1)
      else
        if not async then
          ffi.C.waitpid(pid, nil, 0)
        end
      end
    end
  end
  if async then
    reap_children_nonblocking()
  end
  gs.gesture_queue = {}
end

local function getRep(gs, dev_conf, default)
  local g = gs.type
  local node = dev_conf[g.type]
  if type(node) ~= "table" then return default end
  node = node[g.fingers]
  if type(node) ~= "table" then return default end
  node = node[g.direction]
  if type(node) ~= "table" then return default end
  local rep = node["rep"]
  if type(rep) == "number" then return rep end
  return default
end

-- ================== CLASSIFICATION ==================
local function classify_after_move(gs, dev_conf, evsec)
  local total = gs.total
  local no_slots = gs.nslots

  -- Moved?
  -- pinch start
  if (not gs.type) and gs.pinch and total.moved >= 1 and no_slots == 2 then
    local x_cum = math.abs(total.xcum)
    local y_cum = math.abs(total.ycum)
    local dis_cum = total.discum
    dprintf("gesture", "dis_cum=%.0f x_cum+y_cum=%.0f", dis_cum, x_cum + y_cum)
    if x_cum + y_cum > PINCH_DECISION then
      gs.pinch = false
    end
    if math.abs(dis_cum) > PINCH_THRESHOLD and ((not gs.pinch_deadzone_enabled) or out_of_deadzone(gs)) then
      gs.type = { type="pinch", fingers=tostring(no_slots), event="start", direction = (dis_cum > 0 and 'i' or 'o') }
      enqueue(gs, dev_conf, evsec)
      gs.rep_start = evsec
      -- reset cumulators after triggering start
      total.xcum, total.ycum, total.discum = 0.0,0.0,0.0
      return
    end
  end

  -- swipe start
  if (not gs.type) and no_slots >= 3 and total.moved == no_slots then
    local x_cum = math.abs(total.xcum)
    local y_cum = math.abs(total.ycum)
    if (x_cum*x_cum + y_cum*y_cum) > ((no_slots * DECISION) ^ 2) then
      dprintf("angle", "x_cum=%.0f y_cum=%.0f", x_cum, y_cum)
      local dir
      if y_cum <= x_cum * TANX then
        dir = (total.xcum <= 0) and "l" or "r"
      elseif y_cum >= x_cum * TANY then
        dir = (total.ycum <= 0) and "u" or "d"
      else
        local x = total.xcum; local y = total.ycum
        if x*y > 0 then
          dir = (x <=0 and y <0) and "lu" or "rd"
        else
          dir = (x >=0 and y <0) and "ru" or "ld"
        end
      end
      gs.type = { type="swipe", fingers=tostring(no_slots), event="start", direction=dir }
      enqueue(gs, dev_conf, evsec)
      gs.rep_start = evsec
      total.xcum, total.ycum, total.discum = 0.0,0.0,0.0
    end
  end

  -- updates (repeat)
  if gs.type then
    if (evsec - gs.rep_start) < REP_THRES then
      total.xcum, total.ycum, total.discum = 0.0,0.0,0.0
      return
    end
    gs.type.event = "update"
    if gs.type.type == "pinch" then
      local rep = getRep(gs, dev_conf, PINCH_REP_DEF)
      if math.abs(total.discum) > rep then
        gs.type.update_direction = (total.discum > 0) and 'i' or 'o'
        enqueue(gs, dev_conf, evsec)
        total.discum = 0
      end
    else
      -- swipe
      local dir = gs.type.direction
      if dir == 'l' or dir == 'r' or dir == 'u' or dir == 'd' then
        local rep = getRep(gs, dev_conf, REP_DEF)
        if total.xcum >= rep then
          gs.type.update_direction = 'r'; enqueue(gs, dev_conf, evsec); total.xcum = 0
        elseif total.xcum <= -rep then
          gs.type.update_direction = 'l'; enqueue(gs, dev_conf, evsec); total.xcum = 0
        elseif total.ycum >= rep then
          gs.type.update_direction = 'd'; enqueue(gs, dev_conf, evsec); total.ycum = 0
        elseif total.ycum <= -rep then
          gs.type.update_direction = 'u'; enqueue(gs, dev_conf, evsec); total.ycum = 0
        end
      else
        -- diagonals: lu, rd, ld, ru
        local rep = getRep(gs, dev_conf, REP_DAG_DEF)
        if (total.xcum + total.ycum) >= rep then
          gs.type.update_direction = 'rd'; enqueue(gs, dev_conf, evsec); total.xcum, total.ycum = 0,0
        elseif (total.xcum + total.ycum) <= -rep then
          gs.type.update_direction = 'lu'; enqueue(gs, dev_conf, evsec); total.xcum, total.ycum = 0,0
        elseif (total.xcum - total.ycum) >= rep then
          gs.type.update_direction = 'ru'; enqueue(gs, dev_conf, evsec); total.xcum, total.ycum = 0,0
        elseif (total.xcum - total.ycum) <= -rep then
          gs.type.update_direction = 'ld'; enqueue(gs, dev_conf, evsec); total.xcum, total.ycum = 0,0
        end
      end
    end
  end
end

-- ================== DEVICE LOOP ==================
local function create_device(devpath, calibration, device_key)
  local dev_conf, pinch_deadzone_enabled = load_config_for_device(device_key)
  local fd = ffi.C.open(devpath, bit.bor(O_RDONLY, O_NONBLOCK))
  if fd < 0 then
    io.stderr:write("Failed to open "..devpath.."\n")
    return nil
  end

  -- Orientation (safe)
  local orientation = ""
  do
    local p = io.popen("orientation 2>/dev/null")
    if p then
      local line = p:read("*l")
      p:close()
      orientation = tostring(line or ""):gsub("%s+$","")
    else
      orientation = ""
    end
  end
  local orientation_y = ((orientation == "inverted") or (orientation == "right")) and -1 or 1
  local orientation_x = ((orientation == "inverted") or (orientation == "left")) and -1 or 1
  local swap_x_y = (orientation == "right") or (orientation == "left")
  local inv_calibration = 1 / calibration
  local scale_x = orientation_x * inv_calibration
  local scale_y = orientation_y * inv_calibration
  dprint("gesture", "orientation:", orientation, "swap:", tostring(swap_x_y), "ox:", tostring(orientation_x), "oy:", tostring(orientation_y))

  local ev_size = ffi.sizeof("struct input_event")
  local pts = ffi.new("wenk_point_t[?]", MAX_SLOTS) -- slot -> staged/committed coords
  local n_status = 0   -- maintained incrementally
  local mon = mon_new()
  local current_slot = 0
  local gs = newGestureState(pinch_deadzone_enabled)
  gs.debounce = 0
  local has_mt_slot = false -- detect if device emits ABS_MT_SLOT; else fallback to synthetic slot 0
  local typeA = { mode=false, current_tid=nil, tid_to_slot={}, slot_to_tid={}, next_slot=0 }
  local warned_overflow = false

  local function ensure_slot_exists(slot)
    local st = pts[slot]
    local new_created = false
    if not st.active then
      st.x, st.y = 0.0, 0.0
      st.last_x, st.last_y = 0.0, 0.0
      st.x_updated, st.y_updated = false, false
      st.first = true
      st.active = true
      n_status = n_status + 1
      new_created = true
      -- point set changed: the tracked max pair may be stale
      mon_rebuild(mon, pts, n_status)
    end
    if not gs.sactive[slot] then
      gs.sactive[slot] = true
      gs.nslots = gs.nslots + 1
    end
    if new_created and gs.max_fingers < n_status then
      gs.max_fingers = n_status
    end
  end

  local function remove_point(slot)
    -- Drop a finger from both the coordinate set and the gesture session.
    local st = pts[slot]
    if st.active then
      st.active = false
      st.x_updated, st.y_updated = false, false
      n_status = n_status - 1
      mon_rebuild(mon, pts, n_status)
    end
    if gs.sactive[slot] then
      gs.sactive[slot] = false
      gs.nslots = gs.nslots - 1
    end
  end

  local function finger_start(evsec, slot)
    -- Match Python semantics: every new finger start resets accumulators,
    -- preserving the slot membership set (memset-speed reset).
    gs.last_command_is_gesture_end = false
    gs.debounce = evsec
    gs.type = nil
    ffi.fill(gs.acc, ffi.sizeof("wenk_acc_t") * MAX_SLOTS, 0)
    local ttl = gs.total
    ttl.xcum, ttl.ycum, ttl.discum, ttl.moved = 0.0,0.0,0.0,0
    if not gs.sactive[slot] then
      gs.sactive[slot] = true
      gs.nslots = gs.nslots + 1
    end
    -- Initialize last processed coordinates to current sample. This mutates
    -- committed geometry, so the max-distance monitor must be rebuilt.
    local s = pts[slot]
    s.last_x, s.last_y = s.x, s.y
    mon_rebuild(mon, pts, n_status)
  end

  local function process_update(slot, nx, ny, evsec)
    local sl = gs.acc[slot]
    local prev = pts[slot]
    local dx = nx - prev.last_x
    local dy = ny - prev.last_y
    sl.xcum = sl.xcum + dx
    sl.ycum = sl.ycum + dy
    local ttl = gs.total
    ttl.xcum = ttl.xcum + dx
    ttl.ycum = ttl.ycum + dy

    -- Delta is measured against last-processed coords (prev.last_*), NOT the
    -- staged prev.x/.y (the EV_ABS handler overwrites those before we are
    -- called). The monitor tracks committed coords and is updated in-place;
    -- if this slot did not move, the committed geometry is unchanged and the
    -- delta is provably zero (skip the scan entirely).
    local distance_delta = 0.0
    if dx ~= 0 or dy ~= 0 then
      distance_delta = mon_move(mon, pts, n_status, slot, nx, ny)
    end
    prev.last_x = nx
    prev.last_y = ny
    ttl.discum = ttl.discum + distance_delta

    if sl.moved == 0 then
      if (sl.xcum*sl.xcum + sl.ycum*sl.ycum) > THRESHOLD_SQUARED then
        sl.moved = 1
        ttl.moved = ttl.moved + 1
      end
    end
    dprintf("gesture", "total x=%.0f y=%.0f dis=%.0f moved=%d", ttl.xcum, ttl.ycum, ttl.discum, ttl.moved)
    classify_after_move(gs, dev_conf, evsec)
  end

  local function gesture_end(evsec)
    local no_slots = gs.nslots
    dprint("gesture", "gesture_end slots="..tostring(no_slots))
    if (evsec - gs.debounce) >= DEBOUNCE then
      if gs.type then
        gs.type.event = "end"
        enqueue(gs, dev_conf, evsec)
      else
        -- Tap fallback based on max fingers during gesture session
        local fingers = gs.max_fingers
        if fingers >= 3 then
          gs.type = { type="swipe", fingers=tostring(fingers), direction="t" }
          enqueue(gs, dev_conf, evsec)
        end
      end
    end
    -- Exec queue now; async if 5 fingers
    exec_queue(gs, gs.max_fingers == 5)

    -- reset params (membership preserved; memset-speed accumulator reset)
    gs.gesture_queue = {}
    gs.pinch = true
    gs.debounce = evsec
    gs.started = false
    gs.max_fingers = 0
    gs.type = nil
    ffi.fill(gs.acc, ffi.sizeof("wenk_acc_t") * MAX_SLOTS, 0)
    local ttl = gs.total
    ttl.xcum, ttl.ycum, ttl.discum, ttl.moved = 0.0,0.0,0.0,0
  end

  local function flush_if_debounced(nowsec)
    if (#gs.gesture_queue ~= 0) and ((nowsec - gs.debounce) >= DEBOUNCE) then
      exec_queue(gs, false)
    end
  end

  local function allocate_slot()
    -- Find the lowest free non-negative slot index; consider both coordinate
    -- points and session membership to avoid collisions.
    for i=0,MAX_SLOTS-1 do
      if not (pts[i].active or gs.sactive[i]) then return i end
    end
    return nil
  end

  local function ensure_typeA_slot_for_tid(tid)
    local slot = typeA.tid_to_slot[tid]
    if slot == nil then
      slot = allocate_slot()
      if slot == nil then
        if not warned_overflow then
          warned_overflow = true
          io.stderr:write("Type A contact overflow (>"..MAX_SLOTS.." slots); ignoring extra contacts\n")
        end
        return nil
      end
      typeA.tid_to_slot[tid] = slot
      typeA.slot_to_tid[slot] = tid
    end
    return slot
  end

  local function end_typeA_tid(tid, evsec)
    local slot = typeA.tid_to_slot[tid]
    if slot ~= nil and slot < MAX_SLOTS then
      -- finalize any pending first/update before removal
      local s = pts[slot]
      if s.active and (s.x_updated or s.y_updated) then
        if s.first then s.first = false; finger_start(evsec, slot) else process_update(slot, s.x, s.y, evsec) end
      end
      remove_point(slot)
      typeA.slot_to_tid[slot] = nil
      typeA.tid_to_slot[tid] = nil
    elseif slot == nil then
      typeA.tid_to_slot[tid] = nil
    else
      typeA.slot_to_tid[slot] = nil
      typeA.tid_to_slot[tid] = nil
    end
  end

  local function finalize_slot(slot, evsec)
    local s = pts[slot]
    if not s.active then return end
    if s.first then
      s.first = false
      finger_start(evsec, slot)
    else
      process_update(slot, s.x, s.y, evsec)
    end
    s.x_updated, s.y_updated = false, false
  end

  local dead = false   -- set by read_available on EOF/fatal error
  local closed = false

  -- Batched input: one read() drains up to EVBUF_N events instead of paying a
  -- syscall per event. A partial trailing event (defensive only; evdev returns
  -- whole events when the buffer fits at least one) is moved to the buffer
  -- front and completed by the next read.
  local EVBUF_N = 64
  local evbuf = ffi.new("struct input_event[?]", EVBUF_N)
  local buf_bytes = ev_size * EVBUF_N
  local buf_base = ffi.cast("char*", evbuf)

  local function handle_event(e)
    local evsec = ev_time_seconds(e)

    if e.type == EV_SYN then
      if e.code == SYN_MT_REPORT then
        -- Type A devices: per-contact separator
        if typeA.mode and typeA.current_tid ~= nil then
          local slot = typeA.tid_to_slot[typeA.current_tid]
          if slot ~= nil and slot < MAX_SLOTS then
            -- finalize pending updates for this contact even if only one axis changed
            local s = pts[slot]
            if s.active and (s.x_updated or s.y_updated) then
              finalize_slot(slot, evsec)
            end
          end
          -- clear current finger context
          typeA.current_tid = nil
        end
      elseif e.code == SYN_REPORT then
        -- End of frame: finalize any pending partial updates (scan active
        -- points; early-exit once every tracked point was visited)
        if n_status > 0 then
          local seen = 0
          for slot=0,MAX_SLOTS-1 do
            local s = pts[slot]
            if s.active then
              seen = seen + 1
              if s.x_updated or s.y_updated then
                finalize_slot(slot, evsec)
              end
              if seen >= n_status then break end
            end
          end
        end
        flush_if_debounced(evsec)
      else
        -- Other SYN codes: still consider flushing debounced queue
        flush_if_debounced(evsec)
      end
    elseif e.type == EV_ABS then
      local code = e.code
      local val = e.value
      dprint_raw(e.type, code, val)
      if code == ABS_MT_SLOT then
        if val < MAX_SLOTS then
          current_slot = val
          ensure_slot_exists(current_slot)
          has_mt_slot = true
        elseif not warned_overflow then
          warned_overflow = true
          io.stderr:write("MT slot id "..val.." exceeds "..MAX_SLOTS.." slots; ignoring\n")
        end
      elseif code == ABS_MT_TRACKING_ID then
        local is_minus_one = (val == 0xFFFFFFFF) or (bit.tobit(val) == -1)
        if is_minus_one then
          -- finger removed
          if not gs.last_command_is_gesture_end then
            gs.last_command_is_gesture_end = true
            gesture_end(evsec)
          end
          if has_mt_slot then
            remove_point(current_slot)
          else
            typeA.mode = true
            if typeA.current_tid ~= nil then
              end_typeA_tid(typeA.current_tid, evsec)
              typeA.current_tid = nil
            end
          end
        else
          if has_mt_slot then
            ensure_slot_exists(current_slot)
            local s = pts[current_slot]
            s.first = true
            s.x_updated = false
            s.y_updated = false
          else
            typeA.mode = true
            typeA.current_tid = val
            local slot = ensure_typeA_slot_for_tid(val)
            if slot ~= nil then
              current_slot = slot
              ensure_slot_exists(current_slot)
              local s = pts[current_slot]
              s.first = true
              s.x_updated = false
              s.y_updated = false
            end
          end
        end
      elseif code == ABS_MT_POSITION_X then
        if not has_mt_slot then
          typeA.mode = true
          if typeA.current_tid ~= nil then
            local slot = ensure_typeA_slot_for_tid(typeA.current_tid)
            if slot ~= nil then current_slot = slot end
          else
            -- if no current TID yet, use synthetic slot 0 for now
            current_slot = 0
          end
        end
        if not pts[current_slot].active then ensure_slot_exists(current_slot) end
        local s = pts[current_slot]
        if swap_x_y then
          s.y = val * scale_y
          s.y_updated = true
        else
          s.x = val * scale_x
          s.x_updated = true
        end
      elseif code == ABS_MT_POSITION_Y then
        if not has_mt_slot then
          typeA.mode = true
          if typeA.current_tid ~= nil then
            local slot = ensure_typeA_slot_for_tid(typeA.current_tid)
            if slot ~= nil then current_slot = slot end
          else
            current_slot = 0
          end
        end
        if not pts[current_slot].active then ensure_slot_exists(current_slot) end
        local s = pts[current_slot]
        if swap_x_y then
          s.x = val * scale_x
          s.x_updated = true
        else
          s.y = val * scale_y
          s.y_updated = true
        end
      elseif code == ABS_X or code == ABS_Y then
        -- Single-touch fallback mapped to slot 0, only if device lacks MT slots
        if not has_mt_slot then
          current_slot = 0
          ensure_slot_exists(current_slot)
          local s = pts[current_slot]
          if code == ABS_X then
            if swap_x_y then
              s.y = val * scale_y; s.y_updated = true
            else
              s.x = val * scale_x; s.x_updated = true
            end
          else -- ABS_Y
            if swap_x_y then
              s.x = val * scale_x; s.x_updated = true
            else
              s.y = val * scale_y; s.y_updated = true
            end
          end
        end
      end

      local s = pts[current_slot]
      if s.active and s.x_updated and s.y_updated then
        if s.first then
          s.first = false
          finger_start(evsec, current_slot)
        else
          process_update(current_slot, s.x, s.y, evsec)
        end
        s.x_updated, s.y_updated = false, false
      end
    end
  end

  local dev = {
    fd = fd,
    path = devpath,
    kind = device_key,
    calibration = calibration,
    is_dead = function() return dead end,
    read_available = function()
      local fill = 0 -- valid bytes at buffer front (a carried partial event)
      while true do
        local r = tonumber(ffi.C.read(fd, buf_base + fill, buf_bytes - fill))
        if r < 0 then
          local err = ffi.errno()
          if err == EINTR then
            -- interrupted syscall; retry with unchanged fill
          elseif err == EAGAIN then
            break -- drained; normal for O_NONBLOCK
          else
            dead = true -- fatal (ENODEV etc.): signal removal
            break
          end
        elseif r == 0 then
          dead = true -- EOF: device gone
          break
        else
          local total = fill + r
          local n = total - (total % ev_size)
          local nev = math.floor(total / ev_size)
          for k = 0, nev - 1 do
            handle_event(evbuf[k])
          end
          fill = total - n
          if fill > 0 then
            ffi.copy(buf_base, buf_base + n, fill)
          end
        end
      end
    end,
    close = function()
      if not closed and fd >= 0 then
        closed = true
        ffi.C.close(fd)
      end
    end
  }

  return dev
end

-- ================== DISCOVERY (simplified) ==================
local function discover()
  -- Simple discovery: touch devices with "event*" handler and "Touch" in name
  local f = io.open("/proc/bus/input/devices","r")
  if not f then return {} end
  local content = f:read("*a"); f:close()
  local devices = {}
  for block in content:gmatch("(.-)\n\n") do
    local handlers = block:match("Handlers=([^\n]+)")
    local name = block:match('N: Name="([^"]+)"') or "?"
    if handlers and handlers:find("event") and (block:find("Touch") or block:lower():find("touch")) then
      local event = handlers:match("(event%d+)")
      if event then
        -- crude heuristic: treat touchpad vs touchscreen by name
        local kind = (name:lower():find("touchscreen") or name:lower():find("ts")) and "touchscreen" or "touchpad"
        table.insert(devices, { path="/dev/input/"..event, kind=kind })
      end
    end
  end
  return devices
end

-- ================== MAIN ==================
local function parse_args(argv)
  for _, a in ipairs(argv) do
    if a == "debug" then DEBUG = true
    elseif a == "stop" then STOP = true
    elseif a == "benchmark" then BENCHMARK = true
    elseif a:match("^bench=%d+$") then
      BENCHMARK = true
      BENCH_ITERS = tonumber(a:match("^bench=(%d+)$")) or BENCH_ITERS
    elseif a == "angle" or a == "slot" or a == "gesture" or a == "raw" then
      TYPE_FILTER[#TYPE_FILTER+1] = a
    end
  end
  -- Freeze debug helpers after option parsing: with logging disabled, rebind
  -- to no-ops so hot-path call sites never run the filter loop, vararg
  -- packing or string formatting.
  if not DEBUG then
    dprint = function() end
    dprintf = function() end
    dprint_raw = function() end
  end
end

local function run_benchmark()
  -- Self-contained: legacy full scans (as they existed pre-optimization) vs
  -- the incremental FFI monitor used by process_update. Both compute the
  -- identical delta sequence, so their sums double as a correctness check.
  local function legacy_max(status)
    local max2 = 0.0
    for i, a in pairs(status) do
      for j, b in pairs(status) do
        if i < j then
          local dx, dy = a.x - b.x, a.y - b.y
          local d2 = dx*dx + dy*dy
          if d2 > max2 then max2 = d2 end
        end
      end
    end
    return math.sqrt(max2)
  end

  local iters = BENCH_ITERS
  local status_old = {
    [0] = {x=10, y=10},
    [1] = {x=200, y=50},
    [2] = {x=450, y=600},
    [3] = {x=800, y=300},
    [4] = {x=1000, y=900},
  }
  local pts = ffi.new("wenk_point_t[?]", MAX_SLOTS)
  local init = { {10,10}, {200,50}, {450,600}, {800,300}, {1000,900} }
  for k=0,4 do
    local p = pts[k]
    p.active = true
    p.x, p.y = init[k+1][1], init[k+1][2]
    p.last_x, p.last_y = p.x, p.y
  end
  local mon = mon_new()
  mon_rebuild(mon, pts, 5)
  local sum_old, sum_new = 0, 0

  local t0 = os.clock()
  for i=1,iters do
    local nx = (i * 17) % 1024
    local ny = (i * 31) % 1024
    local before = legacy_max(status_old)
    status_old[1].x, status_old[1].y = nx, ny
    local after = legacy_max(status_old)
    sum_old = sum_old + (after - before)
  end
  local old_secs = os.clock() - t0

  local t1 = os.clock()
  for i=1,iters do
    local nx = (i * 17) % 1024
    local ny = (i * 31) % 1024
    local s1 = pts[1]
    local delta = mon_move(mon, pts, 5, 1, nx, ny)
    s1.last_x, s1.last_y = nx, ny
    sum_new = sum_new + delta
  end
  local new_secs = os.clock() - t1

  local old_ops = (old_secs > 0) and (iters / old_secs) or 0
  local new_ops = (new_secs > 0) and (iters / new_secs) or 0
  local speedup = (new_secs > 0) and (old_secs / new_secs) or 0

  io.stderr:write(string.format("Benchmark iterations: %d\n", iters))
  io.stderr:write(string.format("legacy scan x2: %.6fs (%.0f ops/s) sum=%.3f\n", old_secs, old_ops, sum_old))
  io.stderr:write(string.format("monitor delta: %.6fs (%.0f ops/s) sum=%.3f\n", new_secs, new_ops, sum_new))
  io.stderr:write(string.format("speedup: %.2fx (sums %s)\n", speedup,
    math.abs(sum_old - sum_new) < 1e-3 and "match" or string.format("MISMATCH %.3f vs %.3f", sum_old, sum_new)))
end

local function main(argv)
  parse_args(argv or {})
  if BENCHMARK then
    run_benchmark()
    return
  end
  -- Coroutine-driven device processors polled in one loop, with hotplug
  -- support: dead devices are dropped and discovery reruns periodically.
  local processors = {}
  local processor_count = 0
  local pollfds = ffi.new("struct pollfd[?]", MAX_DEVICES)

  local function safe_call(fn, ...)
    local ok, err = pcall(fn, ...)
    if not ok then
      io.stderr:write("Processor failed: "..tostring(err).."\n")
      RUNNING = false
      return false
    end
    return true
  end

  local function find_processor_by_path(path)
    for i=1,processor_count do
      if processors[i].path == path then return i end
    end
    return nil
  end

  local open_failed = {} -- path -> os.time() of last failed open

  local function add_device(info)
    if processor_count >= MAX_DEVICES then
      io.stderr:write("Device limit reached, ignoring "..info.path.."\n")
      return false
    end
    if find_processor_by_path(info.path) then return false end
    local calibration = (info.kind == "touchscreen") and TOUCHSCREEN_CALIBRATION or TOUCHPAD_CALIBRATION
    local dev = create_device(info.path, calibration, info.kind)
    if not dev then
      open_failed[info.path] = os.time()
      return false
    end
    open_failed[info.path] = nil
    processor_count = processor_count + 1
    processors[processor_count] = dev
    pollfds[processor_count-1].fd = dev.fd
    pollfds[processor_count-1].events = POLLIN
    pollfds[processor_count-1].revents = 0
    io.stderr:write(string.format("Using %s (%s)\n", info.path, info.kind))
    return true
  end

  local function remove_processor(i)
    local dev = processors[i]
    io.stderr:write("Device lost: "..dev.path.."\n")
    dev.close()
    -- swap-with-last compaction; order is irrelevant for poll()
    local last = processor_count
    if i ~= last then
      processors[i] = processors[last]
      pollfds[i-1] = pollfds[last-1]
    end
    processors[last] = nil
    pollfds[last-1].fd = -1
    pollfds[last-1].revents = 0
    processor_count = last - 1
  end

  local function rescan()
    local seen = {}
    local now = os.time()
    for _,info in ipairs(discover()) do
      local cooling = open_failed[info.path] and (now - open_failed[info.path]) < OPEN_RETRY_S
      if not seen[info.path] and not find_processor_by_path(info.path) and not cooling then
        seen[info.path] = true
        add_device(info)
      end
    end
  end

  rescan()
  if processor_count == 0 then
    io.stderr:write("No touch devices yet, waiting...\n")
  end

  while RUNNING do
    local pret
    if processor_count == 0 then
      -- nothing to poll; idle until next rescan window
      ffi.C.usleep(RESCAN_MS * 1000)
      pret = 0
    else
      pret = ffi.C.poll(pollfds, processor_count, RESCAN_MS)
    end
    local need_rescan = false
    if pret < 0 then
      local err = ffi.errno()
      if err == EINTR then
        -- interrupted by signal; just loop again
      else
        io.stderr:write("poll failed: "..tostring(ffi.string(ffi.C.strerror(err))).."\n")
        break
      end
    elseif pret == 0 then
      need_rescan = true
    else
      local i = 1
      while i <= processor_count do
        local rev = pollfds[i-1].revents
        if bit.band(rev, bit.bor(POLLERR, POLLHUP, POLLNVAL)) ~= 0 or processors[i].is_dead() then
          remove_processor(i)
          need_rescan = true
          -- index stays put so a swapped-in entry gets inspected too
        elseif bit.band(rev, POLLIN) ~= 0 then
          if not safe_call(processors[i].read_available) then break end
          i = i + 1
        else
          i = i + 1
        end
      end
    end
    if need_rescan and RUNNING then
      rescan()
    end
  end
  -- shutdown
  for i=1,processor_count do
    local dev = processors[i]
    dev.close()
    processors[i] = nil
  end
end

if pcall(debug.getlocal, 4, 1) == false then
  main(arg)
end
