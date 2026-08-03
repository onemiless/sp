#!/usr/bin/env python3
import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.selfdrive.car.tesla_turn_signal_controller import VALIDATION_LOG_PATH
from openpilot.selfdrive.debug.tesla_turn_signal_test import (
  cancel_validation_session,
  get_validation_status,
  start_validation_session,
)
from openpilot.selfdrive.debug.device_settings import settings_snapshot, validate_and_write
from openpilot.selfdrive.debug.device_hotspot import hotspot_status, set_hotspot_enabled
from openpilot.selfdrive.debug.device_terminal import change_password, run_command, terminal_status
from openpilot.selfdrive.debug.driving_status import driving_status_snapshot


HOST = "0.0.0.0"
PORT = 8088
_SESSION_LOCK = threading.Lock()
_ACTIVE_WEB_TEST_ID: str | None = None
_ACTIVE_WEB_SESSION_STARTED = 0.0
WEB_SESSION_TIMEOUT_S = 20.0


def _clear_active_session(test_id: str) -> None:
  global _ACTIVE_WEB_SESSION_STARTED, _ACTIVE_WEB_TEST_ID
  with _SESSION_LOCK:
    if _ACTIVE_WEB_TEST_ID == test_id:
      _ACTIVE_WEB_TEST_ID = None
      _ACTIVE_WEB_SESSION_STARTED = 0.0


def render_page() -> bytes:
  return """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
  <title>车载设置</title>
  <style>
    :root { color-scheme:dark; font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
    body { margin:0; background:#111827; color:#f9fafb; } main { max-width:840px; margin:auto; padding:18px 14px 42px; }
    h1 { font-size:24px; margin:4px 0; } p { color:#cbd5e1; line-height:1.45; } .tabs { display:flex; gap:8px; margin:18px 0; }
    .tab, button { border:0; border-radius:12px; font-weight:700; color:white; background:#334155; padding:12px 16px; font-size:16px; }
    .tab.active { background:#2563eb; } .notice { padding:11px 13px; border-radius:10px; margin:12px 0; background:#14532d; color:#dcfce7; }
    .notice.onroad { background:#7c2d12; color:#ffedd5; } .group { margin:22px 0 9px; color:#93c5fd; font-size:15px; }
    .category-nav { display:flex; gap:8px; overflow-x:auto; padding:4px 0 10px; position:sticky; top:0; background:#111827; z-index:1; } .category { white-space:nowrap; padding:9px 12px; font-size:14px; }
    .category.active { background:#2563eb; }
    .card { display:grid; grid-template-columns:1fr auto; gap:10px; align-items:center; background:#1e293b; border-radius:13px; padding:14px; margin:8px 0; }
    .card h2 { font-size:16px; margin:0 0 5px; } .card p { font-size:13px; margin:0; } .lock { color:#fbbf24; font-size:12px; }
    input[type=number], select { width:120px; padding:9px; border:1px solid #475569; border-radius:9px; background:#0f172a; color:white; font-size:16px; }
    input[type=checkbox] { width:28px; height:28px; accent-color:#2563eb; } input:disabled, select:disabled, button:disabled { opacity:.45; }
    #turn-panel { text-align:center; } .buttons { display:grid; grid-template-columns:1fr 1fr; gap:16px; margin-top:28px; }
    .turn { min-height:120px; font-size:26px; } #left { background:#2563eb; } #right { background:#ea580c; } #cancel { display:none; width:100%; min-height:64px; margin-top:16px; background:#dc2626; }
    #status { min-height:52px; margin-top:18px; color:#fde68a; white-space:pre-wrap; }
    textarea { width:100%; min-height:130px; box-sizing:border-box; padding:11px; border:1px solid #475569; border-radius:10px; background:#020617; color:#e2e8f0; font:14px ui-monospace,monospace; }
    #terminal-output { min-height:100px; max-height:420px; overflow:auto; text-align:left; white-space:pre-wrap; background:#020617; border-radius:10px; padding:12px; color:#cbd5e1; }
    .terminal-row { display:flex; gap:8px; margin:10px 0; } .terminal-row input { min-width:0; flex:1; padding:9px; border:1px solid #475569; border-radius:9px; background:#0f172a; color:white; }
    .drive-alert { white-space:pre-wrap; } #driving-canvas { display:block; width:100%; height:min(72vh,640px); margin:12px 0; border-radius:13px; background:linear-gradient(#172554,#020617); }
    .can-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:9px; margin-top:10px; } .can-detail { background:#1e293b; border-radius:11px; padding:11px; color:#cbd5e1; font-size:12px; line-height:1.55; } .can-detail strong { display:block; color:#93c5fd; font-size:14px; margin-bottom:3px; } .can-detail .ok { color:#86efac; } .can-detail.warn { color:#fbbf24; }
  </style>
</head><body><main>
  <h1>车载设置</h1><p>通过手机或电脑访问此页面。行驶中仅允许修改实时生效的白名单设置。</p>
  <div class="tabs"><button class="tab active" id="settings-tab" onclick="showPanel('settings')">设置</button><button class="tab" id="driving-tab" onclick="showPanel('driving')">行驶信息</button><button class="tab" id="turn-tab" onclick="showPanel('turn')">转向测试</button><button class="tab" id="terminal-tab" onclick="showPanel('terminal')">终端</button></div>
  <section id="settings-panel"><div id="mode" class="notice">正在读取设置…</div><div id="category-nav" class="category-nav"></div><div id="settings"></div></section>
  <section id="driving-panel" hidden><h1>行驶道路视图</h1><p>只读实时视图；融合 SP 模型与 HW4 Model Y 原车 CAN，不启动视频或屏幕采集。</p><div id="driving-state" class="notice">正在连接车辆数据…</div><canvas id="driving-canvas" aria-label="预测道路轨迹与原车感知"></canvas><div id="can-details" class="can-grid"></div><div id="driving-alert" class="notice drive-alert" hidden></div></section>
  <section id="turn-panel" hidden>
    <h1>Tesla 转向 CAN 测试</h1><p>请求由 card 实时线程跟随原车 0x3E9 模板持续发送；SP 完成变道后会自动关闭转向灯。</p>
    <div class="buttons"><button class="turn" id="left" onclick="run('left')">← 左转</button><button class="turn" id="right" onclick="run('right')">右转 →</button></div>
    <button id="cancel" onclick="cancelSession()">立即取消</button><div id="status"></div>
  </section>
  <section id="terminal-panel" hidden>
    <h1>设备终端</h1><p>仅在设置模式（非行驶状态）且设备端显式启用后可用。命令最长 20 秒，输出上限 64 KiB。</p>
    <div id="terminal-state" class="notice">正在检查终端状态…</div><div class="terminal-row"><input id="terminal-password" type="password" autocomplete="off" placeholder="终端密码"><button onclick="runTerminal()">运行</button></div><p>默认密码：123456（仅需输入一次，浏览器会记住）</p><div class="terminal-row"><input id="terminal-new-password" type="password" autocomplete="new-password" placeholder="新密码（4-64个字符）"><button onclick="changeTerminalPassword()">修改密码</button></div>
    <textarea id="terminal-command" spellcheck="false" placeholder="git status --short"></textarea><pre id="terminal-output"></pre>
  </section>
<script>
let settingsState = null, hotspotState = null, selectedCategory = null, currentPanel = 'settings', drivingLoading = false;
function showPanel(name) {
  currentPanel = name;
  document.getElementById('settings-panel').hidden = name !== 'settings'; document.getElementById('driving-panel').hidden = name !== 'driving'; document.getElementById('turn-panel').hidden = name !== 'turn'; document.getElementById('terminal-panel').hidden = name !== 'terminal';
  document.getElementById('settings-tab').classList.toggle('active', name === 'settings'); document.getElementById('driving-tab').classList.toggle('active', name === 'driving'); document.getElementById('turn-tab').classList.toggle('active', name === 'turn'); document.getElementById('terminal-tab').classList.toggle('active', name === 'terminal');
  if (name === 'driving') loadDrivingStatus();
}
function element(tag, attrs = {}, text = '') { const e = document.createElement(tag); Object.assign(e, attrs); if (text) e.textContent = text; return e; }
function renderSettings(data) {
  settingsState = data; const mode = document.getElementById('mode'); mode.textContent = data.onroad ? '行驶中：只允许修改标注“行驶中可调”的设置。' : '设置模式：可修改全部白名单设置。'; mode.className = 'notice' + (data.onroad ? ' onroad' : '');
  const categories = data.menu; if (!selectedCategory || !categories.includes(selectedCategory)) selectedCategory = categories[0];
  const nav = document.getElementById('category-nav'); nav.replaceChildren(); categories.forEach(category => { const button = element('button', {className:'category' + (category === selectedCategory ? ' active' : '')}, category); button.onclick = () => { selectedCategory = category; renderSettings(data); }; nav.append(button); });
  const root = document.getElementById('settings'); root.replaceChildren(); let group = '';
  data.settings.filter(setting => setting.category === selectedCategory).sort((a,b) => a.group.localeCompare(b.group) || a.title.localeCompare(b.title)).forEach(setting => {
    if (setting.group !== group) { group = setting.group; root.append(element('div', {className:'group'}, group)); }
    const card = element('div', {className:'card'}), description = element('div'), title = element('h2', {}, setting.title || setting.key);
    description.append(title); if (setting.description) description.append(element('p', {}, setting.description));
    const locked = data.onroad && setting.offroad_only; if (locked) description.append(element('div', {className:'lock'}, '仅设置模式可调'));
    let control;
    if (setting.widget === 'toggle') { control = element('input', {type:'checkbox', checked:setting.value, disabled:locked}); control.onchange = () => save(setting, control.checked, control); }
    else if (setting.options) { control = element('select', {disabled:locked}); setting.options.forEach(option => control.append(element('option', {value:String(option.value), selected:option.value === setting.value}, option.label))); control.onchange = () => save(setting, control.value === '' ? '' : Number(control.value), control); }
    else { control = element('input', {type:'number', value:setting.value, min:setting.min ?? '', max:setting.max ?? '', step:setting.step ?? 1, disabled:locked}); control.onchange = () => save(setting, Number(control.value), control); }
    control.title = setting.key; card.append(description, control); root.append(card);
  });
  if (selectedCategory === '设备' && hotspotState?.available) {
    const card = element('div', {className:'card'}), description = element('div');
    description.append(element('h2', {}, '设备 Wi-Fi 热点'));
    description.append(element('p', {}, hotspotState.active ? '热点已开启。连接后访问 ' + hotspotState.url : '开启后设备会切换为热点；连接手机或电脑后访问 ' + hotspotState.url));
    const control = element('input', {type:'checkbox', checked:hotspotState.active});
    control.onchange = () => saveHotspot(control.checked, control); card.append(description, control); root.append(card);
  }
}
async function loadSettings() { try { const [settingsResponse, hotspotResponse] = await Promise.all([fetch('/api/settings', {cache:'no-store'}), fetch('/api/hotspot', {cache:'no-store'})]); if (!settingsResponse.ok) throw new Error('HTTP ' + settingsResponse.status); hotspotState = hotspotResponse.ok ? await hotspotResponse.json() : null; renderSettings(await settingsResponse.json()); } catch (e) { document.getElementById('mode').textContent = '设置读取失败：' + e; } }
async function save(setting, value, control) { control.disabled = true; try { const r = await fetch('/api/settings/' + encodeURIComponent(setting.key), {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({value})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); setting.value = result.value; } catch (e) { alert('保存失败：' + e); } finally { renderSettings(settingsState); } }
async function saveHotspot(enabled, control) { control.disabled = true; try { const r = await fetch('/api/hotspot', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enabled})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); hotspotState = result; renderSettings(settingsState); } catch (e) { alert('热点切换失败：' + e); renderSettings(settingsState); } }
loadSettings();
function drawLine(ctx, points, xScale, yScale, color, width) { if (!points.length) return; ctx.beginPath(); points.forEach(([x,y], i) => { const px = ctx.canvas.clientWidth / 2 + y * yScale, py = ctx.canvas.clientHeight - 32 - x * xScale; i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); }); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }
const canText = {
  lead:'前方', left:'左侧', right:'右侧', cutin:'切入', car:'车辆', truck:'卡车', motorcycle:'摩托车', bicycle:'自行车', pedestrian:'行人', unknown:'未知',
  red:'红灯', green:'绿灯', yellow:'黄灯', white:'白灯', off:'信号灯熄灭', none:'无', stop_sign:'停止标志', traffic_light:'红绿灯', yield:'让行', crosswalk:'人行横道', pedestrian_crossing:'行人过街', ramp_meter:'匝道灯', speed_bump:'减速带', speed_hump:'减速丘',
  disabled:'禁用', unavailable:'不可用', available:'可用', active:'激活', standby:'待机', aware:'感知', warning:'警告', stopping:'停车中', stopped:'已停车', continuing:'继续通行',
  map:'地图', vision:'视觉', map_and_vision:'地图+视觉', navigation:'导航', fused:'已融合', rejected:'拒绝', blacklisted:'黑名单', nominal:'正常', degraded:'降级', severely_degraded:'严重降级', fault:'故障', normal:'普通路面', enhanced:'增强路面',
  regular:'常规限速', advisory:'建议限速', dependent:'条件限速', bumps:'减速设施', class_1_major:'一级主干道', class_2:'二级道路', class_3:'三级道路', class_4:'四级道路', class_5:'五级道路', class_6_minor:'六级支路', circle:'圆形', straight:'直行',
  in_lane:'车道内', lane_change_left:'向左变道', lane_change_right:'向右变道', virtual_lane:'虚拟车道', follow:'跟车', lane_change_requested:'请求变道', lane_change_in_progress:'变道中', waiting_side_obstacle:'等待侧方车辆', waiting_forward_obstacle:'等待前方车辆', lane_change_abort:'变道中止'
};
function ct(value) { return canText[value] || String(value ?? '—').replaceAll('_', ' '); }
function canvasPoint(canvas, x, y, xScale, yScale) { return [canvas.clientWidth / 2 + y * yScale, canvas.clientHeight - 32 - x * xScale]; }
function drawCanVehicle(ctx, vehicle, xScale, yScale) {
  const [px, py] = canvasPoint(ctx.canvas, vehicle.x_m, vehicle.y_m, xScale, yScale); if (py < 52 || py > ctx.canvas.clientHeight - 20) return;
  const color = vehicle.relevant_for_control ? '#ef4444' : vehicle.category === 'left' ? '#60a5fa' : vehicle.category === 'right' ? '#c084fc' : '#f97316'; ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  if (vehicle.type === 'pedestrian') { ctx.beginPath(); ctx.arc(px, py - 6, 4, 0, Math.PI * 2); ctx.fill(); ctx.beginPath(); ctx.moveTo(px, py - 2); ctx.lineTo(px, py + 8); ctx.moveTo(px, py + 2); ctx.lineTo(px - 5, py + 7); ctx.moveTo(px, py + 2); ctx.lineTo(px + 5, py + 7); ctx.stroke(); }
  else if (vehicle.type === 'bicycle' || vehicle.type === 'motorcycle') { ctx.beginPath(); ctx.arc(px - 6, py + 4, 4, 0, Math.PI * 2); ctx.arc(px + 6, py + 4, 4, 0, Math.PI * 2); ctx.moveTo(px - 6, py + 4); ctx.lineTo(px, py - 4); ctx.lineTo(px + 6, py + 4); ctx.stroke(); }
  else { const w = vehicle.type === 'truck' ? 21 : 16, h = vehicle.type === 'truck' ? 27 : 20; ctx.fillRect(px - w / 2, py - h / 2, w, h); ctx.fillStyle = '#0f172a'; ctx.fillRect(px - w / 2 + 3, py - h / 2 + 3, w - 6, 5); }
  ctx.fillStyle = '#f8fafc'; ctx.font = '11px sans-serif'; const rel = Number(vehicle.relative_speed); const velocity = Number.isFinite(rel) ? '  Δv ' + rel.toFixed(0) : ''; ctx.fillText(ct(vehicle.category) + ' ' + vehicle.x_m.toFixed(0) + 'm' + velocity, px + 12, py + 4);
}
function drawTraffic(ctx, traffic, xScale, width, height, mapSign) {
  const sign = traffic?.road_sign_available ? {distance: traffic.stop_line_distance_m, color: traffic.road_sign_color, arrow: traffic.road_sign_arrow, label: traffic.road_sign_type} : null;
  const control = traffic?.control_available ? {distance: traffic.control_distance_m, color: traffic.light_state, arrow: null, label: traffic.light_state, type: traffic.control_type} : null;
  const mapLight = mapSign && mapSign.traffic_light_stop_line_distance_m != null ? {distance: mapSign.traffic_light_stop_line_distance_m, color: null, arrow: null, label: 'traffic_light', type: null, map: true} : null;
  const directional = sign && (sign.arrow === 'left' || sign.arrow === 'right' || sign.arrow === 'straight');
  const src = directional && sign.distance != null ? sign : (control && control.distance != null ? control : (sign && sign.distance != null ? sign : (mapLight || null)));
  if (!src) return; const distance = src.distance; if (distance < 0 || distance > 100) return;
  const color = ({red:'#ef4444',green:'#22c55e',yellow:'#facc15',white:'#f8fafc'})[src.color] || ({red:'#ef4444',green:'#22c55e',yellow:'#facc15',red_yellow:'#f97316'})[src.color] || '#94a3b8'; const py = height - 32 - distance * xScale;
  ctx.strokeStyle = color; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(width / 2 - 78, py); ctx.lineTo(width / 2 + 78, py); ctx.stroke(); ctx.fillStyle = color; ctx.font = 'bold 13px sans-serif';
  let label = ''; if (['red','green','yellow','white','off','other'].includes(src.label)) label = ct(src.label); else if (src.label === 'traffic_light' || src.label === 'stop_sign') label = ct(src.label); else if (src.type === 'crosswalk' || src.type === 'pedestrian_crossing') label = ct(src.type); else label = '红绿灯';
  const arrow = ({left:' 左转', right:' 右转', straight:' 直行', circle:' 圆灯'})[src.arrow] || '';
  ctx.fillText(label + arrow + (src.map ? ' 地图' : '') + '  ' + distance.toFixed(0) + 'm', width / 2 + 86, py + 4);
  if (traffic.control_type === 'crosswalk' || traffic.control_type === 'pedestrian_crossing') { ctx.strokeStyle = '#f8fafc'; ctx.lineWidth = 2; for (let i=-3;i<=3;i++) { ctx.beginPath(); ctx.moveTo(width/2+i*17-6,py+8); ctx.lineTo(width/2+i*17+6,py+8); ctx.stroke(); } }
}
function renderCanDetails(can) {
  const root = document.getElementById('can-details'); root.replaceChildren(); if (!can?.available) { const box = element('div',{className:'can-detail warn'},'尚未收到 HW4 感知 CAN（DBC：tesla_modely_hw4_perception）'); root.append(box); return; }
  const add = (title, lines) => { const box = element('div',{className:'can-detail'}); box.append(element('strong',{},title)); lines.filter(Boolean).forEach(line => box.append(element('div',{},line))); root.append(box); };
  const n = can.navigation || {}; const rejects = [['导航',n.reject_navigation],['HPP',n.reject_hpp],['左车道',n.reject_left_lane],['右车道',n.reject_right_lane],['左自由空间',n.reject_left_free_space],['右自由空间',n.reject_right_free_space],['自动转向',n.reject_autosteer],['手扶方向盘',n.reject_hands_on]].filter(([,value]) => value).map(([name]) => name).join('/'); add('导航 / 地图', [n.route_active ? '路线：已激活' : '路线：未激活', '地图导航：' + (n.nav_available ? '可用' : '不可用') + (n.nav_distance_m != null ? ' · ' + n.nav_distance_m + 'm' : ''), '道路匹配：' + (n.gps_road_match ? '是' : '否'), n.next_branch_distance_m != null ? '下一分支：' + n.next_branch_distance_m + 'm' + (n.next_branch_left_off_ramp ? ' 左出口' : n.next_branch_right_off_ramp ? ' 右出口' : '') : '', n.speed_limit_unlimited ? '限速：不限速' : n.speed_limit != null ? '限速：' + n.speed_limit + ' ' + n.speed_limit_unit.toUpperCase() + ' · ' + ct(n.speed_limit_type) + ' · 依赖 ' + n.speed_limit_dependency : '', '道路：' + ct(n.road_class) + (n.controlled_access ? ' · 封闭道路' : '') + ' · 国家码 ' + n.country_code + ' · 街道数 ' + n.street_count, '导航融合：' + ct(n.autosteer_navigation_usage), '泊车：平行 ' + (n.parallel_autopark_enabled?'开':'关') + ' / 垂直 ' + (n.perpendicular_autopark_enabled?'开':'关'), '标线：Botts dots ' + (n.accept_botts_dots?'接受':'不接受') + ' · PMM ' + (n.pmm_enabled?'开':'关') + ' · SCA ' + (n.sca_enabled?'开':'关'), n.in_supercharger_geofence ? '位于超充地理围栏' : '', n.autosteer_restricted ? '自动转向受限' : '', rejects ? '拒绝项：' + rejects : '拒绝项：无', '数据源总线：' + (n.sources || []).join(' / ')]);
  const l = can.lanes || {}; add('车道', [l.available ? '宽度：' + l.width_m + 'm · 范围：' + l.view_range_m + 'm' : '未收到车道报文', '左线：' + (l.left_exists ? ct(l.left_usage) : '不存在'), '右线：' + (l.right_exists ? ct(l.right_usage) : '不存在'), l.left_fork === 2 ? '左分叉已选择' : '', l.right_fork === 2 ? '右分叉已选择' : '']);
  const t = can.traffic || {}; add('交通控制', [t.control_available ? ct(t.control_type) + ' · ' + ct(t.light_state) : (t.control_frame_fresh ? '收到控制报文，但当前无有效红绿灯控制' : '未收到 APP 交通控制报文'), t.stop_line_distance_m != null ? '停止线：' + t.stop_line_distance_m + 'm' : t.control_distance_m != null ? '控制点：' + t.control_distance_m + 'm' : '', t.control_available ? '状态：' + ct(t.state_machine) + ' · 功能 ' + ct(t.feature_state) + ' · 来源 ' + ct(t.control_source) : '', t.road_sign_available ? '标志：' + ct(t.road_sign_type) + ' · ' + ct(t.road_sign_color) + ' · ' + ct(t.road_sign_source) + ' · 箭头 ' + ct(t.road_sign_arrow) : (t.sign_frame_fresh ? '收到标志帧，但未检测到有效标志' : '未收到道路标志报文'), t.vision_light || t.vision_sign || t.vision_road_marking || t.vision_line ? '视觉：' + [t.vision_light&&'灯',t.vision_sign&&'标志',t.vision_road_marking&&'路面标记',t.vision_line&&'停止线'].filter(Boolean).join('/') : '', t.control_available ? '确认类型 ' + t.confirmation_type + ' · 继续原因 ' + t.continuation_reason + ' · 抑制原因 ' + t.warning_suppression_reason + ' · 不可用原因 ' + t.unavailable_reason : '', '数据源总线：' + (t.sources || []).join(' / ')]);
  const vehicles = can.vehicles || []; add('周边目标', vehicles.length ? [...vehicles.map(v => ct(v.category) + ' ' + ct(v.type) + ' #' + v.track_id + '：纵向 ' + v.x_m + 'm，横向 ' + v.y_m + 'm，相对速度值 ' + v.relative_speed), '注：相对速度的 T-CAN DBC 源未标物理单位'] : ['未收到有效目标', can.rear_vehicles?.left_live ? '左后方检测到车辆' : '', can.rear_vehicles?.right_live ? '右后方检测到车辆' : '']);
  const a = can.driver_assist || {}; add('驾驶辅助', [a.available ? '规划：' + ct(a.planner_state) + ' · 行为 ' + ct(a.behavior) : '未收到驾驶辅助调试报文', a.available ? '健康：' + ct(a.health) + ' · 异常等级 ' + a.health_anomaly_level + ' · 中止原因 ' + a.last_abort_reason : '', a.available ? '路面：' + ct(a.road_surface) + ' · 偏移侧 ' + a.offset_side + ' · 选线原因 ' + a.last_line_preference_reason : '', a.available ? '融合：车辆 ' + ct(a.vehicles_usage) + ' / HPP ' + ct(a.hpp_usage) + ' / 模型 ' + ct(a.model_usage) + ' / Botts dots ' + ct(a.botts_dots_usage) : '', a.available ? '智能限速 ' + (a.smart_speed_active?'激活':'未激活') + ' · 状态 ' + a.smart_speed_state + ' · ISA ' + a.isa_state + ' · 交通感知设定速度 ' + (a.traffic_aware_set_speed?'是':'否') : '', a.available ? 'ULC ' + (a.ulc_in_progress?'进行中':'未进行') + ' · 类型 ' + a.ulc_type + ' · 开发者接口 ' + (a.developer_app_interface_enabled?'开':'关') : '', '全部数据总线：' + (can.buses || []).join(' / ')]);
  const rs = can.road_sign || {}; add('道路标志（地图/车队）', [rs.available ? '红绿灯停止线：' + (rs.traffic_light_stop_line_distance_m != null ? rs.traffic_light_stop_line_distance_m + 'm' : '无') + (rs.traffic_light_stop_line_confidence != null ? '（置信度 ' + rs.traffic_light_stop_line_confidence + '）' : '') : '未收到 UI_driverAssistRoadSign', rs.stop_sign_stop_line_distance_m != null ? '停止标志停止线：' + rs.stop_sign_stop_line_distance_m + 'm（置信度 ' + (rs.stop_sign_stop_line_confidence ?? '—') + '）' : '', rs.base_map_speed_limit_mps != null ? '地图限速：' + (rs.base_map_speed_limit_mps * 3.6).toFixed(0) + ' km/h' : '', rs.mean_fleet_spline_speed_mps != null ? '车队速度：' + (rs.mean_fleet_spline_speed_mps * 3.6).toFixed(0) + ' km/h' : '', rs.ramp_type != null ? '匝道类型 ' + rs.ramp_type : '', '数据源总线：' + (rs.bus || '—')]);
  const p = can.pedestrian_detection || {}; const pedList = (p.closest || []).filter(c => c.x_m || c.y_m); add('行人检测', [p.available ? '相机：前主 ' + (p.front_main?'有':'无') + ' · 前鱼眼 ' + (p.front_fisheye?'有':'无') + ' · 前窄 ' + (p.front_narrow?'有':'无') + ' · 左柱 ' + (p.left_pillar?'有':'无') + ' · 左镜 ' + (p.left_repeater?'有':'无') + ' · 右柱 ' + (p.right_pillar?'有':'无') + ' · 右镜 ' + (p.right_repeater?'有':'无') + ' · 倒车 ' + (p.backup?'有':'无') : '未收到行人检测报文', pedList.length ? '最近行人：' + pedList.map(c => c.x_m.toFixed(1) + ',' + c.y_m.toFixed(1) + 'm').join(' / ') : (p.available ? '当前无最近行人' : ''), '数据源总线：' + (p.bus || '—')]);
  const bs = can.blind_spot || {}; add('盲区 / 侧碰', [bs.available ? '左后等级 ' + (bs.left_level ?? '—') + ' · 右后等级 ' + (bs.right_level ?? '—') + (bs.left_live || bs.right_live ? ' ⚠' : '') : '未收到 DAS_status', bs.side_collision_warning_level != null ? '侧碰预警 ' + bs.side_collision_warning_level + ' · 避让 ' + bs.side_collision_avoid_level + (bs.side_collision_inhibit ? '（抑制）' : '') : '', bs.forward_collision_warning_level != null ? '前碰预警 ' + bs.forward_collision_warning_level : '', bs.lane_departure_warning_level != null ? '车道偏离 ' + bs.lane_departure_warning_level : '', bs.fused_speed_limit_kph ? '融合限速 ' + bs.fused_speed_limit_kph + ' km/h' : '', '数据源总线：' + (bs.sources || []).join(' / ')]);
  const f = can.front_safety || {}; add('前向安全', [f.available && f.target_distance_m != null ? '目标距离 ' + f.target_distance_m + 'm' : (f.available ? '前方无目标' : '未收到前向安全报文'), f.relative_velocity_mps != null ? '相对速度 ' + f.relative_velocity_mps + ' m/s' : '', f.time_to_impact_s != null ? '碰撞时间 ' + f.time_to_impact_s + 's' : '', f.predicted_impact_overlap_pct != null ? '预测重叠 ' + f.predicted_impact_overlap_pct + '%' : '', f.imminent_collision ? '⚠ 即将碰撞' : '', '数据源总线：' + (f.bus || '—')]);
}
function drawDrivingGeometry(geometry, data) {
  const canvas = document.getElementById('driving-canvas'), ratio = window.devicePixelRatio || 1, width = Math.max(1, canvas.clientWidth), height = Math.max(1, canvas.clientHeight); if (canvas.width !== width * ratio || canvas.height !== height * ratio) { canvas.width = width * ratio; canvas.height = height * ratio; }
  const ctx = canvas.getContext('2d'); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, width, height); const grad = ctx.createLinearGradient(0,0,0,height); grad.addColorStop(0,'#172554'); grad.addColorStop(1,'#020617'); ctx.fillStyle=grad; ctx.fillRect(0,0,width,height); const xScale = (height - 70) / 100, yScale = Math.min(width / 12, 31), danger = geometry.hard_brake_predicted, can = geometry.oem_can || {};
  ctx.fillStyle='#111827aa'; ctx.beginPath(); ctx.moveTo(width/2-28,58); ctx.lineTo(width/2+28,58); ctx.lineTo(width/2+Math.min(width*.42,260),height); ctx.lineTo(width/2-Math.min(width*.42,260),height); ctx.fill();
  ctx.setLineDash([7,7]); (geometry.edges || []).forEach(line => drawLine(ctx,line,xScale,yScale,'#64748b',2)); ctx.setLineDash([]); (geometry.lanes || []).forEach(line => drawLine(ctx,line,xScale,yScale,'#e2e8f0',2));
  const oemLanes = can.lanes || {}; if (oemLanes.available) { drawLine(ctx,oemLanes.left||[],xScale,yScale,'#22d3ee',3); drawLine(ctx,oemLanes.right||[],xScale,yScale,'#22d3ee',3); ctx.setLineDash([5,5]); drawLine(ctx,oemLanes.center||[],xScale,yScale,'#0891b2',2); ctx.setLineDash([]); }
  drawLine(ctx, geometry.path || [], xScale, yScale, danger ? '#ef4444' : '#22c55e', 5); const legacyTraffic=geometry.oem_traffic||{}; const traffic=can.traffic?.available?can.traffic:legacyTraffic.available?{available:true,control_available:true,light_state:['none','red','green','yellow'][legacyTraffic.light_color]||'unknown',control_distance_m:legacyTraffic.stop_line_distance,control_type:'traffic_light'}:{}; drawTraffic(ctx, traffic, xScale, width, height, can.road_sign || {});
  (can.vehicles || []).forEach(vehicle => drawCanVehicle(ctx,vehicle,xScale,yScale)); (geometry.leads || []).forEach(lead => { if ((can.vehicles||[]).some(v => v.category==='lead')) return; const [px,py]=canvasPoint(canvas,lead.x,lead.y,xScale,yScale); if(py>50&&py<height){ctx.fillStyle=danger?'#ef4444':'#f97316';ctx.fillRect(px-7,py-7,14,14);ctx.fillStyle='#f8fafc';ctx.font='12px sans-serif';ctx.fillText(Math.round(lead.x)+'m',px+10,py+4);} });
  const ped = can.pedestrian_detection || {}; (ped.closest || []).forEach(c => { if (!c.x_m && !c.y_m) return; const [px,py]=canvasPoint(canvas,c.x_m,c.y_m,xScale,yScale); if (py < 52 || py > height - 20) return; ctx.strokeStyle='#f59e0b'; ctx.fillStyle='#f59e0b'; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(px,py-6,4,0,Math.PI*2); ctx.fill(); ctx.beginPath(); ctx.moveTo(px,py-2); ctx.lineTo(px,py+8); ctx.moveTo(px,py+2); ctx.lineTo(px-5,py+7); ctx.moveTo(px,py+2); ctx.lineTo(px+5,py+7); ctx.stroke(); ctx.fillStyle='#f8fafc'; ctx.font='11px sans-serif'; ctx.fillText('行人 '+Math.round(c.x_m)+'m',px+12,py+4); });
  const rear = can.rear_vehicles || {}, blind = can.blind_spot || {};
  if (blind.left_live || rear.left_live) { ctx.fillStyle='#60a5fa'; ctx.font='bold 19px sans-serif'; ctx.fillText('◀ 左后车',14,height-18); } if (blind.right_live || rear.right_live) { ctx.fillStyle='#c084fc'; ctx.font='bold 19px sans-serif'; ctx.fillText('右后车 ▶',width-95,height-18); }
  ctx.fillStyle='#2563eb'; ctx.fillRect(width/2-14,height-30,28,20); ctx.fillStyle='#f8fafc'; ctx.font='bold 17px sans-serif'; ctx.fillText(data.speed_kph.toFixed(0)+' km/h',14,28); ctx.font='13px sans-serif'; const mode=data.openpilot_enabled?'SP 接管':data.mads_enabled?'MADS 横向':'未接管'; ctx.fillText(mode+'  ·  设定 '+data.set_speed_kph.toFixed(0)+' km/h',14,49);
  const fs = can.front_safety || {}; if (fs.available && fs.target_distance_m != null) { ctx.fillStyle='#34d399'; ctx.font='bold 13px sans-serif'; ctx.fillText('前方目标 '+fs.target_distance_m.toFixed(1)+'m'+(fs.time_to_impact_s!=null?' · TTI '+fs.time_to_impact_s.toFixed(1)+'s':''),14,70); }
  const nav=can.navigation||{}; if(nav.available){ctx.textAlign='right';ctx.fillStyle='#dbeafe';ctx.font='bold 14px sans-serif';const branch=nav.next_branch_distance_m!=null?nav.next_branch_distance_m+'m '+(nav.next_branch_left_off_ramp?'↖ 左出口':nav.next_branch_right_off_ramp?'右出口 ↗':'下一分支'):nav.route_active?'导航路线已激活':'导航可用';ctx.fillText(branch,width-14,28);ctx.font='12px sans-serif';const limit=nav.speed_limit_unlimited?'不限速':nav.speed_limit!=null?'限速 '+nav.speed_limit+' '+nav.speed_limit_unit.toUpperCase():ct(nav.road_class);ctx.fillText(limit,width-14,48);ctx.textAlign='left';}
  if(geometry.lane_change!=='off'){ctx.fillStyle='#fbbf24';ctx.font='bold 15px sans-serif';ctx.fillText('变道 '+(geometry.lane_change_direction==='left'?'←':geometry.lane_change_direction==='right'?'→':'进行中'),width-92,70);} if(danger){ctx.fillStyle='#fecaca';ctx.fillText('注意制动风险',width-100,90);} renderCanDetails(can);
}
async function loadDrivingStatus() { if (drivingLoading) return; drivingLoading = true; const state = document.getElementById('driving-state'), alert = document.getElementById('driving-alert'); try { const r = await fetch('/api/driving-status', {cache:'no-store'}); const data = await r.json(); if (!r.ok) throw new Error(data.message || 'HTTP ' + r.status); const connected = Object.values(data.connected).every(Boolean); state.textContent = !data.onroad ? '设置模式：等待车辆启动。' : connected ? '行驶中：车辆数据正常。' : '行驶中：部分车辆数据暂未收到。'; state.className = 'notice' + ((!data.onroad || !connected) ? ' onroad' : ''); drawDrivingGeometry(data.geometry, data); alert.hidden = !data.alert; alert.textContent = data.alert || ''; } catch (e) { state.textContent = '行驶数据读取失败：' + e; state.className = 'notice onroad'; } finally { drivingLoading = false; } }
setInterval(() => { if (currentPanel === 'driving') loadDrivingStatus(); }, 500);
async function loadTerminalStatus() { const el = document.getElementById('terminal-state'); try { const r = await fetch('/api/terminal/status', {cache:'no-store'}); const s = await r.json(); el.textContent = !s.enabled ? '终端未启用：请在设备上显式启用。' : s.onroad ? '行驶中：请先进入设置模式。' : '终端已启用：请输入密码后运行。'; el.className = 'notice' + ((!s.enabled || s.onroad) ? ' onroad' : ''); } catch (e) { el.textContent = '终端状态读取失败：' + e; } }
const terminalPasswordInput = document.getElementById('terminal-password'); terminalPasswordInput.value = localStorage.getItem('openpilotTerminalPassword') || '123456';
async function runTerminal() { const password = terminalPasswordInput.value, command = document.getElementById('terminal-command').value, output = document.getElementById('terminal-output'); output.textContent = '正在运行…'; try { const r = await fetch('/api/terminal/exec', {method:'POST', headers:{'Content-Type':'application/json', 'X-Terminal-Password':password}, body:JSON.stringify({command})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); localStorage.setItem('openpilotTerminalPassword', password); output.textContent = `[exit ${result.exit_code}${result.timed_out ? ', timeout' : ''}]\n` + result.output; } catch (e) { output.textContent = '运行失败：' + e; } }
async function changeTerminalPassword() { const password = terminalPasswordInput.value, newPassword = document.getElementById('terminal-new-password').value; try { const r = await fetch('/api/terminal/password', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({current_password:password, new_password:newPassword})}); const result = await r.json(); if (!r.ok) throw new Error(result.message || 'HTTP ' + r.status); terminalPasswordInput.value = newPassword; localStorage.setItem('openpilotTerminalPassword', newPassword); document.getElementById('terminal-new-password').value = ''; alert('密码已修改'); } catch (e) { alert('修改失败：' + e); } }
loadTerminalStatus();
let activeTestId = null;
const phaseText = {
  queued: '请求已提交', waiting_vehicle_feedback: '等待车辆转向灯响应',
  waiting_sp_start: '等待 SP 开始变道', lane_changing: 'SP 正在执行变道',
  cancelling: '变道完成，正在关闭转向灯', confirming_cancel: '正在确认转向灯关闭'
};
async function run(direction) {
  document.querySelectorAll('#left,#right').forEach(button => button.disabled = true);
  const status = document.getElementById('status');
  status.textContent = '正在提交…';
  try {
    const response = await fetch('/api/turn/' + direction, {method:'POST'});
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || '提交失败');
    activeTestId = result.test_id;
    document.getElementById('cancel').style.display = 'block';
    await pollStatus();
  } catch (error) { finishUi('请求失败：' + error); }
}
async function pollStatus() {
  if (!activeTestId) return;
  try {
    const response = await fetch('/api/status/' + activeTestId, {cache:'no-store'});
    const result = await response.json();
    const detail = '已发送 ' + (result.action_frames_sent || 0) + ' 帧';
    if (result.done) {
      const ok = result.result === 'PASS';
      finishUi((ok ? '完成：转向灯已自动关闭' : '结束：' + result.result) + '\\n' + detail);
      return;
    }
    document.getElementById('status').textContent = (phaseText[result.phase] || result.phase) + '\\n' + detail;
    setTimeout(pollStatus, 200);
  } catch (error) { finishUi('状态读取失败：' + error); }
}
async function cancelSession() {
  if (!activeTestId) return;
  document.getElementById('status').textContent = '正在请求关闭转向灯…';
  try { await fetch('/api/cancel/' + activeTestId, {method:'POST'}); }
  catch (error) { finishUi('取消请求失败：' + error); }
}
function finishUi(message) {
  document.getElementById('status').textContent = message;
  document.querySelectorAll('#left,#right').forEach(button => button.disabled = false);
  document.getElementById('cancel').style.display = 'none';
  activeTestId = null;
}
</script></main></body></html>""".encode()


class TurnSignalHandler(BaseHTTPRequestHandler):
  server_version = "TeslaTurnSignalTest/1.0"

  def _send(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
    self.send_response(status)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(body)

  def do_GET(self) -> None:
    if self.path == "/api/hotspot":
      self._json(HTTPStatus.OK, hotspot_status())
      return
    if self.path == "/api/driving-status":
      try:
        self._json(HTTPStatus.OK, driving_status_snapshot())
      except Exception as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"行驶数据暂不可用：{error}"})
      return
    if self.path == "/api/terminal/status":
      self._json(HTTPStatus.OK, terminal_status())
      return
    if self.path == "/api/settings":
      self._json(HTTPStatus.OK, settings_snapshot())
      return
    if self.path.startswith("/api/status/"):
      test_id = self.path.removeprefix("/api/status/")
      with _SESSION_LOCK:
        session_expired = (_ACTIVE_WEB_TEST_ID == test_id and
                           time.monotonic() - _ACTIVE_WEB_SESSION_STARTED >= WEB_SESSION_TIMEOUT_S)
      if session_expired:
        cancel_validation_session(test_id)
        _clear_active_session(test_id)
        self._json(HTTPStatus.OK, {"test_id": test_id, "done": True, "result": "WEB_SESSION_TIMEOUT"})
        return
      status = get_validation_status(test_id)
      if status.get("done"):
        _clear_active_session(test_id)
      self._json(HTTPStatus.OK, status)
      return
    if self.path not in ("/", "/index.html"):
      self._send(HTTPStatus.NOT_FOUND, "text/plain; charset=utf-8", b"Not found")
      return
    self._send(HTTPStatus.OK, "text/html; charset=utf-8", render_page())

  def do_POST(self) -> None:
    if self.path == "/api/hotspot":
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 4096:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or not isinstance(payload.get("enabled"), bool):
          raise ValueError("请求必须包含 enabled 开关")
        self._json(HTTPStatus.OK, set_hotspot_enabled(payload["enabled"]))
      except RuntimeError as error:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path == "/api/terminal/password":
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 8192:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict):
          raise ValueError("请求必须为 JSON 对象")
        change_password(payload.get("current_password"), payload.get("new_password"))
        self._json(HTTPStatus.OK, {"ok": True})
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path == "/api/terminal/exec":
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 8192:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or "command" not in payload:
          raise ValueError("请求必须包含 command")
        self._json(HTTPStatus.OK, run_command(payload["command"], self.headers.get("X-Terminal-Password")))
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path.startswith("/api/settings/"):
      key = self.path.removeprefix("/api/settings/")
      if not key or "/" in key:
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知设置"})
        return
      try:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 4096:
          raise ValueError("请求内容无效")
        payload = json.loads(self.rfile.read(content_length))
        if not isinstance(payload, dict) or "value" not in payload:
          raise ValueError("请求必须包含 value")
        self._json(HTTPStatus.OK, {"ok": True, **validate_and_write(key, payload["value"])})
      except KeyError:
        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知或不允许修改的设置"})
      except PermissionError as error:
        self._json(HTTPStatus.FORBIDDEN, {"ok": False, "message": str(error)})
      except (TypeError, ValueError, json.JSONDecodeError) as error:
        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
      return
    if self.path.startswith("/api/cancel/"):
      test_id = self.path.removeprefix("/api/cancel/")
      cancel_validation_session(test_id)
      self._json(HTTPStatus.ACCEPTED, {"ok": True, "test_id": test_id})
      return
    direction = self.path.removeprefix("/api/turn/")
    if direction not in ("left", "right") or self.path != f"/api/turn/{direction}":
      self._json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "未知测试请求"})
      return
    try:
      global _ACTIVE_WEB_SESSION_STARTED, _ACTIVE_WEB_TEST_ID
      with _SESSION_LOCK:
        if _ACTIVE_WEB_TEST_ID is not None:
          existing = get_validation_status(_ACTIVE_WEB_TEST_ID)
          session_expired = time.monotonic() - _ACTIVE_WEB_SESSION_STARTED >= WEB_SESSION_TIMEOUT_S
          if not existing.get("done") and not session_expired:
            self._json(HTTPStatus.CONFLICT, {"ok": False, "message": "已有变道请求正在运行"})
            return
          if not existing.get("done"):
            expired_test_id = _ACTIVE_WEB_TEST_ID
            cancel_validation_session(expired_test_id)
            _ACTIVE_WEB_TEST_ID = None
            _ACTIVE_WEB_SESSION_STARTED = 0.0
            self._json(HTTPStatus.CONFLICT, {
              "ok": False, "message": "上一变道请求已超时并取消，请稍后重试", "test_id": expired_test_id,
            })
            return
        test_id = start_validation_session(direction)
        _ACTIVE_WEB_TEST_ID = test_id
        _ACTIVE_WEB_SESSION_STARTED = time.monotonic()
      self._json(HTTPStatus.ACCEPTED, {"ok": True, "test_id": test_id, "log": VALIDATION_LOG_PATH})
    except RuntimeError as error:
      self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "message": f"测试被阻止：{error}"})

  def _json(self, status: HTTPStatus, payload: dict) -> None:
    self._send(status, "application/json; charset=utf-8", json.dumps(payload, ensure_ascii=False).encode())

  def log_message(self, message_format: str, *args) -> None:
    pass


def main() -> None:
  server = ThreadingHTTPServer((HOST, PORT), TurnSignalHandler)
  server.serve_forever()


if __name__ == "__main__":
  main()
