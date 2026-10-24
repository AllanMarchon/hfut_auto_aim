#include "debug_mjpeg_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace hfut::io {
namespace {

constexpr const char* kBoundary = "hfut_auto_aim_frame";

std::string escapeJson(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (char c : value) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

double finiteOrZero(double value) {
  return std::isfinite(value) ? value : 0.0;
}

std::string statusJson(const DebugMjpegStatus& status) {
  std::ostringstream out;
  out << '{'
      << "\"frames\":" << status.frames
      << ",\"fps\":" << finiteOrZero(status.fps)
      << ",\"latency_ms\":" << finiteOrZero(status.latency_ms)
      << ",\"detections\":" << status.detections
      << ",\"poses\":" << status.poses
      << ",\"armors\":" << status.armors
      << ",\"tracked\":" << status.tracked
      << ",\"selected_id\":\"" << escapeJson(status.selected_id) << "\""
      << ",\"track_state\":\"" << escapeJson(status.track_state) << "\""
      << ",\"reason\":\"" << escapeJson(status.reason) << "\""
      << ",\"mode\":" << status.mode
      << ",\"feedback_yaw_deg\":" << finiteOrZero(status.feedback_yaw_deg)
      << ",\"feedback_pitch_deg\":" << finiteOrZero(status.feedback_pitch_deg)
      << ",\"aligned_feedback_yaw_deg\":" << finiteOrZero(status.aligned_feedback_yaw_deg)
      << ",\"aligned_feedback_pitch_deg\":" << finiteOrZero(status.aligned_feedback_pitch_deg)
      << ",\"feedback_alignment_delta_yaw_deg\":" << finiteOrZero(status.feedback_alignment_delta_yaw_deg)
      << ",\"feedback_alignment_delta_pitch_deg\":" << finiteOrZero(status.feedback_alignment_delta_pitch_deg)
      << ",\"aligned_feedback_age_ms\":" << finiteOrZero(status.aligned_feedback_age_ms)
      << ",\"command_yaw_deg\":" << finiteOrZero(status.command_yaw_deg)
      << ",\"command_pitch_deg\":" << finiteOrZero(status.command_pitch_deg)
      << ",\"command_yaw_vel_rad_s\":" << finiteOrZero(status.command_yaw_vel_rad_s)
      << ",\"command_pitch_vel_rad_s\":" << finiteOrZero(status.command_pitch_vel_rad_s)
      << ",\"command_yaw_acc_rad_s2\":" << finiteOrZero(status.command_yaw_acc_rad_s2)
      << ",\"command_pitch_acc_rad_s2\":" << finiteOrZero(status.command_pitch_acc_rad_s2)
      << ",\"buff_rotation_speed_rad_s\":" << finiteOrZero(status.buff_rotation_speed_rad_s)
      << ",\"buff_angle_delta_speed_rad_s\":" << finiteOrZero(status.buff_angle_delta_speed_rad_s)
      << ",\"buff_motion_fit_cost\":" << finiteOrZero(status.buff_motion_fit_cost)
      << ",\"buff_rotation_speed_valid\":" << (status.buff_rotation_speed_valid ? "true" : "false")
      << ",\"buff_angle_delta_speed_valid\":" << (status.buff_angle_delta_speed_valid ? "true" : "false")
      << ",\"buff_motion_fit_valid\":" << (status.buff_motion_fit_valid ? "true" : "false")
      << ",\"buff_sine_fit_valid\":" << (status.buff_sine_fit_valid ? "true" : "false")
      << ",\"buff_model_predicted\":" << (status.buff_model_predicted ? "true" : "false")
      << ",\"buff_requires_warmup\":" << (status.buff_requires_warmup ? "true" : "false")
      << ",\"raw_target_yaw_deg\":" << finiteOrZero(status.raw_target_yaw_deg)
      << ",\"raw_target_pitch_deg\":" << finiteOrZero(status.raw_target_pitch_deg)
      << ",\"target_yaw_deg\":" << finiteOrZero(status.target_yaw_deg)
      << ",\"target_pitch_deg\":" << finiteOrZero(status.target_pitch_deg)
      << ",\"limiter_yaw_error_deg\":" << finiteOrZero(status.limiter_yaw_error_deg)
      << ",\"limiter_pitch_error_deg\":" << finiteOrZero(status.limiter_pitch_error_deg)
      << ",\"distance_m\":" << finiteOrZero(status.distance_m)
      << ",\"pnp_blade_distance_m\":" << finiteOrZero(status.pnp_blade_distance_m)
      << ",\"pnp_first_distance_m\":" << finiteOrZero(status.pnp_first_distance_m)
      << ",\"yaw_error_deg\":" << finiteOrZero(status.yaw_error_deg)
      << ",\"pitch_error_deg\":" << finiteOrZero(status.pitch_error_deg)
      << ",\"feedback_age_ms\":" << finiteOrZero(status.feedback_age_ms)
      << ",\"feedback_alignment_used\":" << (status.feedback_alignment_used ? "true" : "false")
      << ",\"fire_advice\":" << (status.fire_advice ? "true" : "false")
      << ",\"fire\":" << (status.fire ? "true" : "false")
      << ",\"fire_blocked_by_limiter\":" << (status.fire_blocked_by_limiter ? "true" : "false")
      << ",\"dry_run\":" << (status.dry_run ? "true" : "false")
      << ",\"fire_enabled\":" << (status.fire_enabled ? "true" : "false")
      << ",\"enemy_color\":\"" << escapeJson(status.enemy_color) << "\""
      << ",\"camera_backend\":\"" << escapeJson(status.camera_backend) << "\""
      << ",\"serial_tx\":\"" << escapeJson(status.serial_tx) << "\""
      << ",\"serial_rx\":\"" << escapeJson(status.serial_rx) << "\""
      << '}';
  return out.str();
}

std::string armorAvoidanceJson(const ArmorAvoidanceSettings& settings) {
  std::ostringstream out;
  out << '{'
      << "\"enabled\":" << (settings.enabled ? "true" : "false")
      << ",\"plate_width_mm\":" << settings.plate_width_mm
      << ",\"plate_height_mm\":" << settings.plate_height_mm
      << ",\"center_x_mm\":" << settings.center_x_mm
      << ",\"center_y_mm\":" << settings.center_y_mm
      << ",\"radius_mm\":" << settings.radius_mm
      << ",\"dispersion_sigma_mm\":" << settings.dispersion_sigma_mm
      << '}';
  return out.str();
}

bool parseJsonNumber(const std::string& json, const std::string& key, double& value) {
  const std::string needle = "\"" + key + "\"";
  const size_t key_pos = json.find(needle);
  if (key_pos == std::string::npos) return false;
  const size_t colon = json.find(':', key_pos + needle.size());
  if (colon == std::string::npos) return false;
  const char* begin = json.c_str() + colon + 1;
  while (*begin == ' ' || *begin == '\t' || *begin == '\r' || *begin == '\n') ++begin;
  char* end = nullptr;
  const double parsed = std::strtod(begin, &end);
  if (end == begin || !std::isfinite(parsed)) return false;
  while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') ++end;
  if (*end != ',' && *end != '}') return false;
  value = parsed;
  return true;
}

bool parseJsonBool(const std::string& json, const std::string& key, bool& value) {
  const std::string needle = "\"" + key + "\"";
  const size_t key_pos = json.find(needle);
  if (key_pos == std::string::npos) return false;
  const size_t colon = json.find(':', key_pos + needle.size());
  if (colon == std::string::npos) return false;
  size_t value_pos = colon + 1;
  while (value_pos < json.size() &&
         (json[value_pos] == ' ' || json[value_pos] == '\t' || json[value_pos] == '\r' ||
          json[value_pos] == '\n')) {
    ++value_pos;
  }
  if (json.compare(value_pos, 4, "true") == 0) {
    value = true;
    return true;
  }
  if (json.compare(value_pos, 5, "false") == 0) {
    value = false;
    return true;
  }
  return false;
}

bool parseArmorAvoidanceJson(const std::string& json, ArmorAvoidanceSettings& settings) {
  return parseJsonBool(json, "enabled", settings.enabled) &&
         parseJsonNumber(json, "plate_width_mm", settings.plate_width_mm) &&
         parseJsonNumber(json, "plate_height_mm", settings.plate_height_mm) &&
         parseJsonNumber(json, "center_x_mm", settings.center_x_mm) &&
         parseJsonNumber(json, "center_y_mm", settings.center_y_mm) &&
         parseJsonNumber(json, "radius_mm", settings.radius_mm) &&
         parseJsonNumber(json, "dispersion_sigma_mm", settings.dispersion_sigma_mm);
}

size_t requestContentLength(const std::string& request) {
  const size_t headers_end = request.find("\r\n\r\n");
  if (headers_end == std::string::npos) return 0;
  std::istringstream headers(request.substr(0, headers_end));
  std::string line;
  std::getline(headers, line);
  while (std::getline(headers, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string name = line.substr(0, colon);
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (name != "content-length") continue;
    try {
      return static_cast<size_t>(std::stoul(line.substr(colon + 1)));
    } catch (...) {
      return 0;
    }
  }
  return 0;
}

std::string indexHtml() {
  return R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HFUT Auto Aim Debug</title>
  <style>
    * { box-sizing: border-box; }
    :root {
      --bg: #0d0f12;
      --panel: #171a1f;
      --panel-2: #20242a;
      --border: #303640;
      --muted: #9aa4b2;
      --text: #edf1f7;
      --green: #31d07d;
      --blue: #5cb7ff;
      --orange: #ff9f43;
      --violet: #b78cff;
      --red: #ff5c70;
      --yellow: #ffd166;
    }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Arial, Helvetica, sans-serif;
      font-size: 14px;
    }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1fr) minmax(420px, 500px);
      gap: 12px;
      padding: 12px;
      min-height: 100vh;
    }
    .video-panel {
      min-width: 0;
      background: #050607;
      border: 1px solid var(--border);
      display: flex;
      align-items: flex-start;
      justify-content: center;
      overflow: hidden;
    }
    .video-panel img {
      display: block;
      width: 100%;
      height: auto;
      background: #000;
    }
    .side-panel {
      background: var(--panel);
      border: 1px solid var(--border);
      padding: 12px;
      min-width: 0;
    }
    .topbar {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 10px;
      margin-bottom: 12px;
    }
    h1 { font-size: 18px; line-height: 1.2; margin: 0; font-weight: 700; }
    h2 { font-size: 13px; line-height: 1.2; margin: 0; color: var(--text); font-weight: 700; }
    .subtitle { color: var(--muted); font-size: 12px; margin-top: 4px; }
    .pill {
      flex: 0 0 auto;
      min-width: 78px;
      padding: 6px 8px;
      border-radius: 4px;
      text-align: center;
      border: 1px solid var(--border);
      background: var(--panel-2);
      color: var(--muted);
      font-weight: 700;
      font-size: 12px;
    }
    .pill.ok { color: var(--green); border-color: rgba(49, 208, 125, 0.45); }
    .pill.warn { color: var(--yellow); border-color: rgba(255, 209, 102, 0.45); }
    .pill.bad { color: var(--red); border-color: rgba(255, 92, 112, 0.45); }
    .cards {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
      margin-bottom: 12px;
    }
    .card {
      background: var(--panel-2);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 9px 10px;
      min-width: 0;
    }
    .card .label { color: var(--muted); font-size: 11px; margin-bottom: 5px; }
    .card .value {
      font-size: 18px;
      font-weight: 700;
      line-height: 1.1;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .card .meta { color: var(--muted); font-size: 11px; margin-top: 5px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .chart-block {
      border-top: 1px solid var(--border);
      padding-top: 10px;
      margin-top: 10px;
    }
    .chart-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      margin-bottom: 6px;
    }
    .toggles {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 7px 10px;
      color: var(--muted);
      font-size: 11px;
    }
    .toggle {
      display: inline-flex;
      align-items: center;
      gap: 4px;
      white-space: nowrap;
    }
    .toggle input { width: 13px; height: 13px; margin: 0; }
    canvas {
      display: block;
      width: 100%;
      height: 150px;
      background: #101318;
      border: 1px solid #2b313a;
      border-radius: 5px;
    }
    .toolbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      margin: 8px 0 2px;
      color: var(--muted);
      font-size: 12px;
    }
    button {
      color: var(--text);
      background: var(--panel-2);
      border: 1px solid var(--border);
      border-radius: 4px;
      padding: 5px 9px;
      font: inherit;
      cursor: pointer;
    }
    button:hover { border-color: #566170; }
    details {
      margin-top: 12px;
      border-top: 1px solid var(--border);
      padding-top: 10px;
    }
    summary { cursor: pointer; color: var(--muted); font-weight: 700; }
    .telemetry {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 6px 10px;
      margin-top: 10px;
      font-size: 12px;
    }
    .telemetry span:nth-child(odd) { color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .telemetry span:nth-child(even) { color: var(--text); text-align: right; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; max-width: 260px; }
    .status-line { color: var(--muted); font-size: 12px; margin-top: 10px; }
    .avoidance-block {
      margin-top: 14px;
      border-top: 1px solid var(--border);
      padding-top: 12px;
    }
    .avoidance-description { color: var(--muted); font-size: 11px; line-height: 1.5; margin: 6px 0 9px; }
    .plate-editor {
      display: block;
      width: 100%;
      height: 220px;
      border: 1px solid #2b313a;
      border-radius: 5px;
      background: #101318;
      touch-action: none;
      user-select: none;
    }
    .plate-editor .plate { fill: #26313c; stroke: #d6e1ef; stroke-width: 2; }
    .plate-editor .plate-grid { stroke: rgba(154,164,178,0.19); stroke-width: 1; }
    .plate-editor .avoid-circle { fill: rgba(255,92,112,0.2); stroke: var(--red); stroke-width: 2; }
    .plate-editor .circle-center { stroke: #fff; stroke-width: 1.5; }
    .plate-editor .circle-handle { fill: #ffd166; stroke: #101318; stroke-width: 2; cursor: ew-resize; }
    .plate-editor .axis-label { fill: var(--muted); font: 11px Arial, sans-serif; }
    .avoidance-controls {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
      margin-top: 9px;
    }
    .avoidance-control { display: grid; gap: 4px; min-width: 0; }
    .avoidance-control span { color: var(--muted); font-size: 11px; }
    .avoidance-control input[type="number"] {
      width: 100%; min-width: 0; color: var(--text); background: var(--panel-2);
      border: 1px solid var(--border); border-radius: 4px; padding: 6px 7px; font: inherit;
    }
    .avoidance-enable { color: var(--text); margin-top: 9px; }
    .avoidance-note { color: var(--yellow); font-size: 11px; line-height: 1.45; margin-top: 8px; }
    @media (max-width: 1100px) {
      main { grid-template-columns: 1fr; }
      .side-panel { order: 2; }
    }
    @media (max-width: 520px) {
      main { padding: 8px; }
      .cards { grid-template-columns: 1fr; }
      .chart-head { align-items: flex-start; flex-direction: column; }
      .toggles { justify-content: flex-start; }
    }
  </style>
</head>
<body>
<main>
  <section class="video-panel" aria-label="debug stream">
    <img src="/stream.mjpg" alt="detector stream">
  </section>
  <aside class="side-panel">
    <div class="topbar">
      <div>
        <h1>HFUT Auto Aim Debug</h1>
        <div class="subtitle">Video, gimbal tracking, and fire advice</div>
      </div>
      <div id="track-pill" class="pill">WAIT</div>
    </div>

    <div class="cards">
      <div class="card">
        <div class="label">Vision</div>
        <div id="value-vision" class="value">--</div>
        <div id="meta-vision" class="meta">det / poses / armors / tracked</div>
      </div>
      <div class="card">
        <div class="label">Runtime</div>
        <div id="value-runtime" class="value">--</div>
        <div id="meta-runtime" class="meta">fps / latency</div>
      </div>
      <div class="card">
        <div class="label">Distance</div>
        <div id="value-distance" class="value">--</div>
        <div id="meta-distance" class="meta">command target</div>
      </div>
      <div class="card">
        <div class="label">Fire Advice</div>
        <div id="value-fire" class="value">--</div>
        <div id="meta-fire" class="meta">algorithm only</div>
      </div>
    </div>

    <div class="toolbar">
      <label class="toggle"><input id="pause-history" type="checkbox"> pause curves</label>
      <button id="clear-history" type="button">Clear</button>
    </div>

    <section class="chart-block" aria-label="yaw chart">
      <div class="chart-head">
        <h2>Yaw tracking</h2>
        <div id="yaw-toggles" class="toggles"></div>
      </div>
      <canvas id="chart-yaw"></canvas>
    </section>

    <section class="chart-block" aria-label="pitch chart">
      <div class="chart-head">
        <h2>Pitch tracking</h2>
        <div id="pitch-toggles" class="toggles"></div>
      </div>
      <canvas id="chart-pitch"></canvas>
    </section>

    <section class="chart-block" aria-label="fire advice chart">
      <div class="chart-head">
        <h2>Fire advice</h2>
        <div id="fire-toggles" class="toggles"></div>
      </div>
      <canvas id="chart-fire"></canvas>
    </section>

    <section class="chart-block" aria-label="buff rotation speed chart">
      <div class="chart-head">
        <h2>大符旋转速度</h2>
        <div id="buff-speed-toggles" class="toggles"></div>
      </div>
      <canvas id="chart-buff-speed"></canvas>
    </section>

    <details open>
      <summary>Telemetry</summary>
      <div id="telemetry" class="telemetry"></div>
    </details>
    <section class="avoidance-block" aria-label="装甲板避区参数">
      <div class="chart-head">
        <h2>装甲板避区参数</h2>
        <span id="avoidance-sync" class="status-line">加载中</span>
      </div>
      <div class="avoidance-description">
        在矩形装甲板上拖动红色圆心和黄色把手，设置加固区域的位置与半径；坐标原点在矩形中心，x 向右、y 向上。启用后，参数会传给实际自瞄目标点计算。
      </div>
      <svg id="plate-editor" class="plate-editor" viewBox="0 0 360 240" role="img" aria-label="可拖动的装甲板和加固区域圆形">
        <rect id="plate-rect" class="plate" rx="2"></rect>
        <g id="plate-grid"></g>
        <circle id="avoid-circle" class="avoid-circle" cx="180" cy="120" r="20"></circle>
        <line id="circle-cross-x" class="circle-center" x1="170" y1="120" x2="190" y2="120"></line>
        <line id="circle-cross-y" class="circle-center" x1="180" y1="110" x2="180" y2="130"></line>
        <circle id="circle-handle" class="circle-handle" cx="200" cy="120" r="6"></circle>
        <text id="plate-label" class="axis-label" x="180" y="228" text-anchor="middle">装甲板局部坐标 (mm)</text>
      </svg>
      <div class="avoidance-controls">
        <label class="avoidance-control"><span>板宽 W (mm)</span><input id="plate-width" type="number" min="50" max="500" step="1"></label>
        <label class="avoidance-control"><span>板高 H (mm)</span><input id="plate-height" type="number" min="50" max="400" step="1"></label>
        <label class="avoidance-control"><span>圆心 x (mm)</span><input id="circle-x" type="number" step="1"></label>
        <label class="avoidance-control"><span>圆心 y (mm)</span><input id="circle-y" type="number" step="1"></label>
        <label class="avoidance-control"><span>加固半径 R (mm)</span><input id="circle-radius" type="number" min="1" max="300" step="1"></label>
        <label class="avoidance-control"><span>每轴散布 σ (mm)</span><input id="dispersion-sigma" type="number" min="0" max="100" step="1"></label>
      </div>
      <label class="toggle avoidance-enable"><input id="avoidance-enabled" type="checkbox"> 启用实际避区瞄准</label>
      <div class="avoidance-note">默认板宽高参考当前小装甲板几何；散布 σ 需根据实测弹着数据标定。若没有可用的非加固区，算法会停止瞄准该目标。</div>
    </section>
    <div id="status-note" class="status-line">waiting for status...</div>
  </aside>
</main>
<script>
const HISTORY_SECONDS = 18;
const POLL_MS = 200;
const RAD_TO_DEG = 180 / Math.PI;
const COLORS = {
  feedback: '#5cb7ff',
  command: '#ff9f43',
  target: '#b78cff',
  error: '#ff5c70',
  fire: '#31d07d',
  grid: '#2d3440',
  text: '#edf1f7',
  muted: '#9aa4b2'
};

const CHARTS = [
  {
    id: 'yaw',
    canvas: 'chart-yaw',
    toggles: 'yaw-toggles',
    unit: 'deg',
    series: [
      {key: 'feedback_yaw_deg', label: 'fb_yaw', color: COLORS.feedback, digits: 2},
      {key: 'aligned_feedback_yaw_deg', label: 'align_fb', color: '#65d6ff', digits: 2},
      {key: 'raw_target_yaw_deg', label: 'raw_sp', color: COLORS.target, digits: 2},
      {key: 'target_yaw_deg', label: 'stable_sp', color: '#d3b7ff', digits: 2},
      {key: 'command_yaw_deg', label: 'cmd_yaw', color: COLORS.command, digits: 2},
      {key: 'limiter_yaw_error_deg', label: 'lim_err', color: COLORS.error, digits: 2}
    ]
  },
  {
    id: 'pitch',
    canvas: 'chart-pitch',
    toggles: 'pitch-toggles',
    unit: 'deg',
    series: [
      {key: 'feedback_pitch_deg', label: 'fb_pitch', color: COLORS.feedback, digits: 2},
      {key: 'aligned_feedback_pitch_deg', label: 'align_fb', color: '#65d6ff', digits: 2},
      {key: 'raw_target_pitch_deg', label: 'raw_sp', color: COLORS.target, digits: 2},
      {key: 'target_pitch_deg', label: 'stable_sp', color: '#d3b7ff', digits: 2},
      {key: 'command_pitch_deg', label: 'cmd_pitch', color: COLORS.command, digits: 2},
      {key: 'limiter_pitch_error_deg', label: 'lim_err', color: COLORS.error, digits: 2}
    ]
  },
  {
    id: 'fire',
    canvas: 'chart-fire',
    toggles: 'fire-toggles',
    unit: '0/1',
    fixedRange: [0, 1],
    series: [
      {key: 'fire_advice', label: 'fire_advice', color: COLORS.fire, digits: 0, step: true}
    ]
  },
  {
    id: 'buff-speed',
    canvas: 'chart-buff-speed',
    toggles: 'buff-speed-toggles',
    unit: 'rad/s',
    series: [
      {key: 'buff_rotation_speed_rad_s', label: '模型角速度', color: COLORS.command,
        digits: 2, validWhen: 'buff_rotation_speed_valid'},
      {key: 'buff_angle_delta_speed_rad_s', label: '角度差分', color: COLORS.feedback,
        digits: 2, validWhen: 'buff_angle_delta_speed_valid', dash: [5, 4]}
    ]
  }
];

const TELEMETRY = [
  ['frames', 'frames', 0, ''],
  ['mode', 'mode', 0, ''],
  ['selected', 'selected_id', null, ''],
  ['state', 'track_state', null, ''],
  ['reason', 'reason', null, ''],
  ['enemy', 'enemy_color', null, ''],
  ['camera', 'camera_backend', null, ''],
  ['serial tx', 'serial_tx', null, ''],
  ['serial rx', 'serial_rx', null, ''],
  ['dry run', 'dry_run', null, ''],
  ['fire enabled', 'fire_enabled', null, ''],
  ['distance', 'distance_m', 3, ' m'],
  ['pnp blade distance', 'pnp_blade_distance_m', 3, ' m'],
  ['lim yaw err', 'limiter_yaw_error_deg', 2, ' deg'],
  ['lim pitch err', 'limiter_pitch_error_deg', 2, ' deg'],
  ['fire blocked', 'fire_blocked_by_limiter', null, ''],
  ['cmd yaw vel', 'command_yaw_vel_deg_s', 1, ' deg/s'],
  ['cmd pitch vel', 'command_pitch_vel_deg_s', 1, ' deg/s'],
  ['cmd yaw acc', 'command_yaw_acc_deg_s2', 1, ' deg/s^2'],
  ['cmd pitch acc', 'command_pitch_acc_deg_s2', 1, ' deg/s^2'],
  ['大符模型角速度', 'buff_rotation_speed_rad_s', 2, ' rad/s', 'buff_rotation_speed_valid'],
  ['角度差分速度', 'buff_angle_delta_speed_rad_s', 2, ' rad/s', 'buff_angle_delta_speed_valid'],
  ['运动拟合有效', 'buff_motion_fit_valid', null, ''],
  ['正弦拟合有效', 'buff_sine_fit_valid', null, ''],
  ['拟合代价', 'buff_motion_fit_cost', 4, ' rad^2', 'buff_motion_fit_valid'],
  ['仅模型预测', 'buff_model_predicted', null, ''],
  ['等待运动拟合', 'buff_requires_warmup', null, '']
];

const defaultVisible = CHARTS.flatMap(chart => chart.series.map(series => series.key));
const savedVisibleSeries = [...loadJson('hfut-visible-series', defaultVisible)];
if (!localStorage.getItem('hfut-buff-speed-series-v1')) {
  savedVisibleSeries.push('buff_rotation_speed_rad_s', 'buff_angle_delta_speed_rad_s');
  localStorage.setItem('hfut-buff-speed-series-v1', '1');
}
let visibleSeries = new Set(savedVisibleSeries);
let history = [];
let latestStatus = null;
let avoidanceSettings = {
  enabled: false,
  plate_width_mm: 135,
  plate_height_mm: 56,
  center_x_mm: 0,
  center_y_mm: 0,
  radius_mm: 20,
  dispersion_sigma_mm: 10
};
let avoidanceSaveTimer = null;
let plateDragMode = null;

function loadJson(key, fallback) {
  try {
    const raw = localStorage.getItem(key);
    return raw ? JSON.parse(raw) : fallback;
  } catch (_) {
    return fallback;
  }
}

function saveVisible() {
  localStorage.setItem('hfut-visible-series', JSON.stringify([...visibleSeries]));
}

function finite(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function fmt(value, digits = 1, unit = '') {
  if (value === undefined || value === null) return '--';
  if (typeof value === 'boolean') return value ? '1' : '0';
  if (typeof value === 'number') return Number.isFinite(value) ? `${value.toFixed(digits)}${unit}` : '--';
  return String(value);
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function normalizeAvoidanceSettings(settings) {
  settings.plate_width_mm = clamp(finite(settings.plate_width_mm, 135), 50, 500);
  settings.plate_height_mm = clamp(finite(settings.plate_height_mm, 56), 50, 400);
  settings.center_x_mm = clamp(finite(settings.center_x_mm), -settings.plate_width_mm / 2, settings.plate_width_mm / 2);
  settings.center_y_mm = clamp(finite(settings.center_y_mm), -settings.plate_height_mm / 2, settings.plate_height_mm / 2);
  settings.radius_mm = clamp(finite(settings.radius_mm, 20), 1, 300);
  settings.dispersion_sigma_mm = clamp(finite(settings.dispersion_sigma_mm, 10), 0, 100);
  settings.enabled = Boolean(settings.enabled);
  return settings;
}

function drawAvoidancePlate() {
  avoidanceSettings = normalizeAvoidanceSettings(avoidanceSettings);
  const {plate_width_mm: widthMm, plate_height_mm: heightMm} = avoidanceSettings;
  const scale = Math.min(270 / widthMm, 180 / heightMm);
  const plateWidth = widthMm * scale;
  const plateHeight = heightMm * scale;
  const left = (360 - plateWidth) / 2;
  const top = 10 + (190 - plateHeight) / 2;
  const centerX = left + plateWidth / 2;
  const centerY = top + plateHeight / 2;
  const circleX = centerX + avoidanceSettings.center_x_mm * scale;
  const circleY = centerY - avoidanceSettings.center_y_mm * scale;
  const circleRadius = avoidanceSettings.radius_mm * scale;

  const plate = document.getElementById('plate-rect');
  plate.setAttribute('x', left);
  plate.setAttribute('y', top);
  plate.setAttribute('width', plateWidth);
  plate.setAttribute('height', plateHeight);

  const grid = document.getElementById('plate-grid');
  grid.textContent = '';
  for (let i = 1; i < 4; ++i) {
    const x = left + plateWidth * i / 4;
    const y = top + plateHeight * i / 4;
    const vertical = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    vertical.setAttribute('class', 'plate-grid');
    vertical.setAttribute('x1', x); vertical.setAttribute('x2', x);
    vertical.setAttribute('y1', top); vertical.setAttribute('y2', top + plateHeight);
    grid.appendChild(vertical);
    const horizontal = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    horizontal.setAttribute('class', 'plate-grid');
    horizontal.setAttribute('x1', left); horizontal.setAttribute('x2', left + plateWidth);
    horizontal.setAttribute('y1', y); horizontal.setAttribute('y2', y);
    grid.appendChild(horizontal);
  }

  const circle = document.getElementById('avoid-circle');
  circle.setAttribute('cx', circleX); circle.setAttribute('cy', circleY);
  circle.setAttribute('r', circleRadius);
  const crossX = document.getElementById('circle-cross-x');
  crossX.setAttribute('x1', circleX - 6); crossX.setAttribute('x2', circleX + 6);
  crossX.setAttribute('y1', circleY); crossX.setAttribute('y2', circleY);
  const crossY = document.getElementById('circle-cross-y');
  crossY.setAttribute('x1', circleX); crossY.setAttribute('x2', circleX);
  crossY.setAttribute('y1', circleY - 6); crossY.setAttribute('y2', circleY + 6);
  const handle = document.getElementById('circle-handle');
  handle.setAttribute('cx', circleX + circleRadius); handle.setAttribute('cy', circleY);
  circle.setAttribute('opacity', avoidanceSettings.enabled ? '1' : '0.3');
  crossX.setAttribute('opacity', avoidanceSettings.enabled ? '1' : '0.3');
  crossY.setAttribute('opacity', avoidanceSettings.enabled ? '1' : '0.3');
  handle.setAttribute('opacity', avoidanceSettings.enabled ? '1' : '0.3');

  for (const [id, key] of [
    ['plate-width', 'plate_width_mm'], ['plate-height', 'plate_height_mm'],
    ['circle-x', 'center_x_mm'], ['circle-y', 'center_y_mm'],
    ['circle-radius', 'radius_mm'], ['dispersion-sigma', 'dispersion_sigma_mm']
  ]) {
    const input = document.getElementById(id);
    if (document.activeElement !== input) input.value = Number(avoidanceSettings[key].toFixed(1));
  }
  document.getElementById('avoidance-enabled').checked = avoidanceSettings.enabled;
}

function scheduleAvoidanceSave() {
  drawAvoidancePlate();
  document.getElementById('avoidance-sync').textContent = '待同步';
  clearTimeout(avoidanceSaveTimer);
  avoidanceSaveTimer = setTimeout(async () => {
    try {
      const response = await fetch('/avoidance.json', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(avoidanceSettings),
        cache: 'no-store'
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      avoidanceSettings = normalizeAvoidanceSettings(await response.json());
      drawAvoidancePlate();
      document.getElementById('avoidance-sync').textContent = '已同步';
    } catch (error) {
      document.getElementById('avoidance-sync').textContent = '同步失败';
    }
  }, 100);
}

async function loadAvoidanceSettings() {
  try {
    const response = await fetch('/avoidance.json', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    avoidanceSettings = normalizeAvoidanceSettings(await response.json());
    drawAvoidancePlate();
    document.getElementById('avoidance-sync').textContent = '已同步';
  } catch (_) {
    drawAvoidancePlate();
    document.getElementById('avoidance-sync').textContent = '本地编辑';
  }
}

function svgPoint(event) {
  const svg = document.getElementById('plate-editor');
  const point = new DOMPoint(event.clientX, event.clientY);
  return point.matrixTransform(svg.getScreenCTM().inverse());
}

function plateGeometry() {
  const scale = Math.min(270 / avoidanceSettings.plate_width_mm, 180 / avoidanceSettings.plate_height_mm);
  return {scale, centerX: 180, centerY: 105};
}

document.getElementById('plate-editor').addEventListener('pointerdown', event => {
  const point = svgPoint(event);
  const {scale, centerX, centerY} = plateGeometry();
  const circleX = centerX + avoidanceSettings.center_x_mm * scale;
  const circleY = centerY - avoidanceSettings.center_y_mm * scale;
  const radius = avoidanceSettings.radius_mm * scale;
  const handleDistance = Math.hypot(point.x - (circleX + radius), point.y - circleY);
  const circleDistance = Math.hypot(point.x - circleX, point.y - circleY);
  if (handleDistance <= 14) plateDragMode = 'radius';
  else if (circleDistance <= radius + 12) plateDragMode = 'center';
  else plateDragMode = 'center';
  event.currentTarget.setPointerCapture(event.pointerId);
  if (plateDragMode === 'center') {
    avoidanceSettings.center_x_mm = clamp((point.x - centerX) / scale, -avoidanceSettings.plate_width_mm / 2, avoidanceSettings.plate_width_mm / 2);
    avoidanceSettings.center_y_mm = clamp((centerY - point.y) / scale, -avoidanceSettings.plate_height_mm / 2, avoidanceSettings.plate_height_mm / 2);
  } else {
    avoidanceSettings.radius_mm = clamp(Math.hypot(point.x - circleX, point.y - circleY) / scale, 1, 300);
  }
  scheduleAvoidanceSave();
});

document.getElementById('plate-editor').addEventListener('pointermove', event => {
  if (!plateDragMode) return;
  const point = svgPoint(event);
  const {scale, centerX, centerY} = plateGeometry();
  if (plateDragMode === 'center') {
    avoidanceSettings.center_x_mm = clamp((point.x - centerX) / scale, -avoidanceSettings.plate_width_mm / 2, avoidanceSettings.plate_width_mm / 2);
    avoidanceSettings.center_y_mm = clamp((centerY - point.y) / scale, -avoidanceSettings.plate_height_mm / 2, avoidanceSettings.plate_height_mm / 2);
  } else {
    const circleX = centerX + avoidanceSettings.center_x_mm * scale;
    const circleY = centerY - avoidanceSettings.center_y_mm * scale;
    avoidanceSettings.radius_mm = clamp(Math.hypot(point.x - circleX, point.y - circleY) / scale, 1, 300);
  }
  scheduleAvoidanceSave();
});

for (const eventName of ['pointerup', 'pointercancel', 'lostpointercapture']) {
  document.getElementById('plate-editor').addEventListener(eventName, () => { plateDragMode = null; });
}

for (const [id, key] of [
  ['plate-width', 'plate_width_mm'], ['plate-height', 'plate_height_mm'],
  ['circle-x', 'center_x_mm'], ['circle-y', 'center_y_mm'],
  ['circle-radius', 'radius_mm'], ['dispersion-sigma', 'dispersion_sigma_mm']
]) {
  document.getElementById(id).addEventListener('input', event => {
    const parsed = Number(event.target.value);
    if (!Number.isFinite(parsed)) return;
    avoidanceSettings[key] = parsed;
    scheduleAvoidanceSave();
  });
  document.getElementById(id).addEventListener('blur', drawAvoidancePlate);
}
document.getElementById('avoidance-enabled').addEventListener('change', event => {
  avoidanceSettings.enabled = event.target.checked;
  scheduleAvoidanceSave();
});

function normalizeStatus(status) {
  status.fire_advice = Boolean(status.fire_advice ?? status.fire ?? false);
  status.yaw_error_deg = finite(
      status.yaw_error_deg,
      finite(status.command_yaw_deg) - finite(status.feedback_yaw_deg));
  status.pitch_error_deg = finite(
      status.pitch_error_deg,
      finite(status.command_pitch_deg) - finite(status.feedback_pitch_deg));
  status.raw_target_yaw_deg = finite(status.raw_target_yaw_deg, finite(status.target_yaw_deg, finite(status.command_yaw_deg)));
  status.raw_target_pitch_deg = finite(status.raw_target_pitch_deg, finite(status.target_pitch_deg, finite(status.command_pitch_deg)));
  status.aligned_feedback_yaw_deg = finite(status.aligned_feedback_yaw_deg, finite(status.feedback_yaw_deg));
  status.aligned_feedback_pitch_deg = finite(status.aligned_feedback_pitch_deg, finite(status.feedback_pitch_deg));
  status.feedback_alignment_delta_yaw_deg = finite(
      status.feedback_alignment_delta_yaw_deg,
      finite(status.feedback_yaw_deg) - finite(status.aligned_feedback_yaw_deg));
  status.feedback_alignment_delta_pitch_deg = finite(
      status.feedback_alignment_delta_pitch_deg,
      finite(status.feedback_pitch_deg) - finite(status.aligned_feedback_pitch_deg));
  status.target_yaw_deg = finite(status.target_yaw_deg, finite(status.command_yaw_deg));
  status.target_pitch_deg = finite(status.target_pitch_deg, finite(status.command_pitch_deg));
  status.limiter_yaw_error_deg = finite(
      status.limiter_yaw_error_deg,
      finite(status.target_yaw_deg) - finite(status.command_yaw_deg));
  status.limiter_pitch_error_deg = finite(
      status.limiter_pitch_error_deg,
      finite(status.target_pitch_deg) - finite(status.command_pitch_deg));
  status.command_yaw_vel_deg_s = finite(status.command_yaw_vel_rad_s) * RAD_TO_DEG;
  status.command_pitch_vel_deg_s = finite(status.command_pitch_vel_rad_s) * RAD_TO_DEG;
  status.command_yaw_acc_deg_s2 = finite(status.command_yaw_acc_rad_s2) * RAD_TO_DEG;
  status.command_pitch_acc_deg_s2 = finite(status.command_pitch_acc_rad_s2) * RAD_TO_DEG;
  status.distance_m = finite(status.distance_m);
  status.pnp_blade_distance_m = finite(status.pnp_blade_distance_m);
  status.pnp_first_distance_m = finite(status.pnp_first_distance_m);
  return status;
}

function buildToggles() {
  for (const chart of CHARTS) {
    const root = document.getElementById(chart.toggles);
    root.textContent = '';
    for (const series of chart.series) {
      const label = document.createElement('label');
      label.className = 'toggle';
      label.style.color = series.color;
      const input = document.createElement('input');
      input.type = 'checkbox';
      input.checked = visibleSeries.has(series.key);
      input.addEventListener('change', () => {
        if (input.checked) visibleSeries.add(series.key);
        else visibleSeries.delete(series.key);
        saveVisible();
        drawAllCharts();
      });
      label.append(input, document.createTextNode(series.label));
      root.appendChild(label);
    }
  }
}

function updateCards(status) {
  document.getElementById('value-vision').textContent =
      `${status.detections ?? 0}/${status.poses ?? 0}/${status.armors ?? 0}/${status.tracked ?? 0}`;
  document.getElementById('meta-vision').textContent =
      `selected=${status.selected_id ?? 'none'} state=${status.track_state ?? 'none'}`;

  document.getElementById('value-runtime').textContent =
      `${fmt(finite(status.fps), 1)} fps`;
  document.getElementById('meta-runtime').textContent =
      `latency=${fmt(finite(status.latency_ms), 1, ' ms')} fb_age=${fmt(finite(status.feedback_age_ms), 0, ' ms')} align=${fmt(finite(status.aligned_feedback_age_ms), 1, ' ms')}`;

  document.getElementById('value-distance').textContent =
      `${fmt(finite(status.distance_m), 2, ' m')}`;
  document.getElementById('meta-distance').textContent =
      `plan=${fmt(finite(status.distance_m), 2, ' m')} pnp=${fmt(finite(status.pnp_blade_distance_m), 2, ' m')}`;

  document.getElementById('value-fire').textContent = status.fire ? 'SEND' : (status.fire_advice ? 'BLOCK' : 'HOLD');
  document.getElementById('meta-fire').textContent =
      `sp=${status.fire_advice ? 1 : 0} sent=${status.fire ? 1 : 0} gate=${status.fire_blocked_by_limiter ? 1 : 0}`;

  const pill = document.getElementById('track-pill');
  if (status.fire) {
    pill.textContent = 'FIRE';
    pill.className = 'pill ok';
  } else if (status.fire_advice) {
    pill.textContent = 'BLOCK';
    pill.className = 'pill warn';
  } else if ((status.tracked ?? 0) > 0) {
    pill.textContent = 'TRACK';
    pill.className = 'pill ok';
  } else if ((status.detections ?? 0) > 0) {
    pill.textContent = 'DETECT';
    pill.className = 'pill warn';
  } else {
    pill.textContent = 'WAIT';
    pill.className = 'pill bad';
  }
}

function updateTelemetry(status) {
  const root = document.getElementById('telemetry');
  root.textContent = '';
  for (const [label, key, digits, unit, validWhen] of TELEMETRY) {
    const name = document.createElement('span');
    const value = document.createElement('span');
    name.textContent = label;
    value.textContent = validWhen && !status[validWhen]
      ? '--'
      : (digits === null ? fmt(status[key]) : fmt(finite(status[key]), digits, unit));
    root.append(name, value);
  }
}

function addSample(status) {
  const t = performance.now() / 1000;
  const sample = {t};
  for (const chart of CHARTS) {
    for (const series of chart.series) {
      const value = status[series.key];
      sample[series.key] = series.validWhen && !status[series.validWhen]
        ? NaN
        : (typeof value === 'boolean' ? (value ? 1 : 0) : finite(value, NaN));
    }
  }
  history.push(sample);
  const cutoff = t - HISTORY_SECONDS;
  while (history.length > 1 && history[0].t < cutoff) history.shift();
}

function drawAllCharts() {
  for (const chart of CHARTS) drawChart(chart);
}

function drawChart(chart) {
  const canvas = document.getElementById(chart.canvas);
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(260, Math.floor(rect.width));
  const height = Math.max(120, Math.floor(rect.height || 150));
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(height * dpr)) {
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const left = 46;
  const right = 12;
  const top = 12;
  const bottom = 24;
  const plotW = Math.max(1, width - left - right);
  const plotH = Math.max(1, height - top - bottom);
  const seriesList = chart.series.filter(series => visibleSeries.has(series.key));

  ctx.fillStyle = '#101318';
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth = 1;
  ctx.strokeRect(left, top, plotW, plotH);

  if (history.length < 2 || seriesList.length === 0) {
    ctx.fillStyle = COLORS.muted;
    ctx.font = '12px Arial';
    ctx.fillText('waiting for samples', left + 8, top + 22);
    return;
  }

  const tMax = history[history.length - 1].t;
  const tMin = Math.max(history[0].t, tMax - HISTORY_SECONDS);
  let yMin = Infinity;
  let yMax = -Infinity;
  for (const sample of history) {
    for (const series of seriesList) {
      const value = finite(sample[series.key], NaN);
      if (!Number.isFinite(value)) continue;
      yMin = Math.min(yMin, value);
      yMax = Math.max(yMax, value);
    }
  }
  if (chart.fixedRange) {
    yMin = chart.fixedRange[0];
    yMax = chart.fixedRange[1];
  } else if (!Number.isFinite(yMin) || !Number.isFinite(yMax)) {
    yMin = -1;
    yMax = 1;
  } else if (Math.abs(yMax - yMin) < 1e-6) {
    yMin -= 1;
    yMax += 1;
  } else {
    const pad = (yMax - yMin) * 0.14;
    yMin -= pad;
    yMax += pad;
  }

  const xOf = t => left + ((t - tMin) / Math.max(0.001, tMax - tMin)) * plotW;
  const yOf = v => top + (1 - (v - yMin) / Math.max(0.001, yMax - yMin)) * plotH;

  ctx.font = '11px Arial';
  ctx.fillStyle = COLORS.muted;
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let i = 0; i <= 2; ++i) {
    const frac = i / 2;
    const y = top + plotH * frac;
    const value = yMax - (yMax - yMin) * frac;
    ctx.strokeStyle = i === 0 || i === 2 ? COLORS.grid : 'rgba(154,164,178,0.22)';
    ctx.beginPath();
    ctx.moveTo(left, y);
    ctx.lineTo(left + plotW, y);
    ctx.stroke();
    ctx.fillText(chart.fixedRange ? value.toFixed(0) : value.toFixed(1), left - 7, y);
  }
  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';
  ctx.fillText(`-${HISTORY_SECONDS}s`, left, height - 7);
  ctx.textAlign = 'right';
  ctx.fillText('now', left + plotW, height - 7);

  for (const series of seriesList) {
    ctx.strokeStyle = series.color;
    ctx.lineWidth = series.key.includes('error') ? 1.4 : 2;
    ctx.setLineDash(series.dash || []);
    ctx.beginPath();
    let started = false;
    let prevX = 0;
    let prevY = 0;
    for (const sample of history) {
      if (sample.t < tMin) continue;
      const value = finite(sample[series.key], NaN);
      if (!Number.isFinite(value)) {
        started = false;
        continue;
      }
      const x = xOf(sample.t);
      const y = yOf(value);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else if (series.step) {
        ctx.lineTo(x, prevY);
        ctx.lineTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
      prevX = x;
      prevY = y;
    }
    if (started) ctx.stroke();
    ctx.setLineDash([]);
    if (history.length > 0) {
      const last = history[history.length - 1];
      const value = finite(last[series.key], NaN);
      if (Number.isFinite(value)) {
        const lx = xOf(last.t);
        const ly = yOf(value);
        ctx.fillStyle = series.color;
        ctx.beginPath();
        ctx.arc(lx, ly, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }
}

async function refreshStatus() {
  const note = document.getElementById('status-note');
  try {
    const response = await fetch('/status.json', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    latestStatus = normalizeStatus(await response.json());
    updateCards(latestStatus);
    updateTelemetry(latestStatus);
    if (!document.getElementById('pause-history').checked) addSample(latestStatus);
    drawAllCharts();
    note.textContent = `frames=${latestStatus.frames ?? 0} tx=${latestStatus.serial_tx ?? ''} rx=${latestStatus.serial_rx ?? ''}`;
  } catch (error) {
    note.textContent = `status error: ${error}`;
    const pill = document.getElementById('track-pill');
    pill.textContent = 'ERROR';
    pill.className = 'pill bad';
  }
}

document.getElementById('clear-history').addEventListener('click', () => {
  history = [];
  if (latestStatus) addSample(latestStatus);
  drawAllCharts();
});
window.addEventListener('resize', drawAllCharts);
buildToggles();
drawAvoidancePlate();
loadAvoidanceSettings();
setInterval(refreshStatus, POLL_MS);
refreshStatus();
</script>
</body>
</html>
)HTML";
}

bool sendAll(int fd, const void* data, size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  while (size > 0) {
#ifdef MSG_NOSIGNAL
    const ssize_t sent = ::send(fd, bytes, size, MSG_NOSIGNAL);
#else
    const ssize_t sent = ::send(fd, bytes, size, 0);
#endif
    if (sent <= 0) return false;
    bytes += sent;
    size -= static_cast<size_t>(sent);
  }
  return true;
}

bool sendString(int fd, const std::string& text) {
  return sendAll(fd, text.data(), text.size());
}

std::string httpHeader(const std::string& status, const std::string& content_type,
                       size_t content_length = 0, bool close = true) {
  std::ostringstream out;
  out << "HTTP/1.1 " << status << "\r\n"
      << "Content-Type: " << content_type << "\r\n"
      << "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
      << "Pragma: no-cache\r\n";
  if (content_length > 0) out << "Content-Length: " << content_length << "\r\n";
  if (close) out << "Connection: close\r\n";
  out << "\r\n";
  return out.str();
}

std::string requestPath(const std::string& request) {
  const size_t method_end = request.find(' ');
  if (method_end == std::string::npos) return "/";
  const size_t path_end = request.find(' ', method_end + 1);
  if (path_end == std::string::npos) return "/";
  const std::string target = request.substr(method_end + 1, path_end - method_end - 1);
  const size_t query = target.find('?');
  return query == std::string::npos ? target : target.substr(0, query);
}

}  // namespace

DebugMjpegServer::DebugMjpegServer(DebugMjpegServerConfig config)
    : config_(std::move(config)) {}

DebugMjpegServer::~DebugMjpegServer() { stop(); }

bool DebugMjpegServer::start() {
  if (running_.load()) return true;
  if (!bindListenSocket()) return false;
  running_.store(true);
  accept_thread_ = std::thread(&DebugMjpegServer::acceptLoop, this);
  return true;
}

void DebugMjpegServer::stop() {
  if (!running_.exchange(false)) return;
  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }
  frame_cv_.notify_all();
  if (accept_thread_.joinable()) accept_thread_.join();
  std::vector<std::thread> clients;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients.swap(client_threads_);
  }
  for (auto& thread : clients) {
    if (thread.joinable()) thread.join();
  }
}

std::string DebugMjpegServer::errorMessage() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return error_message_;
}

std::string DebugMjpegServer::url() const {
  const std::string host = (config_.host.empty() || config_.host == "0.0.0.0")
      ? "<nuc-ip>"
      : config_.host;
  return "http://" + host + ":" + std::to_string(config_.port) + "/";
}

void DebugMjpegServer::publish(const cv::Mat& image, const DebugMjpegStatus& status) {
  if (!running_.load() || image.empty()) return;

  cv::Mat encoded_image = image;
  cv::Mat resized;
  if (config_.max_width > 0 && image.cols > config_.max_width) {
    const double scale = static_cast<double>(config_.max_width) / image.cols;
    cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    encoded_image = resized;
  }

  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,
                             std::max(30, std::min(config_.jpeg_quality, 95))};
  std::vector<uint8_t> jpeg;
  if (!cv::imencode(".jpg", encoded_image, jpeg, params)) return;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_jpeg_ = std::move(jpeg);
    latest_status_ = status;
    ++frame_version_;
  }
  frame_cv_.notify_all();
}

ArmorAvoidanceSettings DebugMjpegServer::armorAvoidanceSettings() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return armor_avoidance_settings_;
}

bool DebugMjpegServer::setArmorAvoidanceSettings(const ArmorAvoidanceSettings& settings) {
  const bool valid =
      std::isfinite(settings.plate_width_mm) && settings.plate_width_mm >= 50.0 &&
      settings.plate_width_mm <= 500.0 && std::isfinite(settings.plate_height_mm) &&
      settings.plate_height_mm >= 50.0 && settings.plate_height_mm <= 400.0 &&
      std::isfinite(settings.center_x_mm) &&
      std::abs(settings.center_x_mm) <= settings.plate_width_mm / 2.0 &&
      std::isfinite(settings.center_y_mm) &&
      std::abs(settings.center_y_mm) <= settings.plate_height_mm / 2.0 &&
      std::isfinite(settings.radius_mm) && settings.radius_mm >= 1.0 &&
      settings.radius_mm <= 300.0 && std::isfinite(settings.dispersion_sigma_mm) &&
      settings.dispersion_sigma_mm >= 0.0 && settings.dispersion_sigma_mm <= 100.0;
  if (!valid) return false;
  std::lock_guard<std::mutex> lock(state_mutex_);
  armor_avoidance_settings_ = settings;
  return true;
}

bool DebugMjpegServer::bindListenSocket() {
  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = std::string("socket failed: ") + std::strerror(errno);
    return false;
  }

  int reuse = 1;
  ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(config_.port);
  if (config_.host.empty() || config_.host == "0.0.0.0" || config_.host == "*") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, config_.host.c_str(), &addr.sin_addr) != 1) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = "web host must be an IPv4 address, got: " + config_.host;
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = std::string("bind failed: ") + std::strerror(errno);
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (::listen(server_fd_, 8) != 0) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = std::string("listen failed: ") + std::strerror(errno);
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }
  return true;
}

void DebugMjpegServer::acceptLoop() {
  while (running_.load()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(server_fd_, &read_fds);
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    const int ready = ::select(server_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0) continue;

    const int client_fd = ::accept(server_fd_, nullptr, nullptr);
    if (client_fd < 0) continue;
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_threads_.emplace_back(&DebugMjpegServer::handleClient, this, client_fd);
  }
}

void DebugMjpegServer::handleClient(int client_fd) {
  timeval send_timeout{};
  send_timeout.tv_sec = 1;
  ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
  timeval recv_timeout{};
  recv_timeout.tv_sec = 2;
  ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

  std::string request;
  char buffer[2048] = {};
  size_t request_size = 0;
  while (request_size < 8192 && request.find("\r\n\r\n") == std::string::npos) {
    const ssize_t n = ::recv(client_fd, buffer, sizeof(buffer), 0);
    if (n <= 0) break;
    request.append(buffer, static_cast<size_t>(n));
    request_size += static_cast<size_t>(n);
  }
  const size_t headers_end = request.find("\r\n\r\n");
  if (headers_end == std::string::npos) {
    ::close(client_fd);
    return;
  }
  const size_t method_end = request.find(' ');
  const std::string method = method_end == std::string::npos ? "" : request.substr(0, method_end);
  const std::string path = requestPath(request);

  if (path == "/avoidance.json" && method == "POST") {
    const size_t content_length = requestContentLength(request);
    if (content_length == 0 || content_length > 1024) {
      const std::string body = "invalid content length\n";
      sendString(client_fd, httpHeader("400 Bad Request", "text/plain", body.size()));
      sendString(client_fd, body);
      ::close(client_fd);
      return;
    }
    const size_t body_start = headers_end + 4;
    while (request.size() - body_start < content_length && request_size < 8192) {
      const ssize_t n = ::recv(client_fd, buffer, sizeof(buffer), 0);
      if (n <= 0) break;
      request.append(buffer, static_cast<size_t>(n));
      request_size += static_cast<size_t>(n);
    }
    if (request.size() - body_start < content_length) {
      const std::string body = "incomplete request body\n";
      sendString(client_fd, httpHeader("400 Bad Request", "text/plain", body.size()));
      sendString(client_fd, body);
      ::close(client_fd);
      return;
    }
    ArmorAvoidanceSettings settings = armorAvoidanceSettings();
    const std::string body_text = request.substr(body_start, content_length);
    if (!parseArmorAvoidanceJson(body_text, settings) || !setArmorAvoidanceSettings(settings)) {
      const std::string body = "invalid avoidance settings\n";
      sendString(client_fd, httpHeader("400 Bad Request", "text/plain", body.size()));
      sendString(client_fd, body);
      ::close(client_fd);
      return;
    }
    const std::string body = armorAvoidanceJson(armorAvoidanceSettings());
    sendString(client_fd, httpHeader("200 OK", "application/json", body.size()));
    sendString(client_fd, body);
    ::close(client_fd);
    return;
  }

  if (path == "/" || path == "/index.html") {
    const std::string body = indexHtml();
    sendString(client_fd, httpHeader("200 OK", "text/html; charset=utf-8", body.size()));
    sendString(client_fd, body);
    ::close(client_fd);
    return;
  }

  if (path == "/status.json") {
    DebugMjpegStatus status;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      status = latest_status_;
    }
    const std::string body = statusJson(status);
    sendString(client_fd, httpHeader("200 OK", "application/json", body.size()));
    sendString(client_fd, body);
    ::close(client_fd);
    return;
  }

  if (path == "/avoidance.json" && method == "GET") {
    const std::string body = armorAvoidanceJson(armorAvoidanceSettings());
    sendString(client_fd, httpHeader("200 OK", "application/json", body.size()));
    sendString(client_fd, body);
    ::close(client_fd);
    return;
  }

  if (path == "/snapshot.jpg") {
    std::vector<uint8_t> jpeg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      jpeg = latest_jpeg_;
    }
    if (jpeg.empty()) {
      const std::string body = "no frame yet\n";
      sendString(client_fd, httpHeader("503 Service Unavailable", "text/plain", body.size()));
      sendString(client_fd, body);
    } else {
      sendString(client_fd, httpHeader("200 OK", "image/jpeg", jpeg.size()));
      sendAll(client_fd, jpeg.data(), jpeg.size());
    }
    ::close(client_fd);
    return;
  }

  if (path == "/stream.mjpg") {
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n"
           << "Content-Type: multipart/x-mixed-replace; boundary=" << kBoundary << "\r\n"
           << "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
           << "Pragma: no-cache\r\n\r\n";
    if (!sendString(client_fd, header.str())) {
      ::close(client_fd);
      return;
    }

    uint64_t seen_version = 0;
    while (running_.load()) {
      std::vector<uint8_t> jpeg;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        frame_cv_.wait_for(lock, std::chrono::seconds(2), [&] {
          return !running_.load() || frame_version_ != seen_version;
        });
        if (!running_.load()) break;
        if (latest_jpeg_.empty() || frame_version_ == seen_version) continue;
        seen_version = frame_version_;
        jpeg = latest_jpeg_;
      }

      std::ostringstream part;
      part << "--" << kBoundary << "\r\n"
           << "Content-Type: image/jpeg\r\n"
           << "Content-Length: " << jpeg.size() << "\r\n\r\n";
      if (!sendString(client_fd, part.str())) break;
      if (!sendAll(client_fd, jpeg.data(), jpeg.size())) break;
      if (!sendString(client_fd, "\r\n")) break;
    }
    ::close(client_fd);
    return;
  }

  const std::string body = "not found\n";
  sendString(client_fd, httpHeader("404 Not Found", "text/plain", body.size()));
  sendString(client_fd, body);
  ::close(client_fd);
}

}  // namespace hfut::io
