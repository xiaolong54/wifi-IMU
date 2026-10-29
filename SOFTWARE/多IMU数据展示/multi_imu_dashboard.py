#!/usr/bin/env python3
"""Multi-IMU dashboard UI that consumes APIs from multi_imu_core."""

from __future__ import annotations

import math
import time
import threading
from typing import List, Optional, Tuple
import pygame
from pygame.locals import DOUBLEBUF, OPENGL, QUIT, KEYDOWN, K_ESCAPE, K_UP, K_DOWN, K_c, K_x, K_e, K_o, K_u
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

from multi_imu_core import MultiImuService

WIN_W, WIN_H = 1200, 760
DASHBOARD_VERSION = "1.2.0"
OTA_FIRMWARE_PATH = "build/vqf_esp32.bin"


def _format_remain(minutes: int) -> str:
    """Format remaining battery time for display."""
    if minutes < 0:
        return "--"
    if minutes >= 60:
        h = minutes // 60
        m = minutes % 60
        return f"{h}h{m:02d}m"
    return f"{minutes}m"


def _run_ota(svc: MultiImuService, node_id: str, path: str, status: dict) -> None:
    """Background thread: OTA upload, update status dict."""
    status[node_id] = "OTA:上传中..."
    try:
        ok = svc.ota_update(node_id, path, timeout=90)
        status[node_id] = "OTA:成功 ✓" if ok else "OTA:失败 ✗"
    except Exception as e:
        status[node_id] = f"OTA:异常 {e}"


def quat_to_euler(q):
    w, x, y, z = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = 2.0 * (w * y - z * x)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def quat_to_gl_matrix(q):
    w, x, y, z = q
    return [
        1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 0,
        2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 0,
        2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y), 0,
        0, 0, 0, 1,
    ]


def draw_board():
    hx, hy, hz = 1.0, 0.6, 0.07
    glBegin(GL_QUADS)
    glColor3f(0.06, 0.58, 0.26)
    glVertex3f(-hx, -hy, hz); glVertex3f(hx, -hy, hz); glVertex3f(hx, hy, hz); glVertex3f(-hx, hy, hz)
    glColor3f(0.02, 0.35, 0.12)
    glVertex3f(-hx, -hy, -hz); glVertex3f(-hx, hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(hx, -hy, -hz)
    glEnd()


def draw_text(x, y, text, font, color=(240, 240, 240)):
    s = font.render(text, True, color).convert_alpha()
    data = pygame.image.tostring(s, "RGBA", True)
    w, h = s.get_size()
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
    glWindowPos2f(float(x), float(WIN_H - y - h))
    glDrawPixels(w, h, GL_RGBA, GL_UNSIGNED_BYTE, data)


# ============================================================================
# Calibration visualization helpers
# ============================================================================

THETA_BINS = 12   # elevation bins (0..π)
PHI_BINS = 24     # azimuth bins (0..2π)
COVERAGE_THRESHOLD = 0.80
NORM_STD_THRESHOLD = 0.05  # 5% of mean


def _samples_to_np(samples: List[List[float]]) -> np.ndarray:
    if not samples:
        return np.empty((0, 3))
    return np.array(samples, dtype=np.float32)


def _apply_cal(raw: np.ndarray, hard_iron: List[float],
               soft_iron: List[List[float]]) -> np.ndarray:
    hi = np.array(hard_iron, dtype=np.float32)
    si = np.array(soft_iron, dtype=np.float32)
    return (raw - hi) @ si.T


def _compute_norms(raw: np.ndarray) -> np.ndarray:
    if len(raw) == 0:
        return np.empty(0)
    return np.linalg.norm(raw, axis=1)


def _compute_coverage(raw: np.ndarray) -> Tuple[np.ndarray, float]:
    """Return (heatmap[THETA_BINS, PHI_BINS], coverage_ratio)."""
    hmap = np.zeros((THETA_BINS, PHI_BINS), dtype=np.int32)
    if len(raw) == 0:
        return hmap, 0.0
    norms = np.linalg.norm(raw, axis=1, keepdims=True)
    norms = np.clip(norms, 1e-9, None)
    unit = raw / norms
    theta = np.arccos(np.clip(unit[:, 2], -1, 1))            # 0..π
    phi = np.arctan2(unit[:, 1], unit[:, 0]) + np.pi         # 0..2π
    ti = np.clip((theta / np.pi * THETA_BINS).astype(int), 0, THETA_BINS - 1)
    pi_ = np.clip((phi / (2 * np.pi) * PHI_BINS).astype(int), 0, PHI_BINS - 1)
    for t, p in zip(ti, pi_):
        hmap[t, p] += 1
    filled = np.count_nonzero(hmap)
    coverage = filled / (THETA_BINS * PHI_BINS)
    return hmap, coverage


def _lowest_coverage_direction(hmap: np.ndarray) -> str:
    """Find empty/lowest bin region and return human-readable rotation hint."""
    if hmap.sum() == 0:
        return "缓慢旋转设备，覆盖所有方向"
    region_sums = {}
    half_t = THETA_BINS // 2
    half_p = PHI_BINS // 2
    region_sums["上 (Z+)"] = hmap[:half_t // 2, :].sum()
    region_sums["下 (Z-)"] = hmap[-half_t // 2:, :].sum()
    region_sums["前 (Y+)"] = hmap[:, half_p // 2: half_p].sum()
    region_sums["后 (Y-)"] = hmap[:, -half_p // 2:].sum()
    region_sums["左 (X-)"] = hmap[:, : half_p // 4].sum()
    region_sums["右 (X+)"] = hmap[:, half_p - half_p // 4: half_p + half_p // 4].sum()
    weakest = min(region_sums, key=region_sums.get)
    return f"请向 {weakest} 方向旋转"


def _setup_ortho():
    """Switch to 2D orthographic projection for HUD drawing."""
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, WIN_W, WIN_H, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()


def _restore_projection():
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()


def draw_cal_3d_pointcloud(raw: np.ndarray, calibrated: Optional[np.ndarray],
                           cam_angle: float):
    """Draw 3D magnetometer point cloud in the left viewport area."""
    glViewport(0, 200, 660, 560)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 660 / 560, 0.1, 200.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    dist = 5.0
    cx = dist * math.cos(math.radians(cam_angle)) * math.cos(math.radians(25))
    cy = dist * math.sin(math.radians(cam_angle)) * math.cos(math.radians(25))
    cz = dist * math.sin(math.radians(25))
    gluLookAt(cx, cy, cz, 0, 0, 0, 0, 0, 1)

    # Axis lines
    glBegin(GL_LINES)
    glColor3f(0.7, 0.2, 0.2); glVertex3f(0, 0, 0); glVertex3f(2, 0, 0)
    glColor3f(0.2, 0.7, 0.2); glVertex3f(0, 0, 0); glVertex3f(0, 2, 0)
    glColor3f(0.2, 0.2, 0.7); glVertex3f(0, 0, 0); glVertex3f(0, 0, 2)
    glEnd()

    glPointSize(3.0)
    if len(raw) > 0:
        # auto-scale to fit
        scale = 1.0
        maxv = np.abs(raw).max()
        if maxv > 0:
            scale = 2.0 / maxv

        # Raw points (gray)
        glBegin(GL_POINTS)
        glColor3f(0.55, 0.55, 0.55)
        for p in raw:
            glVertex3f(p[0] * scale, p[1] * scale, p[2] * scale)
        glEnd()

        # Calibrated points (green)
        if calibrated is not None and len(calibrated) > 0:
            glBegin(GL_POINTS)
            glColor3f(0.2, 0.9, 0.3)
            for p in calibrated:
                glVertex3f(p[0] * scale, p[1] * scale, p[2] * scale)
            glEnd()

    # Restore full viewport
    glViewport(0, 0, WIN_W, WIN_H)


def draw_cal_heatmap(hmap: np.ndarray, coverage: float, font, font_lg):
    """Draw spherical coverage heatmap as 2D colored grid in top-right area."""
    ox, oy = 700, 60
    cell_w = 18
    cell_h = 14
    max_count = max(hmap.max(), 1)

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    _setup_ortho()

    for ti in range(THETA_BINS):
        for pi in range(PHI_BINS):
            v = hmap[ti, pi]
            if v == 0:
                r, g, b = 0.12, 0.12, 0.18
            else:
                t = min(v / max_count, 1.0)
                # blue → cyan → green → yellow
                if t < 0.33:
                    f = t / 0.33
                    r, g, b = 0.0, f * 0.6, 0.8 - f * 0.3
                elif t < 0.66:
                    f = (t - 0.33) / 0.33
                    r, g, b = f * 0.8, 0.6 + f * 0.3, 0.5 - f * 0.5
                else:
                    f = (t - 0.66) / 0.34
                    r, g, b = 0.8 + f * 0.2, 0.9 - f * 0.3, 0.0

            x1 = ox + pi * cell_w
            y1 = oy + ti * cell_h
            glColor3f(r, g, b)
            glBegin(GL_QUADS)
            glVertex2f(x1, y1)
            glVertex2f(x1 + cell_w - 1, y1)
            glVertex2f(x1 + cell_w - 1, y1 + cell_h - 1)
            glVertex2f(x1, y1 + cell_h - 1)
            glEnd()

    # Border
    bw = PHI_BINS * cell_w
    bh = THETA_BINS * cell_h
    glColor3f(0.4, 0.4, 0.4)
    glBegin(GL_LINE_LOOP)
    glVertex2f(ox, oy)
    glVertex2f(ox + bw, oy)
    glVertex2f(ox + bw, oy + bh)
    glVertex2f(ox, oy + bh)
    glEnd()

    _restore_projection()

    # Labels
    cov_color = (80, 255, 120) if coverage >= COVERAGE_THRESHOLD else (255, 200, 100)
    draw_text(ox, oy - 24, f"球面覆盖热力图  Coverage: {coverage*100:.1f}%", font_lg, cov_color)
    draw_text(ox, oy + bh + 4, "← φ (方位角 0~360°) →", font, (160, 160, 160))

    glEnable(GL_DEPTH_TEST)


def draw_cal_norm_curve(norms: np.ndarray, font, font_lg):
    """Draw magnetometer norm time-series in bottom area."""
    ox, oy = 40, 560
    w, h = 500, 140

    if len(norms) == 0:
        draw_text(ox, oy, "范数曲线: 等待数据...", font, (160, 160, 160))
        return

    mean_n = norms.mean()
    std_n = norms.std()
    rel_std = (std_n / mean_n * 100) if mean_n > 0 else 0.0

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    _setup_ortho()

    # Background
    glColor4f(0.08, 0.08, 0.14, 0.85)
    glBegin(GL_QUADS)
    glVertex2f(ox, oy); glVertex2f(ox + w, oy)
    glVertex2f(ox + w, oy + h); glVertex2f(ox, oy + h)
    glEnd()

    # Border
    glColor3f(0.3, 0.3, 0.4)
    glBegin(GL_LINE_LOOP)
    glVertex2f(ox, oy); glVertex2f(ox + w, oy)
    glVertex2f(ox + w, oy + h); glVertex2f(ox, oy + h)
    glEnd()

    # Plot norms
    n_show = min(len(norms), 300)
    plot_norms = norms[-n_show:]
    lo = plot_norms.min() * 0.95
    hi = plot_norms.max() * 1.05
    if hi - lo < 0.01:
        lo -= 0.05
        hi += 0.05
    margin = 5

    # Mean line
    my = oy + h - margin - (mean_n - lo) / (hi - lo) * (h - 2 * margin)
    glColor3f(0.3, 0.6, 0.3)
    glBegin(GL_LINES)
    glVertex2f(ox, my); glVertex2f(ox + w, my)
    glEnd()

    # Norm curve
    glColor3f(0.3, 0.85, 1.0)
    glBegin(GL_LINE_STRIP)
    for i, n in enumerate(plot_norms):
        x = ox + margin + i * (w - 2 * margin) / max(n_show - 1, 1)
        y = oy + h - margin - (n - lo) / (hi - lo) * (h - 2 * margin)
        glVertex2f(x, y)
    glEnd()

    _restore_projection()

    # Labels
    std_color = (80, 255, 120) if rel_std < (NORM_STD_THRESHOLD * 100) else (255, 200, 100)
    draw_text(ox, oy - 24, f"‖m‖ 范数曲线  Mean={mean_n:.3f}  Std={std_n:.4f}  RelStd={rel_std:.1f}%",
              font_lg, std_color)
    draw_text(ox + w + 10, oy, f"max={plot_norms.max():.3f}", font, (160, 160, 160))
    draw_text(ox + w + 10, oy + h - 16, f"min={plot_norms.min():.3f}", font, (160, 160, 160))

    glEnable(GL_DEPTH_TEST)


def check_stop_condition(coverage: float, norms: np.ndarray) -> Tuple[bool, str]:
    """Check if calibration stop conditions are met."""
    if len(norms) < 50:
        return False, ""
    mean_n = norms.mean()
    rel_std = (norms.std() / mean_n) if mean_n > 0 else 1.0
    cov_ok = coverage >= COVERAGE_THRESHOLD
    std_ok = rel_std < NORM_STD_THRESHOLD
    if cov_ok and std_ok:
        return True, "✓ 校准完成条件已满足！可以停止采集"
    parts = []
    if not cov_ok:
        parts.append(f"覆盖率 {coverage*100:.0f}% < {COVERAGE_THRESHOLD*100:.0f}%")
    if not std_ok:
        parts.append(f"范数偏差 {rel_std*100:.1f}% > {NORM_STD_THRESHOLD*100:.0f}%")
    return False, "未满足: " + ", ".join(parts)


def main():
    svc = MultiImuService()
    svc.start()

    pygame.init()
    pygame.font.init()
    pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Multi ESP32 IMU Dashboard")

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glClearColor(0.07, 0.08, 0.12, 1.0)

    font = pygame.font.SysFont("DejaVu Sans Mono", 18)
    font_lg = pygame.font.SysFont("DejaVu Sans Mono", 22, bold=True)

    clock = pygame.time.Clock()
    selected_node_id = None
    cam_angle = 30.0  # auto-rotating camera for cal 3D view
    cal_view_node_id = None
    cal_view_active = False
    ota_status: dict = {}  # node_id -> status string

    running = True
    while running:
        devices = svc.list_devices()

        # Resolve selected index from node_id
        selected_idx = 0
        if selected_node_id:
            for i, d in enumerate(devices):
                if d.node_id == selected_node_id:
                    selected_idx = i
                    break
            else:
                # Previously selected device disappeared; reset
                selected_node_id = None
        if devices:
            selected_idx = max(0, min(selected_idx, len(devices) - 1))
            selected_node_id = devices[selected_idx].node_id
            svc.select_device(selected_node_id)
        selected = svc.get_selected()

        for ev in pygame.event.get():
            if ev.type == QUIT:
                running = False
            elif ev.type == KEYDOWN:
                if ev.key == K_ESCAPE:
                    running = False
                elif ev.key == K_UP and devices:
                    selected_idx = max(0, selected_idx - 1)
                    selected_node_id = devices[selected_idx].node_id
                elif ev.key == K_DOWN and devices:
                    selected_idx = min(len(devices) - 1, selected_idx + 1)
                    selected_node_id = devices[selected_idx].node_id
                elif ev.key == K_c and selected:
                    svc.send_cal_start(selected.node_id)
                    cal_view_node_id = selected.node_id
                    cal_view_active = True
                elif ev.key == K_x:
                    # X 始终由用户决定退出校准界面，不再依赖设备状态。
                    target = None
                    if cal_view_node_id:
                        for d in devices:
                            if d.node_id == cal_view_node_id:
                                target = d
                                break
                    cal_view_active = False
                    cal_view_node_id = None
                    # 仅在采集中时请求设备停止采集
                    if target and target.cal_state in ("start", "progress", "collecting", "computing"):
                        svc.send_cal_stop(target.node_id)
                elif ev.key == K_e and selected:
                    svc.send_cal_erase(selected.node_id)
                elif ev.key == K_o and selected:
                    svc.send_shutdown(selected.node_id)
                elif ev.key == K_u and selected:
                    # OTA upgrade: firmware must be at OTA_FIRMWARE_PATH
                    node = selected.node_id
                    if ota_status.get(node, "") != "OTA:上传中...":
                        t = threading.Thread(
                            target=_run_ota,
                            args=(svc, node, OTA_FIRMWARE_PATH, ota_status),
                            daemon=True)
                        t.start()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # 校准界面完全由本地 UI 状态控制，避免任何自动退出。
        cal_device = None
        if cal_view_node_id:
            for d in devices:
                if d.node_id == cal_view_node_id:
                    cal_device = d
                    break
        cal_active = cal_view_active

        if cal_active:
            # === Calibration visualization mode ===
            cam_angle += 0.3  # auto-rotate

            raw_np = _samples_to_np(cal_device.cal_raw_samples) if cal_device else np.empty((0, 3))
            norms = _compute_norms(raw_np)
            hmap, coverage = _compute_coverage(raw_np)

            # Compute calibrated points if params available
            cal_np = None
            if cal_device and cal_device.cal_hard_iron and cal_device.cal_soft_iron:
                cal_np = _apply_cal(raw_np, cal_device.cal_hard_iron,
                                    cal_device.cal_soft_iron)

            # 1) 3D point cloud (left area)
            glDisable(GL_LIGHTING)
            draw_cal_3d_pointcloud(raw_np, cal_np, cam_angle)

            # 2) Heatmap (top-right)
            draw_cal_heatmap(hmap, coverage, font, font_lg)

            # 3) Norm curve (bottom-left)
            draw_cal_norm_curve(norms, font, font_lg)

            # HUD text overlay
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

            # Device list (compact)
            draw_text(14, 10, "Devices (UP/DOWN select)", font_lg)
            y = 36
            now = time.time()
            for i, d in enumerate(devices):
                marker = ">" if i == selected_idx else " "
                conn = "ON" if d.connected else "OFF"
                line = f"{marker} {d.node_id} C:{conn}"
                color = (80, 255, 120) if d.connected else (255, 130, 130)
                draw_text(14, y, line, font, color)
                # OTA status on same row
                ota_s = ota_status.get(d.node_id, "")
                if ota_s:
                    ota_col = (255, 200, 80) if "上传" in ota_s else ((80, 255, 120) if "成功" in ota_s else (255, 100, 100))
                    draw_text(200, y, ota_s, font, ota_col)
                y += 20
            if cal_device:
                draw_text(14, 530,
                          f"校准状态: {cal_device.cal_state.upper()}  "
                          f"样本: {len(cal_device.cal_raw_samples)}  "
                          f"进度: {cal_device.cal_progress_pct}%",
                          font_lg, (100, 200, 255))
            else:
                draw_text(14, 530,
                          "校准视图保持中: 设备暂时离线，等待重连...",
                          font_lg, (255, 200, 100))

            # Stop condition & guidance
            stop_ok, stop_msg = check_stop_condition(coverage, norms)
            if stop_ok:
                draw_text(14, 720, stop_msg, font_lg, (80, 255, 120))
            elif stop_msg:
                draw_text(14, 720, stop_msg, font, (255, 200, 100))

            guidance = _lowest_coverage_direction(hmap)
            draw_text(14, 742, guidance, font, (200, 200, 255))

            draw_text(14, 460, "Commands: C=CalStart  X=ExitCalView  E=CalErase  U=OTA", font)
            draw_text(14, 478, f"Dashboard v{DASHBOARD_VERSION}  FW: {cal_device.firmware_version if cal_device else '--'}", font, (140, 140, 180))
            glDisable(GL_BLEND)

        else:
            # === Normal mode ===
            # 3D selected IMU
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(45, WIN_W / WIN_H, 0.1, 60.0)
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            gluLookAt(3.2, 2.4, 2.8, 0, 0, 0, 0, 0, 1)

            if selected:
                glPushMatrix()
                glMultMatrixf(quat_to_gl_matrix(selected.quat))
                draw_board()
                glPopMatrix()

            # HUD
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

            draw_text(14, 10, "Devices (UP/DOWN select)", font_lg)
            y = 40
            now = time.time()
            for i, d in enumerate(devices):
                age = now - d.last_seen
                marker = ">" if i == selected_idx else " "
                conn = "ON" if d.connected else "OFF"
                batt = f"{d.battery_percent}%" if d.battery_percent is not None else "--"
                src = d.power_source
                charge = "CHG" if d.charging else "BAT"
                remain = _format_remain(d.battery_remain_min)
                hz = f"{d.pkt_rate_hz:.0f}Hz" if d.pkt_rate_hz > 0 else "--Hz"
                fw = d.firmware_version or "--"
                line = f"{marker} {d.node_id} {d.ip} C:{conn} {hz} B:{batt} R:{remain} {charge} v{fw} age:{age:4.1f}s"
                color = (80, 255, 120) if d.connected else (255, 130, 130)
                draw_text(14, y, line, font, color)
                # OTA status
                ota_s = ota_status.get(d.node_id, "")
                if ota_s:
                    ota_col = (255, 200, 80) if "上传" in ota_s else ((80, 255, 120) if "成功" in ota_s else (255, 100, 100))
                    draw_text(14, y + 13, ota_s, font, ota_col)
                    y += 13
                y += 24

            y = 420
            draw_text(14, y, "Commands: C=CalStart  X=CalStop  E=CalErase  O=PowerOff  U=OTA", font)
            y += 26

            if selected:
                roll, pitch, yaw = quat_to_euler(selected.quat)
                draw_text(14, y, f"Selected: {selected.node_id} ({selected.ip})", font_lg)
                y += 32
                draw_text(14, y, f"Euler: Roll={roll:+7.2f} Pitch={pitch:+7.2f} Yaw={yaw:+7.2f}", font)
                y += 24
                draw_text(14, y, f"Battery: {selected.battery_voltage}V {selected.battery_percent}%  Remain:{_format_remain(selected.battery_remain_min)}  Charging:{selected.charging}", font)
                y += 24
                draw_text(14, y, f"Rate: {selected.pkt_rate_hz:.1f} Hz  FW: {selected.firmware_version or '--'}  Key: {selected.key_voltage}V pressed={selected.key_pressed}", font)
                y += 24
                draw_text(14, y, f"Cal: {selected.cal_state}  Msg: {selected.cal_message}", font)
            else:
                draw_text(14, y, "No IMU discovered yet...", font_lg, (255, 200, 100))

            glEnable(GL_DEPTH_TEST)
            glDisable(GL_BLEND)
        pygame.display.flip()
        clock.tick(60)

    svc.stop()
    pygame.quit()


if __name__ == "__main__":
    main()
