"""
无人机地面站系统 - 流畅优化版
- 缓存航线规划结果，避免重复计算
- 减少页面闪烁和卡顿
- 支持地图点击设置 A/B 点
- 默认坐标：A(32.2323,118.749), B(32.2344,118.749)
"""

import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins
import json, os, math, heapq, random
from datetime import datetime
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
import hashlib

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统", layout="wide", page_icon="✈️")

# ==================== 缓存工具 ====================
def get_cache_key(start, end, obstacles, flight_height, safety_radius, strategy):
    """生成缓存键，避免重复计算"""
    key_data = {
        "start": start, "end": end,
        "obstacles": [(o["points"], o["height"]) for o in obstacles],
        "fh": flight_height, "sr": safety_radius, "strategy": strategy
    }
    return hashlib.md5(json.dumps(key_data, sort_keys=True).encode()).hexdigest()

# ==================== 文件持久化 ====================
CONFIG_FILE = "obstacle_config.json"
WP_FILE = "waypoints.json"

def _wp_save():
    ss = st.session_state
    try:
        with open(WP_FILE, "w", encoding="utf-8") as f:
            json.dump({
                "start_point": ss.start_point,
                "end_point": ss.end_point,
                "flight_height": ss.flight_height,
                "safety_radius": ss.safety_radius
            }, f, ensure_ascii=False, indent=2)
    except Exception:
        pass

def _wp_load():
    if os.path.exists(WP_FILE):
        try:
            with open(WP_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return None

def _obs_save():
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump({"obstacles": st.session_state.obstacles}, f, ensure_ascii=False, indent=2)
    except Exception:
        pass

def _obs_load():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                return json.load(f).get("obstacles", [])
        except Exception:
            pass
    return []

# ==================== 默认值 ====================
DEFAULT_A = {"lat": 32.2323, "lng": 118.7490, "height": 0}
DEFAULT_B = {"lat": 32.2344, "lng": 118.7490, "height": 0}
DEFAULT_FH = 10
DEFAULT_SR = 8

# ==================== Session State ====================
def init_session_state():
    if "inited" in st.session_state:
        return
    wp = _wp_load()
    st.session_state.inited = True
    st.session_state.heartbeat_count = 0
    st.session_state.obstacles = _obs_load()
    st.session_state.start_point = wp.get("start_point", DEFAULT_A.copy()) if wp else DEFAULT_A.copy()
    st.session_state.end_point = wp.get("end_point", DEFAULT_B.copy()) if wp else DEFAULT_B.copy()
    st.session_state.flight_height = wp.get("flight_height", DEFAULT_FH) if wp else DEFAULT_FH
    st.session_state.safety_radius = wp.get("safety_radius", DEFAULT_SR) if wp else DEFAULT_SR
    st.session_state.bypass_strategy = "best"
    st.session_state.planned_route = []
    st.session_state.route_analysis = {}
    st.session_state.setting_mode = None
    st.session_state.obstacles_loaded = True
    st.session_state.map_key = 0
    st.session_state.new_obstacle_height = 60
    st.session_state.auto_flight_enabled = False
    st.session_state.flight_paused = False
    st.session_state.flight_progress = 0.0
    st.session_state.current_waypoint_idx = 0
    st.session_state.flight_remaining_dist = 0.0
    st.session_state.flight_battery = 100
    st.session_state.flight_drone_pos = None
    st.session_state.flight_time_elapsed = 0
    st.session_state.flight_speed = 8.0
    st.session_state.comm_logs = []
    st.session_state.link_delay = 25
    st.session_state.link_loss = 0.1
    st.session_state.mission_started = False
    # 用于触发重新规划的版本号
    st.session_state.route_version = 0

init_session_state()

# ==================== 回调函数（仅更新状态，不立即 rerun） ====================
def on_start_lat_change():
    st.session_state.start_point["lat"] = st.session_state.start_lat
    _wp_save()
    st.session_state.route_version += 1

def on_start_lng_change():
    st.session_state.start_point["lng"] = st.session_state.start_lng
    _wp_save()
    st.session_state.route_version += 1

def on_end_lat_change():
    st.session_state.end_point["lat"] = st.session_state.end_lat
    _wp_save()
    st.session_state.route_version += 1

def on_end_lng_change():
    st.session_state.end_point["lng"] = st.session_state.end_lng
    _wp_save()
    st.session_state.route_version += 1

def on_fh_change():
    st.session_state.flight_height = st.session_state.fh_input
    _wp_save()
    st.session_state.route_version += 1

def on_sr_change():
    st.session_state.safety_radius = st.session_state.sr_input
    _wp_save()
    st.session_state.route_version += 1

# ==================== GCJ-02 <-> WGS-84 ====================
_AE = 6378245.0
_EE = 0.00669342162296594323
_PI = math.pi

def _ooc(lat, lng):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

def _tl(lng, lat):
    r = -100+2*lng+3*lat+0.2*lat*lat+0.1*lng*lat+0.2*math.sqrt(abs(lng))
    r += (20*math.sin(6*lng*_PI)+20*math.sin(2*lng*_PI))*2/3
    r += (20*math.sin(lat*_PI)+40*math.sin(lat/3*_PI))*2/3
    r += (160*math.sin(lat/12*_PI)+320*math.sin(lat*_PI/30))*2/3
    return r

def _tg(lng, lat):
    r = 300+lng+2*lat+0.1*lng*lng+0.1*lng*lat+0.1*math.sqrt(abs(lng))
    r += (20*math.sin(6*lng*_PI)+20*math.sin(2*lng*_PI))*2/3
    r += (20*math.sin(lng*_PI)+40*math.sin(lng/3*_PI))*2/3
    r += (150*math.sin(lng/12*_PI)+300*math.sin(lng/30*_PI))*2/3
    return r

def _d(lat, lng):
    dl = _tl(lng-105, lat-35)
    dg = _tg(lng-105, lat-35)
    mg = math.sin(lat*_PI/180)
    mg = 1 - _EE*mg*mg
    sq = math.sqrt(mg)
    return dl*180/((_AE*(1-_EE))/(mg*sq)*_PI), dg*180/(_AE/sq*math.cos(lat*_PI/180)*_PI)

def gcj2wgs(lat, lng):
    if _ooc(lat, lng):
        return float(lat), float(lng)
    dl, dg = _d(lat, lng)
    return float(lat-dl), float(lng-dg)

def wgs2gcj(lat, lng):
    if _ooc(lat, lng):
        return float(lat), float(lng)
    dl, dg = _d(lat, lng)
    return float(lat+dl), float(lng+dg)

# ==================== 米制投影 ====================
def get_ref():
    ss = st.session_state
    return ((ss.start_point["lat"]+ss.end_point["lat"])/2,
            (ss.start_point["lng"]+ss.end_point["lng"])/2)

def ll2m(lat, lng, rl, rg):
    return ((lng-rg)*math.cos(math.radians(rl))*111320, (lat-rl)*111320)

def m2ll(x, y, rl, rg):
    return (y/111320+rl, x/(math.cos(math.radians(rl))*111320)+rg)

def hdist(a, b):
    R = 6371000
    f1, f2 = math.radians(a[0]), math.radians(b[0])
    dp = math.radians(b[0]-a[0])
    dl = math.radians(b[1]-a[1])
    return R*2*math.atan2(math.sqrt(math.sin(dp/2)**2+math.cos(f1)*math.cos(f2)*math.sin(dl/2)**2),
                          math.sqrt(1-(math.sin(dp/2)**2+math.cos(f1)*math.cos(f2)*math.sin(dl/2)**2)))

def plen(path):
    return sum(hdist(path[i], path[i+1]) for i in range(len(path)-1))

# ==================== 安全缓冲区 ====================
def build_union(obs, fh, sr, rl, rg):
    polys = []
    for o in obs:
        if o.get("height", 30) <= fh:
            continue
        pts = o.get("points", [])
        if len(pts) < 3:
            continue
        try:
            xy = [ll2m(p[0], p[1], rl, rg) for p in pts]
            poly = Polygon(xy)
            if not poly.is_valid:
                poly = poly.buffer(0)
            if poly.is_valid and poly.area > 0:
                polys.append(poly.buffer(float(sr)+3.0))
        except:
            continue
    if not polys:
        return None
    u = unary_union(polys)
    return u if not u.is_empty else None

def _seg_collides(ax, ay, bx, by, union_geom):
    seg = LineString([(ax, ay), (bx, by)])
    if seg.length < 1e-6:
        return union_geom.distance(Point(ax, ay)) < 0.1
    seg_buf = seg.buffer(0.1, cap_style=2, join_style=2)
    inter = seg_buf.intersection(union_geom)
    return inter.area > 1e-6

def _seg_free(ax, ay, bx, by, union_geom):
    return not _seg_collides(ax, ay, bx, by, union_geom)

def _direct_blocked(sx, sy, ex, ey, union_geom):
    return _seg_collides(sx, sy, ex, ey, union_geom)

# ==================== Visibility Graph ====================
def _side(px, py, ax, ay, bx, by):
    return (bx-ax)*(py-ay) - (by-ay)*(px-ax)

def _push_out(px, py, tx, ty, union_geom):
    if union_geom.distance(Point(px, py)) > 0.1:
        return px, py
    vl = math.hypot(tx-px, ty-py) or 1
    vx, vy = (tx-px)/vl, (ty-py)/vl
    for d in range(2, 150, 2):
        nx, ny = px - vx*d, py - vy*d
        if union_geom.distance(Point(nx, ny)) > 0.2:
            return nx, ny
    return px, py

def _nodes_sided(union_geom, sx, sy, ex, ey, side):
    nodes = [(sx, sy), (ex, ey)]
    geoms = [union_geom] if union_geom.geom_type == 'Polygon' else [g for g in union_geom.geoms if g.geom_type == 'Polygon']
    for g in geoms:
        for cx, cy in list(g.exterior.coords)[:-1]:
            s = _side(cx, cy, sx, sy, ex, ey)
            if side == 'left' and s > -1.0:
                nodes.append((cx, cy))
            elif side == 'right' and s < 1.0:
                nodes.append((cx, cy))
            elif side == 'both':
                nodes.append((cx, cy))
    seen = set()
    out = []
    for n in nodes:
        k = (round(n[0], 1), round(n[1], 1))
        if k not in seen:
            seen.add(k)
            out.append(n)
    return out

def _dijkstra(nodes, union_geom):
    n = len(nodes)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            ax, ay = nodes[i]
            bx, by = nodes[j]
            if _seg_free(ax, ay, bx, by, union_geom):
                d = math.hypot(bx-ax, by-ay)
                adj[i].append((j, d))
                adj[j].append((i, d))
    INF = float('inf')
    dist = [INF]*n
    prev = [-1]*n
    dist[0] = 0.0
    heap = [(0.0, 0)]
    while heap:
        cost, u = heapq.heappop(heap)
        if cost > dist[u]:
            continue
        if u == 1:
            break
        for v, w in adj[u]:
            nc = cost + w
            if nc < dist[v]:
                dist[v] = nc
                prev[v] = u
                heapq.heappush(heap, (nc, v))
    if dist[1] == INF:
        return None
    path = []
    cur = 1
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return [nodes[i] for i in path]

def _smooth(path_m, union_geom):
    if len(path_m) <= 2:
        return path_m
    out = [path_m[0]]
    i = 0
    while i < len(path_m)-1:
        j = len(path_m)-1
        while j > i+1:
            if _seg_free(*path_m[i], *path_m[j], union_geom):
                break
            j -= 1
        out.append(path_m[j])
        i = j
    return out

def _plan_side(sx, sy, ex, ey, union, side):
    sx2, sy2 = _push_out(sx, sy, ex, ey, union)
    ex2, ey2 = _push_out(ex, ey, sx, sy, union)
    nodes = _nodes_sided(union, sx2, sy2, ex2, ey2, side)
    if len(nodes) < 2:
        return None
    pm = _dijkstra(nodes, union)
    if not pm or len(pm) < 2:
        return None
    pm = _smooth(pm, union)
    pm[0] = (sx, sy)
    pm[-1] = (ex, ey)
    return pm

def _fallback(sx, sy, ex, ey, union):
    L = math.hypot(ex-sx, ey-sy)
    if L < 1e-6:
        return [(sx, sy), (ex, ey)]
    dx, dy = (ex-sx)/L, (ey-sy)/L
    best, bl = None, float('inf')
    for px, py in [(-dy, dx), (dy, -dx)]:
        max_proj = 0.0
        geoms = [union] if union.geom_type == 'Polygon' else list(union.geoms)
        for g in geoms:
            if hasattr(g, 'exterior'):
                for c in g.exterior.coords:
                    proj = (c[0]-sx)*px + (c[1]-sy)*py
                    if proj > max_proj:
                        max_proj = proj
        off = max_proj + 5.0
        for _ in range(15):
            cand = [(sx, sy),
                    (sx+dx*L*0.35+px*off, sy+dy*L*0.35+py*off),
                    (sx+dx*L*0.65+px*off, sy+dy*L*0.65+py*off),
                    (ex, ey)]
            ok = all(_seg_free(cand[k][0], cand[k][1], cand[k+1][0], cand[k+1][1], union) for k in range(3))
            if ok:
                tl = sum(math.hypot(cand[k+1][0]-cand[k][0], cand[k+1][1]-cand[k][1]) for k in range(3))
                if tl < bl:
                    bl = tl
                    best = cand
                break
            off += 12.0
    return best or [(sx, sy), (ex, ey)]

# ==================== 缓存化规划函数 ====================
@st.cache_data(ttl=3600, show_spinner="计算最优航线中...")
def compute_route(start, end, obstacles_tuple, flight_height, safety_radius, strategy):
    """实际计算航线，使用缓存避免重复计算"""
    start_pt = (start["lat"], start["lng"])
    end_pt = (end["lat"], end["lng"])
    # 还原障碍物列表
    obs_list = [{"points": pts, "height": h} for (pts, h) in obstacles_tuple]

    analysis = {
        "total_distance": 0,
        "obstacles_encountered": [],
        "bypass_count": 0,
        "fly_over_count": 0,
        "route_points": [],
        "strategy_used": ""
    }

    for o in obs_list:
        h = o.get("height", 30)
        if h > flight_height:
            analysis["obstacles_encountered"].append({"height": h, "decision": "绕行"})
        else:
            analysis["fly_over_count"] += 1
            analysis["obstacles_encountered"].append({"height": h, "decision": "飞跃(低)"})

    rl, rg = ((start_pt[0]+end_pt[0])/2, (start_pt[1]+end_pt[1])/2)
    union = build_union(obs_list, flight_height, safety_radius, rl, rg)

    if union is None or union.is_empty:
        route = [start_pt, end_pt]
        analysis.update(total_distance=hdist(start_pt, end_pt), strategy_used="直线（无障碍）", route_points=route)
        return route, analysis

    sx, sy = ll2m(start_pt[0], start_pt[1], rl, rg)
    ex, ey = ll2m(end_pt[0], end_pt[1], rl, rg)

    if not _direct_blocked(sx, sy, ex, ey, union):
        route = [start_pt, end_pt]
        analysis.update(total_distance=hdist(start_pt, end_pt), strategy_used="直线（不碰障碍物）", route_points=route)
        return route, analysis

    path_m = None
    strat_name = ""
    if strategy == "left":
        path_m = _plan_side(sx, sy, ex, ey, union, "left")
        strat_name = "左侧绕行"
    elif strategy == "right":
        path_m = _plan_side(sx, sy, ex, ey, union, "right")
        strat_name = "右侧绕行"
    else:
        pm_l = _plan_side(sx, sy, ex, ey, union, "left")
        pm_r = _plan_side(sx, sy, ex, ey, union, "right")
        def ml(pm):
            return sum(math.hypot(pm[i+1][0]-pm[i][0], pm[i+1][1]-pm[i][1]) for i in range(len(pm)-1)) if pm else float('inf')
        ll, lr = ml(pm_l), ml(pm_r)
        if pm_l and ll <= lr:
            path_m = pm_l
            strat_name = f"最佳（左侧{ll:.0f}m ≤ 右侧{lr:.0f}m）"
        elif pm_r:
            path_m = pm_r
            strat_name = f"最佳（右侧{lr:.0f}m < 左侧{ll:.0f}m）"

    if path_m is None:
        path_m = _plan_side(sx, sy, ex, ey, union, "both")
        strat_name += "（全向兜底）"
    if path_m is None:
        path_m = _fallback(sx, sy, ex, ey, union)
        strat_name = "偏移兜底"

    route = [m2ll(x, y, rl, rg) for x, y in path_m]
    nbp = max(0, len(route)-2)
    analysis.update(
        total_distance=plen(route),
        bypass_count=nbp,
        strategy_used=f"{strat_name}（{nbp}个绕行点）",
        route_points=route
    )
    return route, analysis

def plan_route():
    """触发重新规划（由版本号变化自动调用）"""
    ss = st.session_state
    # 将障碍物转换为可哈希的元组
    obstacles_tuple = tuple((tuple(tuple(p) for p in o["points"]), o["height"]) for o in ss.obstacles)
    route, analysis = compute_route(
        ss.start_point, ss.end_point, obstacles_tuple,
        ss.flight_height, ss.safety_radius, ss.bypass_strategy
    )
    ss.planned_route = route
    ss.route_analysis = analysis
    ss.map_key += 1
    return route, analysis

# ==================== 飞行控制 ====================
def reset_flight():
    ss = st.session_state
    ss.auto_flight_enabled = False
    ss.flight_paused = False
    ss.flight_progress = 0.0
    ss.current_waypoint_idx = 0
    ss.flight_battery = 100
    ss.flight_time_elapsed = 0
    ss.flight_remaining_dist = ss.route_analysis.get("total_distance", 0)
    ss.flight_drone_pos = ss.planned_route[0] if ss.planned_route else None
    ss.mission_started = False

def add_log(direction, message):
    ts = datetime.now().strftime("%H:%M:%S")
    st.session_state.comm_logs.insert(0, {"time": ts, "direction": direction, "message": message})
    if len(st.session_state.comm_logs) > 100:
        st.session_state.comm_logs.pop()

def step_forward():
    ss = st.session_state
    route = ss.planned_route
    if not route:
        return
    if ss.current_waypoint_idx >= len(route)-1:
        ss.auto_flight_enabled = False
        add_log("FCU→OBC→GCS", "MISSION_COMPLETE")
        return
    ss.link_delay = random.randint(20, 35)
    ss.link_loss = round(random.uniform(0.05, 0.25), 2)
    ss.current_waypoint_idx += 1
    ss.flight_progress = ss.current_waypoint_idx / (len(route)-1)
    ss.flight_drone_pos = route[ss.current_waypoint_idx]
    add_log("FCU→OBC→GCS", f"WP_REACHED #{ss.current_waypoint_idx+1}")
    ss.flight_remaining_dist = sum(hdist(route[i], route[i+1]) for i in range(ss.current_waypoint_idx, len(route)-1))
    total = ss.route_analysis.get("total_distance", 1)
    if total > 0:
        ss.flight_time_elapsed = int((ss.flight_progress * total) / ss.flight_speed)
    ss.flight_battery = max(0, 100 - ss.flight_progress * 5)

# ==================== 地图 ====================
def create_map():
    ss = st.session_state
    sw = gcj2wgs(ss.start_point["lat"], ss.start_point["lng"])
    ew = gcj2wgs(ss.end_point["lat"], ss.end_point["lng"])
    clat = (sw[0] + ew[0]) / 2
    clng = (sw[1] + ew[1]) / 2

    m = folium.Map(
        location=[clat, clng],
        zoom_start=18,
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri World Imagery',
        control_scale=True,
        prefer_canvas=True
    )
    plugins.Draw(
        draw_options={
            'polygon': {'allowIntersection': False, 'showArea': True, 'shapeOptions': {'color': '#ff3333', 'fillOpacity': 0.35}},
            'rectangle': {'shapeOptions': {'color': '#ff3333', 'fillOpacity': 0.35}},
            'polyline': False, 'circle': False, 'marker': False, 'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    ).add_to(m)
    plugins.MeasureControl(position='bottomleft', primary_length_unit='meters').add_to(m)

    folium.Marker(
        sw,
        popup=f"起点A (GCJ-02: {ss.start_point['lat']:.5f},{ss.start_point['lng']:.5f})",
        icon=folium.Icon(color='green', icon='play', prefix='fa'),
        tooltip="起点 A"
    ).add_to(m)
    folium.Marker(
        ew,
        popup=f"终点B (GCJ-02: {ss.end_point['lat']:.5f},{ss.end_point['lng']:.5f})",
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'),
        tooltip="终点 B"
    ).add_to(m)

    rl, rg = get_ref()
    for idx, obs in enumerate(ss.obstacles):
        pts = obs["points"]
        wpts = [gcj2wgs(p[0], p[1]) for p in pts]
        h = obs.get("height", 10)
        high = h > ss.flight_height
        fc = "#ff2222" if high else "#ff9900"
        bc = "#cc0000" if high else "#cc7700"
        folium.Polygon(
            locations=wpts, color=bc, weight=2, fill=True, fill_color=fc, fill_opacity=0.55,
            popup=f"障碍物{idx+1}|{'⛔绕行' if high else '✅飞越'} {h}m",
            tooltip=f"障碍物{idx+1}|{h}m"
        ).add_to(m)
        if high:
            try:
                xy = [ll2m(p[0], p[1], rl, rg) for p in wpts]
                buf = Polygon(xy).buffer(float(ss.safety_radius) + 3.0)
                if buf.geom_type == 'Polygon':
                    bp = [m2ll(x, y, rl, rg) for x, y in buf.exterior.coords]
                    folium.Polygon(
                        locations=bp, color='#ffff00', weight=1.5, dash_array='5,4',
                        fill=True, fill_color='#ffff00', fill_opacity=0.08, tooltip="安全缓冲区"
                    ).add_to(m)
            except:
                pass
        cl = sum(p[0] for p in wpts) / len(wpts)
        cg = sum(p[1] for p in wpts) / len(wpts)
        folium.map.Marker(
            [cl, cg],
            icon=folium.DivIcon(
                html=f'<div style="background:rgba(0,0,0,.72);color:#fff;font-size:11px;font-weight:bold;padding:2px 6px;border-radius:4px;border:1px solid {fc};white-space:nowrap;">↑{h}m</div>',
                icon_size=(58, 22), icon_anchor=(29, 11)
            )
        ).add_to(m)

    route = ss.planned_route
    in_f = ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
    if route:
        rwgs = [gcj2wgs(p[0], p[1]) for p in route]
        wpi = ss.current_waypoint_idx
        if not in_f:
            folium.PolyLine(rwgs, color='#1e90ff', weight=4, opacity=0.9, dash_array='12,8', tooltip="规划航线（待飞）").add_to(m)
            for i, pt in enumerate(rwgs):
                if i == 0 or i == len(rwgs)-1:
                    continue
                folium.CircleMarker(pt, radius=6, color='#1e90ff', weight=2,
                                    fill=True, fill_color='white', fill_opacity=0.9, tooltip=f"航点{i}").add_to(m)
        else:
            if wpi >= 1:
                folium.PolyLine(rwgs[:wpi+1], color='#00dd44', weight=5, opacity=1.0, tooltip="已飞轨迹").add_to(m)
                for i in range(1, wpi):
                    folium.CircleMarker(rwgs[i], radius=5, color='#00aa33', weight=2,
                                        fill=True, fill_color='#00ff55', fill_opacity=1.0).add_to(m)
            if wpi < len(rwgs)-1:
                folium.PolyLine(rwgs[wpi:], color='#1e90ff', weight=3, opacity=0.75,
                                dash_array='10,7', tooltip="待飞航线").add_to(m)
                for i in range(wpi+1, len(rwgs)-1):
                    folium.CircleMarker(rwgs[i], radius=5, color='#1e90ff', weight=2,
                                        fill=True, fill_color='white', fill_opacity=0.85).add_to(m)
            if ss.flight_drone_pos:
                dw = gcj2wgs(ss.flight_drone_pos[0], ss.flight_drone_pos[1])
                folium.CircleMarker(dw, radius=22, color='#ff8c00', weight=1,
                                    fill=True, fill_color='#ff8c00', fill_opacity=0.18).add_to(m)
                folium.Marker(
                    dw, tooltip="🚁 无人机当前位置",
                    icon=folium.DivIcon(
                        html='<div style="width:32px;height:32px;background:#ff6600;border:3px solid white;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:16px;box-shadow:0 0 8px rgba(255,102,0,.7);">✈</div>',
                        icon_size=(32, 32), icon_anchor=(16, 16)
                    )
                ).add_to(m)
    return m

# ==================== 通信日志 ====================
def render_comm_logs_page():
    ss = st.session_state
    st.markdown("### 📡 通信链路拓扑与数据流")
    c1, c2, c3 = st.columns(3)
    with c1:
        st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🖥️ GCS 在线</span>', unsafe_allow_html=True)
    with c2:
        st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🧠 OBC 在线</span>', unsafe_allow_html=True)
    with c3:
        st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ ⚙️ FCU 在线</span>', unsafe_allow_html=True)
    good = ss.link_delay < 50
    st.markdown(f"""
<style>
.lrow{{display:flex;align-items:center;gap:12px;background:#f5f7fa;border-radius:12px;padding:14px 18px;margin:10px 0;flex-wrap:wrap;}}
.lcard{{border-radius:10px;padding:10px 18px;text-align:center;min-width:96px;background:white;}}
.lcard.gcs{{border:2px solid #2196F3;}}.lcard.obc{{border:2px solid #FF9800;}}.lcard.fcu{{border:2px solid #9C27B0;}}
.lb{{font-weight:bold;font-size:15px;}}.ls{{font-size:11px;color:#888;}}
.larr{{text-align:center;font-size:13px;color:#555;}}.lstat{{font-size:12px;color:#4CAF50;}}
</style>
<div class="lrow">
  <div class="lcard gcs"><div>🖥️</div><div class="lb">GCS</div><div class="ls">地面站</div><div class="ls">192.168.1.100</div></div>
  <div class="larr"><div>↑↓</div><div class="lstat">UDP:14550</div><div class="lstat">● 已连接</div></div>
  <div class="lcard obc"><div>🧠</div><div class="lb">OBC</div><div class="ls">机载计算机</div><div class="ls">Raspberry Pi 4</div></div>
  <div class="larr"><div>↑↓</div><div class="lstat">MAVLink</div><div class="lstat">● 已连接</div></div>
  <div class="lcard fcu"><div>⚙️</div><div class="lb">FCU</div><div class="ls">飞控</div><div class="ls">PX4 / ArduPilot</div></div>
</div>
<p style="font-size:13px;color:#555;margin:4px 0;">
📊 <b>链路统计：</b>GCS↔OBC: {"正常" if good else "延迟高"} &nbsp;
OBC↔FCU: {"正常" if good else "延迟高"} &nbsp; 延迟:~{ss.link_delay}ms &nbsp; 丢包率:{ss.link_loss}%</p>
""", unsafe_allow_html=True)

    st.markdown("---")
    st.markdown("### 📋 通信日志")
    logs = ss.comm_logs
    g2f = [l for l in logs if l["direction"] == "GCS→OBC→FCU"]
    f2g = [l for l in logs if l["direction"] == "FCU→OBC→GCS"]
    t1, t2, t3 = st.tabs(["🔄 业务流程", "📤 GCS→OBC→FCU", "📥 FCU→OBC→GCS"])

    with t1:
        with st.container(height=260):
            if not logs:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>', unsafe_allow_html=True)
            else:
                rl2 = [l for l in logs if any(k in l["message"] for k in ["规划", "MISSION", "START", "SET_"])]
                nl2 = [l for l in logs if "WP_REACHED" in l["message"]]
                wpc = len(ss.planned_route)
                rlen = ss.route_analysis.get("total_distance", 0)
                if rl2:
                    st.markdown('<span style="color:#4CAF50;font-weight:bold;font-size:13px;">✅ 航线规划</span>', unsafe_allow_html=True)
                    for l in rl2[:6]:
                        st.markdown(f'<div style="background:#f0fff4;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                                    f'[{l["time"]}] 航线规划完成 | 航点数:{wpc} | 路径:{rlen:.1f}m'
                                    f'<br><span style="color:#888;font-size:11px;">🔵 OBC 内部</span></div>', unsafe_allow_html=True)
                if nl2:
                    sp = ss.start_point
                    ep = ss.end_point
                    st.markdown('<span style="color:#2196F3;font-weight:bold;font-size:13px;">ℹ️ 导航目标</span>', unsafe_allow_html=True)
                    for l in nl2[:4]:
                        st.markdown(f'<div style="background:#e3f2fd;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                                    f'[{l["time"]}] 起:({sp["lat"]:.5f},{sp["lng"]:.5f})→终:({ep["lat"]:.5f},{ep["lng"]:.5f})|{ss.flight_height}m'
                                    f'<br><span style="color:#888;font-size:11px;">🟢 GCS→🔵 OBC</span></div>', unsafe_allow_html=True)
                if not rl2 and not nl2:
                    h2 = '<div style="font-size:12px;font-family:monospace;line-height:1.9;">'
                    for l in logs[:12]:
                        bg = "#f0fff4" if "GCS" in l["direction"] else "#fff8e1"
                        h2 += f'<div style="background:{bg};border-radius:4px;padding:2px 7px;margin:2px 0;">[{l["time"]}] {l["direction"]}: <b>{l["message"]}</b></div>'
                    st.markdown(h2+'</div>', unsafe_allow_html=True)

    with t2:
        with st.container(height=260):
            if not g2f:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>', unsafe_allow_html=True)
            else:
                h2 = '<div style="font-size:12px;font-family:monospace;line-height:2;"><div style="color:#e65100;font-weight:bold;margin-bottom:4px;">📤 GCS→OBC→FCU</div>'
                for l in g2f:
                    h2 += f'<div style="border-bottom:1px solid #eee;padding:2px 0;"><span style="color:#888;">[{l["time"]}]</span> <span style="color:#e65100;">GCS→OBC→FCU:</span> <b>{l["message"]}</b></div>'
                st.markdown(h2+'</div>', unsafe_allow_html=True)

    with t3:
        with st.container(height=260):
            if not f2g:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>', unsafe_allow_html=True)
            else:
                h2 = '<div style="background:#fff8e1;border-radius:6px;padding:8px;font-size:12px;font-family:monospace;margin-bottom:8px;"><div style="color:#e65100;font-weight:bold;margin-bottom:4px;">📥 FCU→OBC</div>'
                for l in f2g:
                    h2 += f'<div style="border-bottom:1px dashed #ece;padding:2px 0;"><span style="color:#888;">[{l["time"]}]</span> FCU→OBC→GCS: <b>{l["message"]}</b></div>'
                h2 += '</div><div style="background:#fff8e1;border-radius:6px;padding:8px;font-size:12px;font-family:monospace;"><div style="color:#7B1FA2;font-weight:bold;margin-bottom:4px;">📤 OBC→GCS</div>'
                for l in f2g:
                    h2 += f'<div style="border-bottom:1px dashed #ece;padding:2px 0;"><span style="color:#888;">[{l["time"]}]</span> FCU→OBC→GCS: <b>{l["message"]}</b></div>'
                st.markdown(h2+'</div>', unsafe_allow_html=True)

# ==================== 飞行监控 ====================
def render_flight_monitor():
    ss = st.session_state
    st.markdown("### ✈️ 飞行实时画面 - 任务执行监控")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        if st.button("▶ 开始任务", type="primary", use_container_width=True, disabled=ss.auto_flight_enabled):
            if not ss.planned_route:
                st.error("请先规划航线！")
            else:
                reset_flight()
                ss.auto_flight_enabled = True
                ss.mission_started = True
                add_log("GCS→OBC→FCU", "START_MISSION | Mode: AUTO")
                add_log("业务流程", "任务开始")
                st.rerun()
    with c2:
        if st.button("⏸️ 暂停", use_container_width=True, disabled=not ss.auto_flight_enabled or ss.flight_paused):
            ss.flight_paused = True
            ss.auto_flight_enabled = False
            add_log("GCS→OBC→FCU", "PAUSE")
            st.rerun()
    with c3:
        if st.button("⏹️ 停止", use_container_width=True, disabled=not (ss.auto_flight_enabled or ss.flight_paused)):
            ss.auto_flight_enabled = False
            ss.flight_paused = False
            add_log("GCS→OBC→FCU", "STOP")
            st.rerun()
    with c4:
        if st.button("🔄 重置", use_container_width=True):
            reset_flight()
            st.rerun()

    if ss.auto_flight_enabled and not ss.flight_paused:
        if ss.planned_route and ss.current_waypoint_idx < len(ss.planned_route)-1:
            step_forward()
            from streamlit_autorefresh import st_autorefresh
            st_autorefresh(interval=350, key="afr")
        else:
            ss.auto_flight_enabled = False
            st.success("✅ 已到达终点！任务完成")

    twp = len(ss.planned_route) if ss.planned_route else 0
    cwp = ss.current_waypoint_idx + 1
    c1, c2, c3, c4, c5, c6 = st.columns(6)
    with c1:
        st.metric("当前航点", f"{cwp}/{twp}")
    with c2:
        st.metric("飞行速度", f"{ss.flight_speed:.1f} m/s")
    with c3:
        mv, sv = ss.flight_time_elapsed // 60, ss.flight_time_elapsed % 60
        st.metric("已用时间", f"{mv:02d}:{sv:02d}")
    with c4:
        st.metric("剩余距离", f"{ss.flight_remaining_dist:.0f} m")
    with c5:
        eta = int(ss.flight_remaining_dist / ss.flight_speed) if ss.flight_speed > 0 else 0
        st.metric("预计到达", f"{eta//60:02d}:{eta%60:02d}")
    with c6:
        b = ss.flight_battery
        st.metric("电量模拟", f"{'🟢' if b>50 else '🟡' if b>20 else '🔴'} {b:.0f}%")
    st.progress(ss.flight_progress, text=f"任务进度:{ss.flight_progress*100:.1f}% | {cwp}/{twp} 航点")

# ==================== 障碍物管理 ====================
def save_obstacles():
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump({"obstacles": st.session_state.obstacles}, f, ensure_ascii=False, indent=2)

def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles = data.get("obstacles", [])
            return True, len(st.session_state.obstacles)
    return False, 0

def auto_load_obstacles():
    if not st.session_state.obstacles_loaded:
        load_obstacles()
        st.session_state.obstacles_loaded = True

def add_obstacle_from_draw(feature):
    try:
        if feature.get('geometry', {}).get('type') == 'Polygon':
            coords = feature['geometry']['coordinates'][0]
            pts = []
            for c in coords:
                gl, gg = wgs2gcj(c[1], c[0])
                pts.append([gl, gg])
            if len(pts) > 1 and pts[0] == pts[-1]:
                pts = pts[:-1]
            if len(pts) >= 3:
                h = st.session_state.new_obstacle_height
                st.session_state.obstacles.append({
                    "points": pts,
                    "height": h,
                    "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                })
                save_obstacles()
                add_log("业务流程", f"障碍物已添加|高度:{h}m|顶点:{len(pts)}")
                st.session_state.route_version += 1
                return True
    except Exception as e:
        st.error(f"添加障碍物失败:{e}")
    return False

def remove_obstacle(idx):
    if 0 <= idx < len(st.session_state.obstacles):
        st.session_state.obstacles.pop(idx)
        save_obstacles()
        st.session_state.route_version += 1

def clear_obstacles():
    st.session_state.obstacles = []
    save_obstacles()
    st.session_state.route_version += 1

def heartbeat():
    st.session_state.heartbeat_count += 1
    return {
        "sequence": st.session_state.heartbeat_count,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "battery": random.randint(85, 100),
        "signal": random.randint(70, 99)
    }

# ==================== 主函数 ====================
def main():
    st.title("✈️ 无人机地面站系统")
    st.caption("卫星实况地图 | 智能绕行算法 | 流畅优化版 | 起点(32.2323,118.749) 终点(32.2344,118.749)")

    auto_load_obstacles()
    hb = heartbeat()
    c1, c2, c3, c4, c5 = st.columns(5)
    with c1:
        st.metric("💓 心跳", "在线")
    with c2:
        st.metric("📡 序列号", hb["sequence"])
    with c3:
        st.metric("🔋 电量", f"{hb['battery']}%")
    with c4:
        st.metric("📶 信号", f"{hb['signal']}%")
    with c5:
        st.metric("🕐 时间", hb["timestamp"])
    st.divider()
    render_flight_monitor()
    st.divider()

    tab1, tab2 = st.tabs(["🗺️ 飞行监控与规划", "📡 通信链路与日志"])

    with tab1:
        left, mid, right = st.columns([2, 1, 1])

        with left:
            ss = st.session_state
            st.subheader("🛰️ 实时飞行地图（卫星）")
            mc1, mc2, mc3, mc4, mc5, mc6 = st.columns(6)
            with mc1:
                if st.button("📍 起点A", use_container_width=True):
                    ss.setting_mode = "start"
            with mc2:
                if st.button("🏁 终点B", use_container_width=True):
                    ss.setting_mode = "end"
            with mc3:
                if st.button("❌ 取消", use_container_width=True):
                    ss.setting_mode = None
            with mc4:
                if st.button("⬅️ 左绕行", use_container_width=True):
                    ss.bypass_strategy = "left"
                    ss.route_version += 1
                    st.rerun()
            with mc5:
                if st.button("➡️ 右绕行", use_container_width=True):
                    ss.bypass_strategy = "right"
                    ss.route_version += 1
                    st.rerun()
            with mc6:
                if st.button("🌟 最佳航线", type="primary", use_container_width=True):
                    ss.bypass_strategy = "best"
                    ss.route_version += 1
                    st.rerun()

            # 版本变化时自动重新规划
            if "last_version" not in st.session_state or st.session_state.last_version != st.session_state.route_version:
                st.session_state.last_version = st.session_state.route_version
                with st.spinner("正在规划最优航线..."):
                    plan_route()
                    add_log("业务流程", f"航线规划完成 | 航点数:{len(st.session_state.planned_route)} | 距离:{st.session_state.route_analysis.get('total_distance',0):.1f}m")
                st.rerun()

            mode = ss.setting_mode
            sl = {"left": "⬅️左绕行", "right": "➡️右绕行", "best": "🌟最佳"}.get(ss.bypass_strategy, "")
            if mode == "start":
                st.info("🔵 点击地图设置起点A（设置完自动退出）")
            elif mode == "end":
                st.info("🔴 点击地图设置终点B（设置完自动退出）")
            else:
                inf = ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
                st.caption(f"当前:{sl} | " +
                           ("🟢绿线=已飞|🔵蓝虚=待飞|🟠=无人机" if inf else
                            "🔵蓝虚=规划|🔴红=高障|🟠橙=低障|黄虚=安全缓冲"))

            try:
                mp = create_map()
                out = st_folium(mp, width=820, height=560,
                                key=f"map_{ss.map_key}",
                                returned_objects=["last_active_drawing", "last_clicked"])

                # 处理地图点击设置A/B点
                if out and out.get("last_clicked"):
                    ck = out["last_clicked"]
                    cur_mode = ss.setting_mode
                    if ck and "lat" in ck and "lng" in ck and cur_mode in ("start", "end"):
                        gl, gg = wgs2gcj(ck["lat"], ck["lng"])
                        ss.setting_mode = None
                        if cur_mode == "start":
                            ss.start_point = {"lat": gl, "lng": gg, "height": 0}
                            add_log("GCS→OBC→FCU", f"SET_START ({gl:.5f},{gg:.5f})")
                        else:
                            ss.end_point = {"lat": gl, "lng": gg, "height": 0}
                            add_log("GCS→OBC→FCU", f"SET_END ({gl:.5f},{gg:.5f})")
                        _wp_save()
                        ss.route_version += 1
                        st.rerun()

                if out and out.get("last_active_drawing"):
                    feat = out["last_active_drawing"]
                    if feat.get("geometry", {}).get("type") == "Polygon":
                        if add_obstacle_from_draw(feat):
                            st.success("✅ 障碍物已添加，航线已重新规划")
                            st.rerun()
            except Exception as e:
                st.error(f"地图错误: {e}")
                import traceback
                st.code(traceback.format_exc())

        with mid:
            ss = st.session_state
            st.subheader("🎮 控制面板")
            with st.expander("📍 起点 A（GCJ-02）", expanded=True):
                st.number_input(
                    "纬度", value=ss.start_point["lat"], format="%.6f",
                    key="start_lat", on_change=on_start_lat_change
                )
                st.number_input(
                    "经度", value=ss.start_point["lng"], format="%.6f",
                    key="start_lng", on_change=on_start_lng_change
                )
            with st.expander("🏁 终点 B（GCJ-02）", expanded=True):
                st.number_input(
                    "纬度", value=ss.end_point["lat"], format="%.6f",
                    key="end_lat", on_change=on_end_lat_change
                )
                st.number_input(
                    "经度", value=ss.end_point["lng"], format="%.6f",
                    key="end_lng", on_change=on_end_lng_change
                )
            st.divider()
            st.subheader("✈️ 飞行参数")
            st.number_input(
                "飞行高度 (m)", value=ss.flight_height, step=5, min_value=10, max_value=200,
                key="fh_input", on_change=on_fh_change
            )
            st.number_input(
                "安全半径 (m)", value=ss.safety_radius, step=1, min_value=5, max_value=50,
                key="sr_input", on_change=on_sr_change
            )
            spd = st.slider("飞行速度 (m/s)", 1.0, 20.0, value=float(ss.flight_speed), step=0.5)
            if abs(spd - ss.flight_speed) > 0.01:
                ss.flight_speed = spd
            st.divider()
            st.subheader("⛔ 新障碍物高度")
            st.number_input("障碍物高度 (m)", value=60, step=5, min_value=10, max_value=200, key="new_obstacle_height")
            st.caption("💡 在地图上用多边形工具绘制障碍物区域")

        with right:
            ss = st.session_state
            st.subheader("📊 航线分析")
            if ss.route_analysis:
                a = ss.route_analysis
                st.metric("📏 总距离", f"{a.get('total_distance', 0):.1f} m")
                st.metric("🔄 绕行节点", a.get('bypass_count', 0))
                st.metric("✅ 飞跃次数", a.get('fly_over_count', 0))
                st.metric("🎯 使用策略", a.get('strategy_used', '未知'))
                st.divider()
                st.caption("📋 障碍物处理")
                for o in a.get('obstacles_encountered', []):
                    st.text(f"{'🔄' if '绕行' in o['decision'] else '✅'} {o['height']}m → {o['decision']}")
            else:
                st.info("点击规划按钮生成报告")
            st.divider()
            st.subheader("⛔ 障碍物列表")
            if ss.obstacles:
                st.caption(f"共 {len(ss.obstacles)} 个障碍物")
                for idx, obs in enumerate(ss.obstacles):
                    ca, cb, cc = st.columns([1, 2, 1])
                    with ca:
                        if st.button("🗑️", key=f"d{idx}"):
                            remove_obstacle(idx)
                            st.rerun()
                    with cb:
                        st.text(f"障碍 {idx+1}")
                        if obs.get('created_at'):
                            st.caption(obs['created_at'][:10])
                    with cc:
                        h_ = obs.get('height', 10)
                        st.text(f"{'🔴' if h_ > ss.flight_height else '🟠'} {h_}m")
            else:
                st.info("暂无障碍物\n在地图绘制多边形添加")
            st.divider()
            cs, cl, cc2 = st.columns(3)
            with cs:
                if st.button("💾 保存", use_container_width=True):
                    save_obstacles()
                    _wp_save()
                    st.success("已保存")
            with cl:
                if st.button("📂 加载", use_container_width=True):
                    ok, cnt = load_obstacles()
                    wp = _wp_load()
                    if wp:
                        if "start_point" in wp:
                            ss.start_point = wp["start_point"]
                        if "end_point" in wp:
                            ss.end_point = wp["end_point"]
                        if "flight_height" in wp:
                            ss.flight_height = wp["flight_height"]
                        if "safety_radius" in wp:
                            ss.safety_radius = wp["safety_radius"]
                    st.success(f"已加载{cnt}个障碍物及起终点")
                    ss.route_version += 1
                    st.rerun()
            with cc2:
                if st.button("🗑️ 清空", use_container_width=True):
                    clear_obstacles()
                    st.rerun()

    with tab2:
        render_comm_logs_page()

if __name__ == "__main__":
    main()
