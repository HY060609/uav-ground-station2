"""
无人机地面站系统 - v5
核心升级：航线规划算法替换为 Visibility Graph + Dijkstra
- 提取所有障碍物安全缓冲区的凸包顶点作为可见性图节点
- 在节点间连线，仅保留不穿过任何障碍安全区的边
- Dijkstra 求最短路径 → 真正意义上的最短安全航线
- 支持多障碍物、任意形状、自动找最优绕行方向
"""
 
import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins
import json, os, math, random, heapq
from datetime import datetime
from shapely.geometry import Polygon, LineString, Point, MultiPolygon
from shapely.ops import unary_union
from streamlit_autorefresh import st_autorefresh
 
# ==================== 配置常量 ====================
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ  = [118.751589, 32.235204]
CONFIG_FILE    = "obstacle_config.json"
DEFAULT_SAFETY_RADIUS = 8
 
# ==================== Session State ====================
def init_session_state():
    defaults = {
        'heartbeat_count':      0,
        'obstacles':            [],
        'start_point':          {"lat": DEFAULT_A_GCJ[1], "lng": DEFAULT_A_GCJ[0], "height": 0},
        'end_point':            {"lat": DEFAULT_B_GCJ[1], "lng": DEFAULT_B_GCJ[0], "height": 0},
        'flight_height':        50,
        'safety_radius':        DEFAULT_SAFETY_RADIUS,
        'bypass_strategy':      "best",
        'planned_route':        [],
        'route_analysis':       {},
        'setting_mode':         None,
        'obstacles_loaded':     False,
        'map_key':              0,
        'new_obstacle_height':  60,
        'auto_flight_enabled':  False,
        'flight_paused':        False,
        'flight_progress':      0.0,
        'current_waypoint_idx': 0,
        'flight_remaining_dist':0.0,
        'flight_battery':       100,
        'flight_drone_pos':     None,
        'flight_time_elapsed':  0,
        'flight_speed':         8.0,
        'comm_logs':            [],
        'link_delay':           25,
        'link_loss':            0.1,
        'mission_started':      False,
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v
 
init_session_state()
 
# ==================== GCJ-02 ↔ WGS-84 ====================
_A, _EE, _PI, _180 = 6378245.0, 0.00669342162296594323, math.pi, 180.0
 
def _out_of_china(lat, lng):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)
 
def _tlat(lng, lat):
    r  = -100+2*lng+3*lat+0.2*lat*lat+0.1*lng*lat+0.2*math.sqrt(abs(lng))
    r += (20*math.sin(6*lng*_PI)+20*math.sin(2*lng*_PI))*2/3
    r += (20*math.sin(lat*_PI)+40*math.sin(lat/3*_PI))*2/3
    r += (160*math.sin(lat/12*_PI)+320*math.sin(lat*_PI/30))*2/3
    return r
 
def _tlng(lng, lat):
    r  = 300+lng+2*lat+0.1*lng*lng+0.1*lng*lat+0.1*math.sqrt(abs(lng))
    r += (20*math.sin(6*lng*_PI)+20*math.sin(2*lng*_PI))*2/3
    r += (20*math.sin(lng*_PI)+40*math.sin(lng/3*_PI))*2/3
    r += (150*math.sin(lng/12*_PI)+300*math.sin(lng/30*_PI))*2/3
    return r
 
def _delta(lat, lng):
    dl = _tlat(lng-105, lat-35); dg = _tlng(lng-105, lat-35)
    rl = lat/_180*_PI; mg = math.sin(rl); mg = 1-_EE*mg*mg; sq = math.sqrt(mg)
    dl = dl*180/((_A*(1-_EE))/(mg*sq)*_PI)
    dg = dg*180/(_A/sq*math.cos(rl)*_PI)
    return dl, dg
 
def gcj02_to_wgs84(lat, lng):
    if _out_of_china(lat, lng): return float(lat), float(lng)
    dl, dg = _delta(lat, lng)
    return float(lat-dl), float(lng-dg)
 
def wgs84_to_gcj02(lat, lng):
    if _out_of_china(lat, lng): return float(lat), float(lng)
    dl, dg = _delta(lat, lng)
    return float(lat+dl), float(lng+dg)
 
# ==================== 米制投影 ====================
def get_ref():
    lat = (st.session_state.start_point["lat"]+st.session_state.end_point["lat"])/2
    lng = (st.session_state.start_point["lng"]+st.session_state.end_point["lng"])/2
    return lat, lng
 
def ll2m(lat, lng, rlat, rlng):
    return ((lng-rlng)*math.cos(math.radians(rlat))*111320,
            (lat-rlat)*111320)
 
def m2ll(x, y, rlat, rlng):
    return (y/111320+rlat,
            x/(math.cos(math.radians(rlat))*111320)+rlng)
 
def hdist(lat1, lng1, lat2, lng2):
    R=6371000; p1,p2=math.radians(lat1),math.radians(lat2)
    dp=math.radians(lat2-lat1); dl=math.radians(lng2-lng1)
    a=math.sin(dp/2)**2+math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R*2*math.atan2(math.sqrt(a),math.sqrt(1-a))
 
def path_len(path):
    return sum(hdist(path[i][0],path[i][1],path[i+1][0],path[i+1][1]) for i in range(len(path)-1))
 
# ==================== 构建安全缓冲区联合体 ====================
def build_safe_union(obstacles, fh, sr, rlat, rlng):
    """返回所有高于飞行高度的障碍物的安全缓冲区联合体（米制坐标）"""
    polys = []
    for obs in obstacles:
        if obs.get("height", 30) <= fh:
            continue
        pts = obs.get("points", [])
        if len(pts) < 3:
            continue
        try:
            xy = [ll2m(p[0], p[1], rlat, rlng) for p in pts]
            poly = Polygon(xy)
            if not poly.is_valid:
                poly = poly.buffer(0)
            polys.append(poly.buffer(sr))   # 真实安全半径 buffer
        except:
            continue
    if not polys:
        return None
    return unary_union(polys)
 
# ==================== Visibility Graph 核心 ====================
 
def _seg_free(ax, ay, bx, by, union_safe, eps=0.05):
    """检查线段(A→B)是否完全在安全区之外（不与障碍物缓冲区相交）"""
    seg = LineString([(ax,ay),(bx,by)])
    # 稍微缩短线段两端，避免顶点本身在边界上导致误判
    if seg.length < eps*2:
        return not Point((ax+bx)/2,(ay+by)/2).within(union_safe)
    shrunk = seg.interpolate(eps), seg.interpolate(seg.length-eps)
    short  = LineString([shrunk[0], shrunk[1]])
    return not short.intersects(union_safe)
 
def _get_visibility_nodes(union_safe, sx, sy, ex, ey):
    """
    从障碍物安全缓冲区轮廓提取候选绕行顶点。
    策略：取每个多边形外轮廓的所有顶点，这些顶点是最短路径的候选转折点。
    """
    nodes = [(sx, sy), (ex, ey)]
    geoms = [union_safe] if union_safe.geom_type == 'Polygon' else list(union_safe.geoms)
    for g in geoms:
        if g.geom_type != 'Polygon':
            continue
        coords = list(g.exterior.coords)[:-1]   # 去掉重复的最后一个点
        nodes.extend(coords)
    return nodes
 
def _dijkstra_shortest(nodes, union_safe):
    """
    在可见性图上运行 Dijkstra，返回从 nodes[0] 到 nodes[1] 的最短路径节点下标列表。
    nodes[0] = 起点, nodes[1] = 终点
    """
    n = len(nodes)
    # 构建邻接表（只保留不穿过障碍物的边）
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            ax, ay = nodes[i]
            bx, by = nodes[j]
            if _seg_free(ax, ay, bx, by, union_safe):
                d = math.hypot(bx-ax, by-ay)
                adj[i].append((j, d))
                adj[j].append((i, d))
 
    # Dijkstra
    INF  = float('inf')
    dist = [INF]*n
    prev = [-1]*n
    dist[0] = 0
    heap = [(0.0, 0)]
    while heap:
        cost, u = heapq.heappop(heap)
        if cost > dist[u]:
            continue
        for v, w in adj[u]:
            nc = cost+w
            if nc < dist[v]:
                dist[v] = nc
                prev[v] = u
                heapq.heappush(heap, (nc, v))
 
    # 回溯路径
    if dist[1] == INF:
        return None   # 无路可走
    path_idx = []
    cur = 1
    while cur != -1:
        path_idx.append(cur)
        cur = prev[cur]
    path_idx.reverse()
    return path_idx
 
def _smooth_path(path_m, union_safe):
    """
    路径平滑：对 Dijkstra 结果做贪心 shortcut。
    如果能从节点 i 直接看见节点 j（j>i+1），则跳过中间节点，缩短路径。
    """
    if len(path_m) <= 2:
        return path_m
    smoothed = [path_m[0]]
    i = 0
    while i < len(path_m)-1:
        # 尽量往后跳
        j = len(path_m)-1
        while j > i+1:
            ax, ay = path_m[i]
            bx, by = path_m[j]
            if _seg_free(ax, ay, bx, by, union_safe):
                break
            j -= 1
        smoothed.append(path_m[j])
        i = j
    return smoothed
 
# ==================== 主规划函数 ====================
def plan_visibility_graph(start_gcj, end_gcj, obstacles, fh, sr):
    """
    Visibility Graph + Dijkstra 最短安全路径规划。
    返回 (path_gcj_list, analysis_dict)
    """
    rlat, rlng = get_ref()
    sx, sy = ll2m(start_gcj[0], start_gcj[1], rlat, rlng)
    ex, ey = ll2m(end_gcj[0],   end_gcj[1],   rlat, rlng)
 
    analysis = {
        "total_distance": 0,
        "obstacles_encountered": [],
        "bypass_count": 0,
        "fly_over_count": 0,
        "route_points": [],
        "strategy_used": "",
        "waypoint_count": 0,
    }
 
    # 统计障碍物处理情况
    union = build_safe_union(obstacles, fh, sr, rlat, rlng)
    for obs in obstacles:
        h = obs.get("height", 30)
        if h > fh:
            analysis["obstacles_encountered"].append({"height": h, "decision": "绕行"})
        else:
            analysis["fly_over_count"] += 1
            analysis["obstacles_encountered"].append({"height": h, "decision": "飞跃(低于飞行高度)"})
 
    # 无障碍物或直线可通
    if union is None or union.is_empty:
        path_gcj = [start_gcj, end_gcj]
        analysis["total_distance"] = hdist(*start_gcj, *end_gcj)
        analysis["strategy_used"]  = "直线（无障碍）"
        analysis["waypoint_count"] = 2
        analysis["route_points"]   = path_gcj
        return path_gcj, analysis
 
    direct = LineString([(sx,sy),(ex,ey)])
    if not direct.intersects(union):
        path_gcj = [start_gcj, end_gcj]
        analysis["total_distance"] = hdist(*start_gcj, *end_gcj)
        analysis["strategy_used"]  = "直线（直线不碰障碍物）"
        analysis["waypoint_count"] = 2
        analysis["route_points"]   = path_gcj
        return path_gcj, analysis
 
    # ---- Visibility Graph ----
    nodes   = _get_visibility_nodes(union, sx, sy, ex, ey)
    path_idx = _dijkstra_shortest(nodes, union)
 
    if path_idx is None:
        # 极端情况：图上找不到路（起终点被包围），退回简单偏移
        path_gcj = [start_gcj, end_gcj]
        analysis["strategy_used"] = "⚠️ 无法规划（起/终点在障碍物内）"
        analysis["route_points"]  = path_gcj
        return path_gcj, analysis
 
    path_m = [nodes[i] for i in path_idx]
 
    # ---- 路径平滑 ----
    path_m = _smooth_path(path_m, union)
 
    # ---- 转回 GCJ-02 ----
    path_gcj = [m2ll(x, y, rlat, rlng) for x, y in path_m]
 
    # 统计绕行次数（转折点数量）
    waypoints      = len(path_gcj)
    bypass_wps     = waypoints - 2   # 去掉起终点
    analysis["bypass_count"]    = max(0, bypass_wps)
    analysis["total_distance"]  = path_len(path_gcj)
    analysis["waypoint_count"]  = waypoints
    analysis["strategy_used"]   = f"Visibility Graph 最短路径（{bypass_wps}个绕行点）"
    analysis["route_points"]    = path_gcj
    return path_gcj, analysis
 
# ==================== plan_route 入口 ====================
def plan_route():
    ss    = st.session_state
    start = (ss.start_point["lat"], ss.start_point["lng"])
    end   = (ss.end_point["lat"],   ss.end_point["lng"])
    fh, sr = ss.flight_height, ss.safety_radius
    obs    = ss.obstacles
 
    path_gcj, analysis = plan_visibility_graph(start, end, obs, fh, sr)
 
    ss.planned_route  = path_gcj
    ss.route_analysis = analysis
    ss.map_key       += 1
    return path_gcj, analysis
 
# ==================== 飞行控制 ====================
def reset_flight():
    ss = st.session_state
    ss.auto_flight_enabled   = False
    ss.flight_paused         = False
    ss.flight_progress       = 0.0
    ss.current_waypoint_idx  = 0
    ss.flight_remaining_dist = ss.route_analysis.get("total_distance", 0)
    ss.flight_battery        = 100
    ss.flight_drone_pos      = ss.planned_route[0] if ss.planned_route else None
    ss.flight_time_elapsed   = 0
    ss.mission_started       = False
 
def add_log(direction, message):
    ts = datetime.now().strftime("%H:%M:%S")
    st.session_state.comm_logs.insert(0, {"time": ts, "direction": direction, "message": message})
    if len(st.session_state.comm_logs) > 100:
        st.session_state.comm_logs.pop()
 
def step_forward():
    ss    = st.session_state
    route = ss.planned_route
    if not route: return
    if ss.current_waypoint_idx >= len(route)-1:
        ss.auto_flight_enabled = False
        add_log("FCU→OBC→GCS", "MISSION_COMPLETE")
        return
    ss.link_delay           = random.randint(20, 35)
    ss.link_loss            = round(random.uniform(0.05, 0.25), 2)
    ss.current_waypoint_idx += 1
    ss.flight_progress      = ss.current_waypoint_idx / (len(route)-1)
    ss.flight_drone_pos     = route[ss.current_waypoint_idx]
    add_log("FCU→OBC→GCS", f"WP_REACHED #{ss.current_waypoint_idx+1}")
    remaining = sum(hdist(route[i][0],route[i][1],route[i+1][0],route[i+1][1])
                    for i in range(ss.current_waypoint_idx, len(route)-1))
    ss.flight_remaining_dist = remaining
    total = ss.route_analysis.get("total_distance", 1)
    if total > 0:
        ss.flight_time_elapsed = int((ss.flight_progress*total)/ss.flight_speed)
    ss.flight_battery = max(0, 100 - ss.flight_progress*5)
 
# ==================== 地图创建 ====================
def create_map():
    ss = st.session_state
    sw = gcj02_to_wgs84(ss.start_point["lat"], ss.start_point["lng"])
    ew = gcj02_to_wgs84(ss.end_point["lat"],   ss.end_point["lng"])
    clat = (sw[0]+ew[0])/2; clng = (sw[1]+ew[1])/2
 
    m = folium.Map(
        location=[clat, clng], zoom_start=18,
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri World Imagery', control_scale=True, prefer_canvas=True
    )
 
    plugins.Draw(
        draw_options={
            'polygon':   {'allowIntersection': False, 'showArea': True,
                          'shapeOptions': {'color':'#ff3333','fillOpacity':0.35}},
            'rectangle': {'shapeOptions': {'color':'#ff3333','fillOpacity':0.35}},
            'polyline': False, 'circle': False, 'marker': False, 'circlemarker': False,
        },
        edit_options={'edit': True, 'remove': True}
    ).add_to(m)
    plugins.MeasureControl(position='bottomleft', primary_length_unit='meters').add_to(m)
 
    # 起终点
    folium.Marker(sw, popup=f"起点A", icon=folium.Icon(color='green', icon='play', prefix='fa'), tooltip="起点 A").add_to(m)
    folium.Marker(ew, popup=f"终点B", icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'), tooltip="终点 B").add_to(m)
 
    # 障碍物
    rlat, rlng = get_ref()
    for idx, obs in enumerate(ss.obstacles):
        pts   = obs["points"]
        wpts  = [gcj02_to_wgs84(p[0], p[1]) for p in pts]
        h     = obs.get("height", 10)
        high  = h > ss.flight_height
        fc    = "#ff2222" if high else "#ff9900"
        bc    = "#cc0000" if high else "#cc7700"
 
        folium.Polygon(locations=wpts, color=bc, weight=2,
                       fill=True, fill_color=fc, fill_opacity=0.55,
                       popup=f"障碍物{idx+1} | {'⛔绕行' if high else '✅飞越'} {h}m",
                       tooltip=f"障碍物{idx+1} | {h}m").add_to(m)
 
        # 安全缓冲区（仅高障碍物显示）
        if high:
            try:
                xy  = [ll2m(p[0], p[1], rlat, rlng) for p in wpts]
                buf = Polygon(xy).buffer(ss.safety_radius)
                if buf.geom_type == 'Polygon':
                    bp = [m2ll(x, y, rlat, rlng) for x, y in buf.exterior.coords]
                    folium.Polygon(locations=bp, color='#ffdd00', weight=1.5,
                                   dash_array='6,4', fill=True,
                                   fill_color='#ffdd00', fill_opacity=0.10,
                                   tooltip="安全缓冲区").add_to(m)
            except: pass
 
        # 高度标注
        cl = sum(p[0] for p in wpts)/len(wpts)
        cg = sum(p[1] for p in wpts)/len(wpts)
        folium.map.Marker([cl, cg], icon=folium.DivIcon(
            html=f'<div style="background:rgba(0,0,0,.72);color:#fff;font-size:11px;'
                 f'font-weight:bold;padding:2px 6px;border-radius:4px;'
                 f'border:1px solid {fc};white-space:nowrap;">↑{h}m</div>',
            icon_size=(58,22), icon_anchor=(29,11)
        )).add_to(m)
 
    # 航线
    route = ss.planned_route
    if route:
        rwgs      = [gcj02_to_wgs84(p[0], p[1]) for p in route]
        wp_idx    = ss.current_waypoint_idx
        in_flight = ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
 
        if not in_flight:
            # 规划阶段：蓝色虚线
            folium.PolyLine(rwgs, color='#1e90ff', weight=4, opacity=0.9,
                            dash_array='12,8', tooltip="规划航线（待飞）").add_to(m)
            for i, pt in enumerate(rwgs):
                if i == 0 or i == len(rwgs)-1: continue
                folium.CircleMarker(pt, radius=6, color='#1e90ff', weight=2,
                                    fill=True, fill_color='white', fill_opacity=0.9,
                                    tooltip=f"绕行航点 {i}").add_to(m)
        else:
            # 已飞：绿色实线
            if wp_idx >= 1:
                folium.PolyLine(rwgs[:wp_idx+1], color='#00dd44', weight=5,
                                opacity=1.0, tooltip="已飞轨迹").add_to(m)
                for i in range(1, wp_idx):
                    folium.CircleMarker(rwgs[i], radius=5, color='#00aa33', weight=2,
                                        fill=True, fill_color='#00ff55', fill_opacity=1.0).add_to(m)
            # 未飞：蓝色虚线
            if wp_idx < len(rwgs)-1:
                folium.PolyLine(rwgs[wp_idx:], color='#1e90ff', weight=3,
                                opacity=0.75, dash_array='10,7', tooltip="待飞航线").add_to(m)
                for i in range(wp_idx+1, len(rwgs)-1):
                    folium.CircleMarker(rwgs[i], radius=5, color='#1e90ff', weight=2,
                                        fill=True, fill_color='white', fill_opacity=0.85).add_to(m)
 
        # 无人机
        if ss.flight_drone_pos and in_flight:
            dw = gcj02_to_wgs84(ss.flight_drone_pos[0], ss.flight_drone_pos[1])
            folium.CircleMarker(dw, radius=22, color='#ff8c00', weight=1,
                                fill=True, fill_color='#ff8c00', fill_opacity=0.18).add_to(m)
            folium.Marker(dw, icon=folium.DivIcon(
                html='<div style="width:32px;height:32px;background:#ff6600;border:3px solid white;'
                     'border-radius:50%;display:flex;align-items:center;justify-content:center;'
                     'font-size:16px;box-shadow:0 0 8px rgba(255,102,0,.7);">✈</div>',
                icon_size=(32,32), icon_anchor=(16,16)
            ), tooltip="🚁 无人机当前位置").add_to(m)
 
    return m
 
# ==================== 通信日志 ====================
def render_comm_logs():
    ss = st.session_state
    st.markdown("---")
    st.markdown("### 📡 通信链路拓扑与数据流")
    c1,c2,c3 = st.columns(3)
    with c1: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🖥️ GCS 在线</span>', unsafe_allow_html=True)
    with c2: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🧠 OBC 在线</span>', unsafe_allow_html=True)
    with c3: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ ⚙️ FCU 在线</span>', unsafe_allow_html=True)
 
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
📊 <b>链路统计：</b>GCS↔OBC: {"正常" if ss.link_delay<50 else "延迟高"} &nbsp;
OBC↔FCU: {"正常" if ss.link_delay<50 else "延迟高"} &nbsp;
延迟: ~{ss.link_delay}ms &nbsp; 丢包率: {ss.link_loss}%
</p>
""", unsafe_allow_html=True)
 
    st.markdown("---")
    st.markdown("### 📋 通信日志")
    logs         = ss.comm_logs
    gcs2fcu_logs = [l for l in logs if l["direction"] == "GCS→OBC→FCU"]
    fcu2gcs_logs = [l for l in logs if l["direction"] == "FCU→OBC→GCS"]
    tab1, tab2, tab3 = st.tabs(["🔄 业务流程", "📤 GCS→OBC→FCU", "📥 FCU→OBC→GCS"])
 
    with tab1:
        with st.container(height=260):
            if not logs:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>', unsafe_allow_html=True)
            else:
                route_logs = [l for l in logs if any(k in l["message"] for k in ["规划","MISSION","START","SET_"])]
                nav_logs   = [l for l in logs if "WP_REACHED" in l["message"]]
                wp_count   = len(ss.planned_route)
                route_len  = ss.route_analysis.get("total_distance", 0)
                if route_logs:
                    st.markdown('<span style="color:#4CAF50;font-weight:bold;font-size:13px;">✅ 航线规划</span>', unsafe_allow_html=True)
                    for log in route_logs[:6]:
                        st.markdown(
                            f'<div style="background:#f0fff4;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                            f'[{log["time"]}] 航线规划完成 | 类型: horizontal | 航点数: {wp_count} | 路径长度: {route_len:.1f}m'
                            f'<br><span style="color:#888;font-size:11px;">🔵 OBC 内部</span></div>',
                            unsafe_allow_html=True)
                if nav_logs:
                    sp = ss.start_point; ep = ss.end_point
                    st.markdown('<span style="color:#2196F3;font-weight:bold;font-size:13px;">ℹ️ 导航目标</span>', unsafe_allow_html=True)
                    for log in nav_logs[:4]:
                        st.markdown(
                            f'<div style="background:#e3f2fd;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                            f'[{log["time"]}] 起:({sp["lat"]:.5f},{sp["lng"]:.5f}) → 终:({ep["lat"]:.5f},{ep["lng"]:.5f}) | 高度:{ss.flight_height}m'
                            f'<br><span style="color:#888;font-size:11px;">🟢 GCS → 🔵 OBC</span></div>',
                            unsafe_allow_html=True)
                if not route_logs and not nav_logs:
                    html = '<div style="font-size:12px;font-family:monospace;line-height:1.9;">'
                    for log in logs[:12]:
                        bg = "#f0fff4" if "GCS" in log["direction"] else "#fff8e1"
                        html += f'<div style="background:{bg};border-radius:4px;padding:2px 7px;margin:2px 0;">[{log["time"]}] {log["direction"]}: <b>{log["message"]}</b></div>'
                    html += '</div>'
                    st.markdown(html, unsafe_allow_html=True)
 
    with tab2:
        with st.container(height=260):
            if not gcs2fcu_logs:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>', unsafe_allow_html=True)
            else:
                html = '<div style="font-size:12px;font-family:monospace;line-height:2;">'
                html += '<div style="color:#e65100;font-weight:bold;margin-bottom:4px;">📤 GCS → OBC → FCU</div>'
                for log in gcs2fcu_logs:
                    html += (f'<div style="border-bottom:1px solid #eee;padding:2px 0;">'
                             f'<span style="color:#888;">[{log["time"]}]</span> '
                             f'<span style="color:#e65100;">GCS→OBC→FCU:</span> <b>{log["message"]}</b></div>')
                html += '</div>'
                st.markdown(html, unsafe_allow_html=True)
 
    with tab3:
        with st.container(height=260):
            if not fcu2gcs_logs:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>', unsafe_allow_html=True)
            else:
                html  = '<div style="background:#fff8e1;border-radius:6px;padding:8px;font-size:12px;font-family:monospace;margin-bottom:8px;">'
                html += '<div style="color:#e65100;font-weight:bold;margin-bottom:4px;">📥 FCU → OBC</div>'
                for log in fcu2gcs_logs:
                    html += (f'<div style="border-bottom:1px dashed #ece;padding:2px 0;">'
                             f'<span style="color:#888;">[{log["time"]}]</span> FCU→OBC→GCS: <b>{log["message"]}</b></div>')
                html += '</div>'
                html += '<div style="background:#fff8e1;border-radius:6px;padding:8px;font-size:12px;font-family:monospace;">'
                html += '<div style="color:#7B1FA2;font-weight:bold;margin-bottom:4px;">📤 OBC → GCS</div>'
                for log in fcu2gcs_logs:
                    html += (f'<div style="border-bottom:1px dashed #ece;padding:2px 0;">'
                             f'<span style="color:#888;">[{log["time"]}]</span> FCU→OBC→GCS: <b>{log["message"]}</b></div>')
                html += '</div>'
                st.markdown(html, unsafe_allow_html=True)
 
# ==================== 飞行监控 ====================
def render_flight_monitor():
    ss = st.session_state
    st.markdown("### ✈️ 飞行实时画面 - 任务执行监控")
    c1,c2,c3,c4 = st.columns(4)
    with c1:
        if st.button("▶ 开始任务", type="primary", use_container_width=True, disabled=ss.auto_flight_enabled):
            if not ss.planned_route: st.error("请先规划航线！")
            else:
                reset_flight()
                ss.auto_flight_enabled = True
                ss.mission_started     = True
                add_log("GCS→OBC→FCU", "START_MISSION | Mode: AUTO")
                add_log("业务流程", "任务开始")
                st.rerun()
    with c2:
        if st.button("⏸️ 暂停", use_container_width=True, disabled=not ss.auto_flight_enabled or ss.flight_paused):
            ss.flight_paused = True; ss.auto_flight_enabled = False
            add_log("GCS→OBC→FCU", "PAUSE"); st.rerun()
    with c3:
        if st.button("⏹️ 停止", use_container_width=True, disabled=not (ss.auto_flight_enabled or ss.flight_paused)):
            ss.auto_flight_enabled = False; ss.flight_paused = False
            add_log("GCS→OBC→FCU", "STOP"); st.rerun()
    with c4:
        if st.button("🔄 重置", use_container_width=True):
            reset_flight(); st.rerun()
 
    if ss.auto_flight_enabled and not ss.flight_paused:
        route = ss.planned_route
        if route and ss.current_waypoint_idx < len(route)-1:
            step_forward()
            st_autorefresh(interval=350, key="afr")
        else:
            ss.auto_flight_enabled = False
            st.success("✅ 已到达终点！任务完成")
 
    total_wp = len(ss.planned_route) if ss.planned_route else 0
    cur_wp   = ss.current_waypoint_idx+1
    c1,c2,c3,c4,c5,c6 = st.columns(6)
    with c1: st.metric("当前航点", f"{cur_wp}/{total_wp}")
    with c2: st.metric("飞行速度", f"{ss.flight_speed:.1f} m/s")
    with c3:
        m_,s_ = ss.flight_time_elapsed//60, ss.flight_time_elapsed%60
        st.metric("已用时间", f"{m_:02d}:{s_:02d}")
    with c4: st.metric("剩余距离", f"{ss.flight_remaining_dist:.0f} m")
    with c5:
        eta = int(ss.flight_remaining_dist/ss.flight_speed) if ss.flight_speed>0 else 0
        st.metric("预计到达", f"{eta//60:02d}:{eta%60:02d}")
    with c6:
        b = ss.flight_battery
        st.metric("电量模拟", f"{'🟢' if b>50 else '🟡' if b>20 else '🔴'} {b:.0f}%")
    st.progress(ss.flight_progress,
                text=f"任务进度: {ss.flight_progress*100:.1f}%  |  {cur_wp}/{total_wp} 航点")
 
# ==================== 障碍物管理 ====================
def save_obs():
    with open(CONFIG_FILE,"w",encoding="utf-8") as f:
        json.dump({"obstacles":st.session_state.obstacles},f,ensure_ascii=False,indent=2)
 
def load_obs():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE,"r",encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles = data.get("obstacles",[])
            return True, len(st.session_state.obstacles)
    return False, 0
 
def auto_load():
    if not st.session_state.obstacles_loaded:
        load_obs(); st.session_state.obstacles_loaded = True
 
def add_obs_from_draw(feature):
    try:
        if feature.get('geometry',{}).get('type')=='Polygon':
            coords = feature['geometry']['coordinates'][0]
            pts    = []
            for c in coords:
                gl,gg = wgs84_to_gcj02(c[1],c[0])
                pts.append([gl,gg])
            if len(pts)>1 and pts[0]==pts[-1]: pts=pts[:-1]
            if len(pts)>=3:
                h = st.session_state.new_obstacle_height
                st.session_state.obstacles.append({
                    "points":pts,"height":h,
                    "created_at":datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                })
                save_obs()
                add_log("业务流程",f"障碍物已添加 | 高度:{h}m | 顶点:{len(pts)}")
                return True
    except Exception as e:
        st.error(f"添加障碍物失败: {e}")
    return False
 
def del_obs(idx):
    if 0<=idx<len(st.session_state.obstacles):
        st.session_state.obstacles.pop(idx); save_obs()
 
def clear_obs():
    st.session_state.obstacles=[]; save_obs()
 
def heartbeat():
    st.session_state.heartbeat_count += 1
    return {"sequence":st.session_state.heartbeat_count,
            "timestamp":datetime.now().strftime("%H:%M:%S"),
            "battery":random.randint(85,100),"signal":random.randint(70,99)}
 
# ==================== 主函数 ====================
def main():
    st.set_page_config(page_title="无人机地面站 v5", layout="wide", page_icon="✈️")
    st.title("✈️ 无人机地面站系统")
    st.caption("卫星实况地图 | Visibility Graph 最短路径规划 | 实时飞行监控")
 
    auto_load()
    hb = heartbeat()
 
    c1,c2,c3,c4,c5 = st.columns(5)
    with c1: st.metric("💓 心跳","在线")
    with c2: st.metric("📡 序列号",hb["sequence"])
    with c3: st.metric("🔋 电量",f"{hb['battery']}%")
    with c4: st.metric("📶 信号",f"{hb['signal']}%")
    with c5: st.metric("🕐 时间",hb["timestamp"])
    st.divider()
 
    render_flight_monitor()
    st.divider()
 
    left, mid, right = st.columns([2,1,1])
 
    # ===== 左栏：地图 =====
    with left:
        st.subheader("🛰️ 实时飞行地图（卫星）")
        mc1,mc2,mc3,mc4,mc5,mc6 = st.columns(6)
        with mc1:
            if st.button("📍 起点A", use_container_width=True): st.session_state.setting_mode="start"
        with mc2:
            if st.button("🏁 终点B", use_container_width=True): st.session_state.setting_mode="end"
        with mc3:
            if st.button("❌ 取消",  use_container_width=True): st.session_state.setting_mode=None
        with mc4:
            if st.button("⬅️ 左绕行", use_container_width=True):
                st.session_state.bypass_strategy="left"; plan_route(); st.rerun()
        with mc5:
            if st.button("➡️ 右绕行", use_container_width=True):
                st.session_state.bypass_strategy="right"; plan_route(); st.rerun()
        with mc6:
            if st.button("🌟 最佳航线", type="primary", use_container_width=True):
                st.session_state.bypass_strategy="best"; plan_route(); st.rerun()
 
        if st.session_state.setting_mode=="start": st.info("🔵 点击地图设置起点A")
        elif st.session_state.setting_mode=="end": st.info("🔴 点击地图设置终点B")
 
        in_flight = st.session_state.auto_flight_enabled or st.session_state.flight_paused or st.session_state.mission_started
        if in_flight:
            st.caption("🟢 绿色实线=已飞轨迹 | 🔵 蓝色虚线=待飞航线 | 🟠 橙色图标=无人机当前位置")
        else:
            st.caption("🔵 蓝色虚线=规划航线 | 🔴 红色=高障碍物(绕行) | 🟠 橙色=低障碍物(飞越) | 黄虚线=安全缓冲区")
 
        try:
            m      = create_map()
            output = st_folium(m, width=820, height=560,
                               key=f"map_{st.session_state.map_key}",
                               returned_objects=["last_active_drawing","last_clicked"])
 
            if output and output.get("last_clicked"):
                ck = output["last_clicked"]
                if ck and "lat" in ck and "lng" in ck:
                    gl,gg = wgs84_to_gcj02(ck["lat"],ck["lng"])
                    if st.session_state.setting_mode=="start":
                        st.session_state.start_point={"lat":gl,"lng":gg,"height":0}
                        st.session_state.setting_mode=None
                        add_log("GCS→OBC→FCU",f"SET_START ({gl:.5f},{gg:.5f})")
                        plan_route(); st.rerun()
                    elif st.session_state.setting_mode=="end":
                        st.session_state.end_point={"lat":gl,"lng":gg,"height":0}
                        st.session_state.setting_mode=None
                        add_log("GCS→OBC→FCU",f"SET_END ({gl:.5f},{gg:.5f})")
                        plan_route(); st.rerun()
 
            if output and output.get("last_active_drawing"):
                feat = output["last_active_drawing"]
                if feat.get("geometry",{}).get("type")=="Polygon":
                    if add_obs_from_draw(feat):
                        st.success("✅ 障碍物已添加，正在重新规划航线…")
                        plan_route(); st.rerun()
        except Exception as e:
            st.error(f"地图错误: {e}")
            import traceback; st.code(traceback.format_exc())
 
        render_comm_logs()
 
    # ===== 中栏：控制面板 =====
    with mid:
        st.subheader("🎮 控制面板")
        with st.expander("📍 起点 A", expanded=True):
            nl = st.number_input("纬度", value=float(st.session_state.start_point["lat"]), format="%.6f", key="sl")
            ng = st.number_input("经度", value=float(st.session_state.start_point["lng"]), format="%.6f", key="sg")
            if nl!=st.session_state.start_point["lat"] or ng!=st.session_state.start_point["lng"]:
                st.session_state.start_point={"lat":nl,"lng":ng,"height":0}; plan_route(); st.rerun()
        with st.expander("🏁 终点 B", expanded=True):
            nl = st.number_input("纬度", value=float(st.session_state.end_point["lat"]), format="%.6f", key="el")
            ng = st.number_input("经度", value=float(st.session_state.end_point["lng"]), format="%.6f", key="eg")
            if nl!=st.session_state.end_point["lat"] or ng!=st.session_state.end_point["lng"]:
                st.session_state.end_point={"lat":nl,"lng":ng,"height":0}; plan_route(); st.rerun()
 
        st.divider()
        st.subheader("✈️ 飞行参数")
        fh = st.number_input("飞行高度 (m)", value=st.session_state.flight_height, step=5, min_value=10, max_value=200)
        if fh!=st.session_state.flight_height: st.session_state.flight_height=fh; plan_route(); st.rerun()
        sr = st.number_input("安全半径 (m)", value=st.session_state.safety_radius, step=1, min_value=3, max_value=50)
        if sr!=st.session_state.safety_radius: st.session_state.safety_radius=sr; plan_route(); st.rerun()
        spd = st.slider("飞行速度 (m/s)", 1.0, 20.0, value=st.session_state.flight_speed, step=0.5)
        if spd!=st.session_state.flight_speed: st.session_state.flight_speed=spd
 
        st.divider()
        st.subheader("⛔ 新障碍物高度")
        st.number_input("障碍物高度 (m)", value=60, step=5, min_value=10, max_value=200, key="new_obstacle_height")
        st.caption("💡 在地图上用多边形工具绘制障碍物区域")
 
        # 算法说明
        st.divider()
        with st.expander("📐 算法说明"):
            st.markdown("""
**Visibility Graph + Dijkstra**
 
1. 对所有高于飞行高度的障碍物做 `safety_radius` 米缓冲区（Shapely buffer）
2. 提取所有缓冲区多边形的外轮廓顶点作为候选绕行节点
3. 在所有节点对之间连线，仅保留**不穿过任何障碍物安全区**的边
4. 以欧氏距离为权重，**Dijkstra 算法**求起点→终点的最短路径
5. 贪心**路径平滑**：跳过视线内的中间节点，进一步缩短距离
 
这保证了：✅ 路径最短 ✅ 全程不碰任何障碍物 ✅ 与障碍物保持安全距离
""")
 
    # ===== 右栏：分析 + 障碍物列表 =====
    with right:
        st.subheader("📊 航线分析")
        if st.session_state.route_analysis:
            a = st.session_state.route_analysis
            st.metric("📏 总距离",    f"{a.get('total_distance',0):.1f} m")
            st.metric("📍 总航点数",  a.get('waypoint_count', 0))
            st.metric("🔄 绕行节点",  a.get('bypass_count', 0))
            st.metric("✅ 飞越次数",  a.get('fly_over_count', 0))
            st.metric("🎯 规划策略",  a.get('strategy_used','未知'))
            st.divider()
            st.caption("📋 障碍物处理")
            for obs in a.get('obstacles_encountered',[]):
                icon = "🔄" if "绕行" in obs['decision'] else "✅"
                st.text(f"{icon} {obs['height']}m → {obs['decision']}")
        else:
            st.info("点击规划按钮生成报告")
 
        st.divider()
        st.subheader("⛔ 障碍物列表")
        if st.session_state.obstacles:
            st.caption(f"共 {len(st.session_state.obstacles)} 个障碍物")
            for idx,obs in enumerate(st.session_state.obstacles):
                ca,cb,cc_ = st.columns([1,2,1])
                with ca:
                    if st.button("🗑️", key=f"d{idx}"): del_obs(idx); plan_route(); st.rerun()
                with cb:
                    st.text(f"障碍 {idx+1}")
                    if obs.get('created_at'): st.caption(obs['created_at'][:10])
                with cc_:
                    h_ = obs.get('height',10)
                    st.text(f"{'🔴' if h_>st.session_state.flight_height else '🟠'} {h_}m")
        else:
            st.info("暂无障碍物\n在地图绘制多边形添加")
 
        st.divider()
        cs,cl,cc2 = st.columns(3)
        with cs:
            if st.button("💾 保存", use_container_width=True): save_obs(); st.success("已保存")
        with cl:
            if st.button("📂 加载", use_container_width=True):
                ok,cnt = load_obs()
                if ok: st.success(f"加载{cnt}个"); plan_route(); st.rerun()
                else: st.warning("无保存文件")
        with cc2:
            if st.button("🗑️ 清空", use_container_width=True): clear_obs(); plan_route(); st.rerun()
 
if __name__ == "__main__":
    main()
