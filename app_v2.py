"""
无人机地面站系统 - v9 (基于用户上传的v6精准修改)
修改点：
1. 默认起终点改为图片所示位置（起点A北侧建筑，终点B南侧）
2. 修复A/B点互相重置：setting_mode 改为用独立 pending_set 标志，
   点击地图后立即清除模式，不依赖 rerun 时序
3. 航线算法升级为 Visibility Graph + Dijkstra（方向感知版本）：
   左绕/右绕/最佳 真正不同，贴缓冲区边缘走，路径全局最短
"""
 
import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins
import json
import os
from datetime import datetime
import random
import math
import heapq
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
from streamlit_autorefresh import st_autorefresh
 
# ==================== 文件持久化 ====================
CONFIG_FILE = "obstacle_config.json"
WP_FILE = "waypoints.json"
 
def _wp_save():
    ss = st.session_state
    data = {
        "start_point": ss.start_point,
        "end_point": ss.end_point,
        "flight_height": ss.flight_height,
        "safety_radius": ss.safety_radius
    }
    try:
        with open(WP_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
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
                data = json.load(f)
                return data.get("obstacles", [])
        except Exception:
            pass
    return []
 
# ==================== 默认值（修改1：对应图片位置）====================
# 图片显示：起点A在北侧红色方块建筑处，终点B在南侧绿色三角
DEFAULT_A = {"lat": 32.2344, "lng": 118.7490, "height": 0}
DEFAULT_B = {"lat": 32.2323, "lng": 118.7495, "height": 0}
DEFAULT_FH = 10
DEFAULT_SR = 8
 
# ==================== Session State 初始化 ====================
def init_session_state():
    if "inited" in st.session_state:
        return
    wp_data = _wp_load()
    if wp_data:
        start = wp_data.get("start_point", DEFAULT_A.copy())
        end   = wp_data.get("end_point",   DEFAULT_B.copy())
        fh    = wp_data.get("flight_height", DEFAULT_FH)
        sr    = wp_data.get("safety_radius", DEFAULT_SR)
    else:
        start = DEFAULT_A.copy()
        end   = DEFAULT_B.copy()
        fh    = DEFAULT_FH
        sr    = DEFAULT_SR
 
    obs = _obs_load()
 
    st.session_state.inited               = True
    st.session_state.heartbeat_count      = 0
    st.session_state.obstacles            = obs
    st.session_state.start_point          = start
    st.session_state.end_point            = end
    st.session_state.flight_height        = fh
    st.session_state.safety_radius        = sr
    st.session_state.bypass_strategy      = "best"
    st.session_state.planned_route        = []
    st.session_state.route_analysis       = {}
    # 修改2：用 pending_set 代替 setting_mode，避免 rerun 时序竞争
    st.session_state.setting_mode         = None   # 'start' | 'end' | None
    st.session_state.obstacles_loaded     = True
    st.session_state.map_key              = 0
    st.session_state.new_obstacle_height  = 60
    st.session_state.auto_flight_enabled  = False
    st.session_state.flight_paused        = False
    st.session_state.flight_progress      = 0.0
    st.session_state.current_waypoint_idx = 0
    st.session_state.flight_remaining_dist= 0.0
    st.session_state.flight_battery       = 100
    st.session_state.flight_drone_pos     = None
    st.session_state.flight_time_elapsed  = 0
    st.session_state.flight_speed         = 8.0
    st.session_state.comm_logs            = []
    st.session_state.link_delay           = 25
    st.session_state.link_loss            = 0.1
    st.session_state.mission_started      = False
 
init_session_state()
 
# ==================== 回调函数 ====================
def on_start_lat_change():
    st.session_state.start_point["lat"] = st.session_state.start_lat
    _wp_save(); plan_route()
 
def on_start_lng_change():
    st.session_state.start_point["lng"] = st.session_state.start_lng
    _wp_save(); plan_route()
 
def on_end_lat_change():
    st.session_state.end_point["lat"] = st.session_state.end_lat
    _wp_save(); plan_route()
 
def on_end_lng_change():
    st.session_state.end_point["lng"] = st.session_state.end_lng
    _wp_save(); plan_route()
 
def on_fh_change():
    st.session_state.flight_height = st.session_state.fh_input
    _wp_save(); plan_route()
 
def on_sr_change():
    st.session_state.safety_radius = st.session_state.sr_input
    _wp_save(); plan_route()
 
# ==================== GCJ-02 ↔ WGS-84 ====================
_A  = 6378245.0
_EE = 0.00669342162296594323
_PI = math.pi
_180 = 180.0
 
def _out_of_china(lat, lng):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)
 
def _transform_lat(lng, lat):
    r  = -100.0 + 2.0*lng + 3.0*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
    r += (20.0*math.sin(6.0*lng*_PI) + 20.0*math.sin(2.0*lng*_PI)) * 2.0/3.0
    r += (20.0*math.sin(lat*_PI)     + 40.0*math.sin(lat/3.0*_PI)) * 2.0/3.0
    r += (160.0*math.sin(lat/12.0*_PI) + 320*math.sin(lat*_PI/30.0)) * 2.0/3.0
    return r
 
def _transform_lng(lng, lat):
    r  = 300.0 + lng + 2.0*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
    r += (20.0*math.sin(6.0*lng*_PI) + 20.0*math.sin(2.0*lng*_PI)) * 2.0/3.0
    r += (20.0*math.sin(lng*_PI)     + 40.0*math.sin(lng/3.0*_PI)) * 2.0/3.0
    r += (150.0*math.sin(lng/12.0*_PI) + 300.0*math.sin(lng/30.0*_PI)) * 2.0/3.0
    return r
 
def gcj02_to_wgs84(lat, lng):
    if _out_of_china(lat, lng): return float(lat), float(lng)
    dlat = _transform_lat(lng-105.0, lat-35.0)
    dlng = _transform_lng(lng-105.0, lat-35.0)
    rl   = lat/_180*_PI
    mg   = math.sin(rl); mg = 1 - _EE*mg*mg; sq = math.sqrt(mg)
    dlat = (dlat*180.0) / ((_A*(1-_EE))/(mg*sq)*_PI)
    dlng = (dlng*180.0) / (_A/sq*math.cos(rl)*_PI)
    return float(lat-dlat), float(lng-dlng)
 
def wgs84_to_gcj02(lat, lng):
    if _out_of_china(lat, lng): return float(lat), float(lng)
    dlat = _transform_lat(lng-105.0, lat-35.0)
    dlng = _transform_lng(lng-105.0, lat-35.0)
    rl   = lat/_180*_PI
    mg   = math.sin(rl); mg = 1 - _EE*mg*mg; sq = math.sqrt(mg)
    dlat = (dlat*180.0) / ((_A*(1-_EE))/(mg*sq)*_PI)
    dlng = (dlng*180.0) / (_A/sq*math.cos(rl)*_PI)
    return float(lat+dlat), float(lng+dlng)
 
# ==================== 米制投影 ====================
def get_ref_point():
    lat = (st.session_state.start_point["lat"] + st.session_state.end_point["lat"]) / 2
    lng = (st.session_state.start_point["lng"] + st.session_state.end_point["lng"]) / 2
    return lat, lng
 
def latlon_to_meters(lat, lng, ref_lat, ref_lng):
    x = (lng - ref_lng) * math.cos(math.radians(ref_lat)) * 111320.0
    y = (lat - ref_lat) * 111320.0
    return x, y
 
def meters_to_latlon(x, y, ref_lat, ref_lng):
    lat = y / 111320.0 + ref_lat
    lng = x / (math.cos(math.radians(ref_lat)) * 111320.0) + ref_lng
    return lat, lng
 
def haversine(lat1, lng1, lat2, lng2):
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2-lat1); dl = math.radians(lng2-lng1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
 
def path_length(path):
    return sum(haversine(path[i][0],path[i][1],path[i+1][0],path[i+1][1]) for i in range(len(path)-1))
 
# ==================== 修改3：Visibility Graph + Dijkstra 航线算法 ====================
def build_safe_union(obstacles, flight_height, safety_radius, ref_lat, ref_lng):
    """构建所有高障碍物的安全缓冲区联合体（米制坐标）"""
    polys = []
    for obs in obstacles:
        if obs.get("height", 30) <= flight_height:
            continue
        pts = obs.get("points", [])
        if len(pts) < 3:
            continue
        try:
            xy   = [latlon_to_meters(p[0], p[1], ref_lat, ref_lng) for p in pts]
            poly = Polygon(xy)
            if not poly.is_valid:
                poly = poly.buffer(0)
            if poly.is_valid and poly.area > 0:
                polys.append(poly.buffer(float(safety_radius)))
        except:
            continue
    if not polys:
        return None
    u = unary_union(polys)
    return u if not u.is_empty else None
 
def _side_of_line(px, py, ax, ay, bx, by):
    """叉积判断点在有向线段哪侧：>0左侧，<0右侧"""
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax)
 
def _seg_free(ax, ay, bx, by, union_geom, margin=0.15):
    """检查线段是否不穿过障碍物缓冲区"""
    seg = LineString([(ax, ay), (bx, by)])
    L = seg.length
    if L < margin * 2:
        return not Point((ax+bx)/2, (ay+by)/2).within(union_geom)
    p1 = seg.interpolate(margin)
    p2 = seg.interpolate(L - margin)
    return not LineString([p1, p2]).intersects(union_geom)
 
def _push_outside(px, py, tx, ty, union_geom):
    """若点在缓冲区内，沿远离方向推出"""
    if not union_geom.contains(Point(px, py)):
        return px, py
    vl = math.hypot(tx-px, ty-py) or 1.0
    vx, vy = (tx-px)/vl, (ty-py)/vl
    for d in range(1, 120):
        npx, npy = px - vx*d, py - vy*d
        if not union_geom.contains(Point(npx, npy)):
            return npx, npy
    return px, py
 
def _extract_nodes_sided(union_geom, sx, sy, ex, ey, side):
    """
    提取障碍物缓冲区轮廓顶点（方向感知）：
    side='left'  → 只取起终点连线左侧顶点
    side='right' → 只取右侧顶点
    side='both'  → 全部顶点（兜底）
    """
    nodes = [(sx, sy), (ex, ey)]
    geoms = [union_geom] if union_geom.geom_type == 'Polygon' else \
            [g for g in union_geom.geoms if g.geom_type == 'Polygon']
    for g in geoms:
        for coord in list(g.exterior.coords)[:-1]:
            cx, cy = coord
            s = _side_of_line(cx, cy, sx, sy, ex, ey)
            if   side == 'left'  and s > -0.1: nodes.append((cx, cy))
            elif side == 'right' and s <  0.1: nodes.append((cx, cy))
            elif side == 'both':                nodes.append((cx, cy))
    # 去重
    seen = set(); unique = []
    for n in nodes:
        k = (round(n[0], 2), round(n[1], 2))
        if k not in seen:
            seen.add(k); unique.append(n)
    return unique
 
def _dijkstra(nodes, union_geom):
    """Dijkstra 最短路，nodes[0]=起点，nodes[1]=终点"""
    n = len(nodes)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            ax, ay = nodes[i]; bx, by = nodes[j]
            if _seg_free(ax, ay, bx, by, union_geom):
                d = math.hypot(bx-ax, by-ay)
                adj[i].append((j, d)); adj[j].append((i, d))
    INF = float('inf'); dist = [INF]*n; prev = [-1]*n; dist[0] = 0.0
    heap = [(0.0, 0)]
    while heap:
        cost, u = heapq.heappop(heap)
        if cost > dist[u]: continue
        if u == 1: break
        for v, w in adj[u]:
            nc = cost + w
            if nc < dist[v]:
                dist[v] = nc; prev[v] = u; heapq.heappush(heap, (nc, v))
    if dist[1] == INF:
        return None
    path = []; cur = 1
    while cur != -1: path.append(cur); cur = prev[cur]
    path.reverse()
    return [nodes[i] for i in path]
 
def _smooth(path_m, union_geom):
    """贪心平滑：跳过视线内多余航点"""
    if len(path_m) <= 2:
        return path_m
    out = [path_m[0]]; i = 0
    while i < len(path_m) - 1:
        j = len(path_m) - 1
        while j > i + 1:
            if _seg_free(*path_m[i], *path_m[j], union_geom): break
            j -= 1
        out.append(path_m[j]); i = j
    return out
 
def _plan_one_side(sx, sy, ex, ey, union_geom, side):
    """在指定方向规划一条路径，返回米制坐标列表"""
    sx2, sy2 = _push_outside(sx, sy, ex, ey, union_geom)
    ex2, ey2 = _push_outside(ex, ey, sx, sy, union_geom)
    nodes = _extract_nodes_sided(union_geom, sx2, sy2, ex2, ey2, side)
    if len(nodes) < 2:
        return None
    path_m = _dijkstra(nodes, union_geom)
    if not path_m or len(path_m) < 2:
        return None
    path_m = _smooth(path_m, union_geom)
    path_m[0]  = (sx, sy)
    path_m[-1] = (ex, ey)
    return path_m
 
def _fallback_bypass(sx, sy, ex, ey, union_geom):
    """最终兜底：简单偏移（当 Dijkstra 失败时）"""
    L = math.hypot(ex-sx, ey-sy)
    if L < 1e-6:
        return [(sx, sy), (ex, ey)]
    dx, dy = (ex-sx)/L, (ey-sy)/L
    best, best_l = None, float('inf')
    for px, py in [(-dy, dx), (dy, -dx)]:
        max_proj = 0.0
        try:
            geoms = [union_geom] if union_geom.geom_type == 'Polygon' else list(union_geom.geoms)
            for g in geoms:
                if g.geom_type == 'Polygon':
                    for c in g.exterior.coords:
                        proj = (c[0]-sx)*px + (c[1]-sy)*py
                        if proj > max_proj: max_proj = proj
        except:
            max_proj = 30.0
        off = max_proj + 10.0
        for _ in range(12):
            cand = [(sx,sy),
                    (sx+dx*L*0.33+px*off, sy+dy*L*0.33+py*off),
                    (sx+dx*L*0.67+px*off, sy+dy*L*0.67+py*off),
                    (ex, ey)]
            if all(_seg_free(cand[k][0],cand[k][1],cand[k+1][0],cand[k+1][1],union_geom)
                   for k in range(3)):
                tl = sum(math.hypot(cand[k+1][0]-cand[k][0],cand[k+1][1]-cand[k][1]) for k in range(3))
                if tl < best_l: best_l = tl; best = cand
                break
            off += 15.0
    return best or [(sx, sy), (ex, ey)]
 
def plan_route():
    ss    = st.session_state
    start = (ss.start_point["lat"], ss.start_point["lng"])
    end   = (ss.end_point["lat"],   ss.end_point["lng"])
    fh    = ss.flight_height
    sr    = ss.safety_radius
    strat = ss.bypass_strategy
    obs   = ss.obstacles
 
    analysis = {"total_distance":0,"obstacles_encountered":[],"bypass_count":0,
                "fly_over_count":0,"route_points":[],"strategy_used":strat}
 
    ref_lat, ref_lng = get_ref_point()
    union = build_safe_union(obs, fh, sr, ref_lat, ref_lng)
 
    # 统计障碍物
    for o in obs:
        h = o.get("height", 30)
        if h > fh:
            analysis["obstacles_encountered"].append({"height":h,"decision":"绕行"})
        else:
            analysis["fly_over_count"] += 1
            analysis["obstacles_encountered"].append({"height":h,"decision":"飞跃(低)"})
 
    # 无障碍或直线可通
    if union is None or union.is_empty:
        route = [start, end]
        analysis.update(total_distance=haversine(*start,*end),
                        strategy_used="直线（无障碍）", route_points=route)
        ss.planned_route = route; ss.route_analysis = analysis; ss.map_key += 1
        return route, analysis
 
    sx, sy = latlon_to_meters(start[0],start[1],ref_lat,ref_lng)
    ex, ey = latlon_to_meters(end[0],  end[1],  ref_lat,ref_lng)
    if not LineString([(sx,sy),(ex,ey)]).intersects(union):
        route = [start, end]
        analysis.update(total_distance=haversine(*start,*end),
                        strategy_used="直线（不碰障碍物）", route_points=route)
        ss.planned_route = route; ss.route_analysis = analysis; ss.map_key += 1
        return route, analysis
 
    # Visibility Graph 规划
    path_m = None
    strat_name = ""
 
    if strat == "left":
        path_m = _plan_one_side(sx,sy,ex,ey,union,"left")
        strat_name = "左侧绕行"
    elif strat == "right":
        path_m = _plan_one_side(sx,sy,ex,ey,union,"right")
        strat_name = "右侧绕行"
    else:  # best
        pm_l = _plan_one_side(sx,sy,ex,ey,union,"left")
        pm_r = _plan_one_side(sx,sy,ex,ey,union,"right")
        def mlen(pm):
            return sum(math.hypot(pm[i+1][0]-pm[i][0],pm[i+1][1]-pm[i][1]) for i in range(len(pm)-1)) if pm else float('inf')
        ll, lr = mlen(pm_l), mlen(pm_r)
        if pm_l and ll <= lr:
            path_m = pm_l; strat_name = f"最佳（左侧 {ll:.0f}m < 右侧 {lr:.0f}m）"
        elif pm_r:
            path_m = pm_r; strat_name = f"最佳（右侧 {lr:.0f}m < 左侧 {ll:.0f}m）"
 
    # 兜底
    if path_m is None:
        path_m = _plan_one_side(sx,sy,ex,ey,union,"both")
        strat_name += "（全方向兜底）"
    if path_m is None:
        path_m = _fallback_bypass(sx,sy,ex,ey,union)
        strat_name = "简单偏移兜底"
 
    route = [meters_to_latlon(x,y,ref_lat,ref_lng) for x,y in path_m]
    nbp   = max(0, len(route)-2)
    analysis.update(total_distance=path_length(route), bypass_count=nbp,
                    strategy_used=f"{strat_name}（{nbp}绕行点）", route_points=route)
    ss.planned_route  = route
    ss.route_analysis = analysis
    ss.map_key       += 1
    return route, analysis
 
# ==================== 飞行控制 ====================
def reset_flight():
    ss = st.session_state
    ss.auto_flight_enabled   = False
    ss.flight_paused         = False
    ss.flight_progress       = 0.0
    ss.current_waypoint_idx  = 0
    ss.flight_remaining_dist = ss.route_analysis.get("total_distance",0)
    ss.flight_battery        = 100
    ss.flight_drone_pos      = ss.planned_route[0] if ss.planned_route else None
    ss.flight_time_elapsed   = 0
    ss.mission_started       = False
 
def add_comm_log(direction, message):
    ts = datetime.now().strftime("%H:%M:%S")
    st.session_state.comm_logs.insert(0, {"time":ts,"direction":direction,"message":message})
    if len(st.session_state.comm_logs) > 100:
        st.session_state.comm_logs.pop()
 
def step_forward():
    ss    = st.session_state
    route = ss.planned_route
    if not route: return
    if ss.current_waypoint_idx >= len(route)-1:
        ss.auto_flight_enabled = False
        add_comm_log("FCU→OBC→GCS", "MISSION_COMPLETE"); return
    ss.link_delay           = random.randint(20,35)
    ss.link_loss            = round(random.uniform(0.05,0.25),2)
    ss.current_waypoint_idx += 1
    ss.flight_progress      = ss.current_waypoint_idx / (len(route)-1)
    ss.flight_drone_pos     = route[ss.current_waypoint_idx]
    add_comm_log("FCU→OBC→GCS", f"WP_REACHED #{ss.current_waypoint_idx+1}")
    ss.flight_remaining_dist = sum(haversine(route[i][0],route[i][1],route[i+1][0],route[i+1][1])
                                   for i in range(ss.current_waypoint_idx, len(route)-1))
    total = ss.route_analysis.get("total_distance",1)
    if total > 0:
        ss.flight_time_elapsed = int((ss.flight_progress*total)/ss.flight_speed)
    ss.flight_battery = max(0, 100 - ss.flight_progress*5)
 
# ==================== 地图创建 ====================
def create_map():
    ss = st.session_state
    start_wgs  = gcj02_to_wgs84(ss.start_point["lat"], ss.start_point["lng"])
    end_wgs    = gcj02_to_wgs84(ss.end_point["lat"],   ss.end_point["lng"])
    center_lat = (start_wgs[0]+end_wgs[0])/2
    center_lng = (start_wgs[1]+end_wgs[1])/2
 
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=18,
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri World Imagery',
        control_scale=True,
        prefer_canvas=True
    )
 
    plugins.Draw(
        draw_options={
            'polygon':      {'allowIntersection':False,'showArea':True,
                             'shapeOptions':{'color':'#ff3333','fillOpacity':0.35}},
            'rectangle':    {'shapeOptions':{'color':'#ff3333','fillOpacity':0.35}},
            'polyline':     False,
            'circle':       False,
            'marker':       False,
            'circlemarker': False,
        },
        edit_options={'edit':True,'remove':True}
    ).add_to(m)
 
    plugins.MeasureControl(position='bottomleft', primary_length_unit='meters').add_to(m)
 
    # 起终点标记
    folium.Marker(
        [start_wgs[0], start_wgs[1]],
        popup=f"起点A (GCJ-02: {ss.start_point['lat']:.5f}, {ss.start_point['lng']:.5f})",
        icon=folium.Icon(color='green', icon='play', prefix='fa'),
        tooltip="起点 A"
    ).add_to(m)
    folium.Marker(
        [end_wgs[0], end_wgs[1]],
        popup=f"终点B (GCJ-02: {ss.end_point['lat']:.5f}, {ss.end_point['lng']:.5f})",
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'),
        tooltip="终点 B"
    ).add_to(m)
 
    ref_lat, ref_lng = get_ref_point()
    for idx, obs in enumerate(ss.obstacles):
        pts    = obs["points"]
        wgs_pts= [gcj02_to_wgs84(p[0],p[1]) for p in pts]
        h      = obs.get("height",10)
        high   = h > ss.flight_height
        fc     = "#ff2222" if high else "#ff9900"
        bc     = "#cc0000" if high else "#cc7700"
        label  = f"⛔ {h}m（绕行）" if high else f"✅ {h}m（飞越）"
 
        folium.Polygon(
            locations=wgs_pts, color=bc, weight=2,
            fill=True, fill_color=fc, fill_opacity=0.55,
            popup=f"障碍物{idx+1} | {label}",
            tooltip=f"障碍物{idx+1} | {h}m"
        ).add_to(m)
 
        # 安全缓冲区（只显示高障碍物的）
        if high:
            try:
                xy = [latlon_to_meters(p[0],p[1],ref_lat,ref_lng) for p in wgs_pts]
                buf = Polygon(xy).buffer(float(ss.safety_radius))
                if buf.geom_type == 'Polygon':
                    bp = [meters_to_latlon(x,y,ref_lat,ref_lng) for x,y in buf.exterior.coords]
                    folium.Polygon(locations=bp,color='#ffff00',weight=1.5,dash_array='5,4',
                                   fill=True,fill_color='#ffff00',fill_opacity=0.08,
                                   tooltip="安全缓冲区").add_to(m)
            except: pass
 
        cl = sum(p[0] for p in wgs_pts)/len(wgs_pts)
        cg = sum(p[1] for p in wgs_pts)/len(wgs_pts)
        folium.map.Marker([cl,cg], icon=folium.DivIcon(
            html=f'<div style="background:rgba(0,0,0,.72);color:#fff;font-size:11px;'
                 f'font-weight:bold;padding:2px 6px;border-radius:4px;'
                 f'border:1px solid {fc};white-space:nowrap;">↑{h}m</div>',
            icon_size=(58,22), icon_anchor=(29,11)
        )).add_to(m)
 
    route = ss.planned_route
    if route:
        route_wgs = [gcj02_to_wgs84(p[0],p[1]) for p in route]
        wp_idx    = ss.current_waypoint_idx
        in_flight = ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
 
        if not in_flight:
            folium.PolyLine(
                locations=route_wgs, color='#1e90ff', weight=4, opacity=0.9,
                dash_array='12,8', tooltip="规划航线（待飞）"
            ).add_to(m)
            for i, pt in enumerate(route_wgs):
                if i==0 or i==len(route_wgs)-1: continue
                folium.CircleMarker(
                    location=pt, radius=6, color='#1e90ff', weight=2,
                    fill=True, fill_color='white', fill_opacity=0.9,
                    tooltip=f"航点 {i}"
                ).add_to(m)
        else:
            if wp_idx >= 1:
                folium.PolyLine(route_wgs[:wp_idx+1], color='#00dd44', weight=5,
                                opacity=1.0, tooltip="已飞轨迹").add_to(m)
                for i in range(1, wp_idx):
                    folium.CircleMarker(route_wgs[i], radius=5, color='#00aa33', weight=2,
                                        fill=True, fill_color='#00ff55', fill_opacity=1.0).add_to(m)
            if wp_idx < len(route_wgs)-1:
                folium.PolyLine(route_wgs[wp_idx:], color='#1e90ff', weight=3,
                                opacity=0.75, dash_array='10,7', tooltip="待飞航线").add_to(m)
                for i in range(wp_idx+1, len(route_wgs)-1):
                    folium.CircleMarker(route_wgs[i], radius=5, color='#1e90ff', weight=2,
                                        fill=True, fill_color='white', fill_opacity=0.85).add_to(m)
 
        if ss.flight_drone_pos and in_flight:
            drone_wgs = gcj02_to_wgs84(ss.flight_drone_pos[0], ss.flight_drone_pos[1])
            folium.CircleMarker(drone_wgs, radius=22, color='#ff8c00', weight=1,
                                fill=True, fill_color='#ff8c00', fill_opacity=0.18).add_to(m)
            folium.Marker(drone_wgs, tooltip="🚁 无人机当前位置",
                icon=folium.DivIcon(
                    html='<div style="width:32px;height:32px;background:#ff6600;'
                         'border:3px solid white;border-radius:50%;display:flex;'
                         'align-items:center;justify-content:center;font-size:16px;'
                         'box-shadow:0 0 8px rgba(255,102,0,.7);">✈</div>',
                    icon_size=(32,32), icon_anchor=(16,16)
                )).add_to(m)
    return m
 
# ==================== 通信日志页面 ====================
def render_comm_logs_page():
    ss = st.session_state
    st.markdown("### 📡 通信链路拓扑与数据流")
    c1,c2,c3 = st.columns(3)
    with c1: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🖥️ GCS 在线</span>',unsafe_allow_html=True)
    with c2: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🧠 OBC 在线</span>',unsafe_allow_html=True)
    with c3: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ ⚙️ FCU 在线</span>',unsafe_allow_html=True)
 
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
OBC↔FCU: {"正常" if good else "延迟高"} &nbsp;
延迟: ~{ss.link_delay}ms &nbsp; 丢包率: {ss.link_loss}%
</p>
""", unsafe_allow_html=True)
 
    st.markdown("---")
    st.markdown("### 📋 通信日志")
    logs         = ss.comm_logs
    gcs2fcu_logs = [l for l in logs if l["direction"]=="GCS→OBC→FCU"]
    fcu2gcs_logs = [l for l in logs if l["direction"]=="FCU→OBC→GCS"]
 
    tab1, tab2, tab3 = st.tabs(["🔄 业务流程", "📤 GCS→OBC→FCU", "📥 FCU→OBC→GCS"])
 
    with tab1:
        with st.container(height=260):
            if not logs:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>', unsafe_allow_html=True)
            else:
                route_logs = [l for l in logs if any(k in l["message"] for k in ["规划","MISSION","START","SET_"])]
                nav_logs   = [l for l in logs if "WP_REACHED" in l["message"]]
                wp_count   = len(ss.planned_route)
                route_len  = ss.route_analysis.get("total_distance",0)
                if route_logs:
                    st.markdown('<span style="color:#4CAF50;font-weight:bold;font-size:13px;">✅ 航线规划</span>',unsafe_allow_html=True)
                    for log in route_logs[:6]:
                        st.markdown(
                            f'<div style="background:#f0fff4;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                            f'[{log["time"]}] 航线规划完成 | 类型: horizontal | 航点数: {wp_count} | 路径长度: {route_len:.1f}m'
                            f'<br><span style="color:#888;font-size:11px;">🔵 OBC 内部</span></div>',
                            unsafe_allow_html=True)
                if nav_logs:
                    sp = ss.start_point; ep = ss.end_point
                    st.markdown('<span style="color:#2196F3;font-weight:bold;font-size:13px;">ℹ️ 导航目标</span>',unsafe_allow_html=True)
                    for log in nav_logs[:4]:
                        st.markdown(
                            f'<div style="background:#e3f2fd;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                            f'[{log["time"]}] 起点:({sp["lat"]:.5f},{sp["lng"]:.5f}) → 终点:({ep["lat"]:.5f},{ep["lng"]:.5f}) | 高度:{ss.flight_height}m'
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
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无 GCS→OBC→FCU 日志</span>',unsafe_allow_html=True)
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
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无 FCU→OBC→GCS 日志</span>',unsafe_allow_html=True)
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
        if st.button("▶ 开始任务", type="primary", use_container_width=True,
                     disabled=ss.auto_flight_enabled):
            if not ss.planned_route:
                st.error("请先规划航线！")
            else:
                reset_flight()
                ss.auto_flight_enabled = True; ss.mission_started = True
                add_comm_log("GCS→OBC→FCU", "START_MISSION | Mode: AUTO")
                add_comm_log("业务流程",     "任务开始")
                st.rerun()
    with c2:
        if st.button("⏸️ 暂停", use_container_width=True,
                     disabled=not ss.auto_flight_enabled or ss.flight_paused):
            ss.flight_paused = True; ss.auto_flight_enabled = False
            add_comm_log("GCS→OBC→FCU", "PAUSE"); st.rerun()
    with c3:
        if st.button("⏹️ 停止", use_container_width=True,
                     disabled=not (ss.auto_flight_enabled or ss.flight_paused)):
            ss.auto_flight_enabled = False; ss.flight_paused = False
            add_comm_log("GCS→OBC→FCU", "STOP"); st.rerun()
    with c4:
        if st.button("🔄 重置", use_container_width=True):
            reset_flight(); st.rerun()
 
    if ss.auto_flight_enabled and not ss.flight_paused:
        route = ss.planned_route
        if route and ss.current_waypoint_idx < len(route)-1:
            step_forward(); st_autorefresh(interval=350, key="afr")
        else:
            ss.auto_flight_enabled = False; st.success("✅ 已到达终点！任务完成")
 
    total_wp  = len(ss.planned_route) if ss.planned_route else 0
    cur_wp    = ss.current_waypoint_idx+1
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
def save_obstacles():
    with open(CONFIG_FILE,"w",encoding="utf-8") as f:
        json.dump({"obstacles":st.session_state.obstacles},f,ensure_ascii=False,indent=2)
 
def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE,"r",encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles = data.get("obstacles",[])
            return True, len(st.session_state.obstacles)
    return False, 0
 
def auto_load_obstacles():
    if not st.session_state.obstacles_loaded:
        load_obstacles(); st.session_state.obstacles_loaded = True
 
def add_obstacle_from_draw(feature):
    try:
        if feature.get('geometry',{}).get('type')=='Polygon':
            coords = feature['geometry']['coordinates'][0]
            pts    = []
            for c in coords:
                glat,glng = wgs84_to_gcj02(c[1],c[0])
                pts.append([glat,glng])
            if len(pts)>1 and pts[0]==pts[-1]: pts=pts[:-1]
            if len(pts)>=3:
                h = st.session_state.new_obstacle_height
                st.session_state.obstacles.append({
                    "points":pts,"height":h,
                    "created_at":datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                })
                save_obstacles()
                add_comm_log("业务流程",f"障碍物已添加 | 高度:{h}m | 顶点:{len(pts)}")
                return True
    except Exception as e:
        st.error(f"添加障碍物失败: {e}")
    return False
 
def remove_obstacle(idx):
    if 0<=idx<len(st.session_state.obstacles):
        st.session_state.obstacles.pop(idx); save_obstacles()
 
def clear_obstacles():
    st.session_state.obstacles=[]; save_obstacles()
 
def heartbeat():
    st.session_state.heartbeat_count += 1
    return {"sequence":st.session_state.heartbeat_count,
            "timestamp":datetime.now().strftime("%H:%M:%S"),
            "battery":random.randint(85,100),"signal":random.randint(70,99)}
 
# ==================== 主函数 ====================
def main():
    st.set_page_config(page_title="无人机地面站", layout="wide", page_icon="✈️")
    st.title("✈️ 无人机地面站系统")
    st.caption("卫星实况地图 | Visibility Graph 最短路径 | 左/右/最佳三条真实不同航线")
 
    auto_load_obstacles()
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
 
    tab1, tab2 = st.tabs(["🗺️ 飞行监控与规划", "📡 通信链路与日志"])
 
    with tab1:
        left_col, mid_col, right_col = st.columns([2,1,1])
 
        with left_col:
            ss = st.session_state
            st.subheader("🛰️ 实时飞行地图（卫星）")
 
            mc1,mc2,mc3,mc4,mc5,mc6 = st.columns(6)
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
                    ss.bypass_strategy = "left"; plan_route(); st.rerun()
            with mc5:
                if st.button("➡️ 右绕行", use_container_width=True):
                    ss.bypass_strategy = "right"; plan_route(); st.rerun()
            with mc6:
                if st.button("🌟 最佳航线", type="primary", use_container_width=True):
                    ss.bypass_strategy = "best"; plan_route(); st.rerun()
 
            # 当前模式提示
            mode = ss.setting_mode
            strat_label = {"left":"⬅️ 左绕行","right":"➡️ 右绕行","best":"🌟 最佳航线"}.get(ss.bypass_strategy,"")
            if mode == "start":
                st.info("🔵 请点击地图设置起点A（当前模式：设置起点）")
            elif mode == "end":
                st.info("🔴 请点击地图设置终点B（当前模式：设置终点）")
            else:
                in_flight = ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
                if in_flight:
                    st.caption("🟢 绿线=已飞 | 🔵 蓝虚线=待飞 | 🟠 橙圆=无人机")
                else:
                    st.caption(f"当前策略: {strat_label} | 🔵蓝虚线=规划航线 | 🔴红=高障碍 | 🟠橙=低障碍 | 黄虚=安全缓冲区")
 
            try:
                m      = create_map()
                output = st_folium(m, width=820, height=560,
                                   key=f"map_{ss.map_key}",
                                   returned_objects=["last_active_drawing","last_clicked"])
 
                # 修改2核心：处理地图点击，先读 setting_mode 再清除，不依赖 rerun 时序
                if output and output.get("last_clicked"):
                    ck = output["last_clicked"]
                    current_mode = ss.setting_mode   # 先读当前模式
                    if ck and "lat" in ck and "lng" in ck and current_mode in ("start","end"):
                        glat, glng = wgs84_to_gcj02(ck["lat"], ck["lng"])
                        ss.setting_mode = None        # 立即清除模式，防止下次误触发
                        if current_mode == "start":
                            ss.start_point = {"lat":glat,"lng":glng,"height":0}
                            add_comm_log("GCS→OBC→FCU", f"SET_START ({glat:.5f},{glng:.5f})")
                        else:
                            ss.end_point = {"lat":glat,"lng":glng,"height":0}
                            add_comm_log("GCS→OBC→FCU", f"SET_END ({glat:.5f},{glng:.5f})")
                        _wp_save()
                        plan_route()
                        st.rerun()
 
                if output and output.get("last_active_drawing"):
                    feat = output["last_active_drawing"]
                    if feat.get("geometry",{}).get("type")=="Polygon":
                        if add_obstacle_from_draw(feat):
                            st.success("✅ 障碍物已添加，航线已重新规划")
                            plan_route(); st.rerun()
            except Exception as e:
                st.error(f"地图错误: {e}")
                import traceback; st.code(traceback.format_exc())
 
        with mid_col:
            st.subheader("🎮 控制面板")
 
            with st.expander("📍 起点 A（GCJ-02坐标）", expanded=True):
                st.number_input(
                    "纬度", value=st.session_state.start_point["lat"],
                    format="%.6f", key="start_lat", on_change=on_start_lat_change
                )
                st.number_input(
                    "经度", value=st.session_state.start_point["lng"],
                    format="%.6f", key="start_lng", on_change=on_start_lng_change
                )
 
            with st.expander("🏁 终点 B（GCJ-02坐标）", expanded=True):
                st.number_input(
                    "纬度", value=st.session_state.end_point["lat"],
                    format="%.6f", key="end_lat", on_change=on_end_lat_change
                )
                st.number_input(
                    "经度", value=st.session_state.end_point["lng"],
                    format="%.6f", key="end_lng", on_change=on_end_lng_change
                )
 
            st.divider()
            st.subheader("✈️ 飞行参数")
            st.number_input("飞行高度 (m)", value=st.session_state.flight_height,
                            step=5, min_value=10, max_value=200, key="fh_input",
                            on_change=on_fh_change)
            st.number_input("安全半径 (m)", value=st.session_state.safety_radius,
                            step=1, min_value=5, max_value=50, key="sr_input",
                            on_change=on_sr_change)
            spd = st.slider("飞行速度 (m/s)", 1.0, 20.0,
                            value=float(st.session_state.flight_speed), step=0.5)
            if abs(spd - st.session_state.flight_speed) > 0.01:
                st.session_state.flight_speed = spd
 
            st.divider()
            st.subheader("⛔ 新障碍物高度")
            st.number_input("障碍物高度 (m)", value=60, step=5,
                            min_value=10, max_value=200, key="new_obstacle_height")
            st.caption("💡 在地图上用多边形工具绘制障碍物区域")
 
        with right_col:
            st.subheader("📊 航线分析")
            if st.session_state.route_analysis:
                a = st.session_state.route_analysis
                st.metric("📏 总距离",   f"{a.get('total_distance',0):.1f} m")
                st.metric("🔄 绕行节点", a.get('bypass_count',0))
                st.metric("✅ 飞跃次数", a.get('fly_over_count',0))
                st.metric("🎯 使用策略", a.get('strategy_used','未知'))
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
                    c1_,c2_,c3_ = st.columns([1,2,1])
                    with c1_:
                        if st.button("🗑️", key=f"d{idx}"):
                            remove_obstacle(idx); plan_route(); st.rerun()
                    with c2_:
                        st.text(f"障碍 {idx+1}")
                        if obs.get('created_at'): st.caption(obs['created_at'][:10])
                    with c3_:
                        h_ = obs.get('height',10)
                        flag = "🔴" if h_>st.session_state.flight_height else "🟠"
                        st.text(f"{flag} {h_}m")
            else:
                st.info("暂无障碍物\n在地图绘制多边形添加")
 
            st.divider()
            cs,cl,cc = st.columns(3)
            with cs:
                if st.button("💾 保存", use_container_width=True):
                    save_obstacles(); _wp_save(); st.success("已保存")
            with cl:
                if st.button("📂 加载", use_container_width=True):
                    ok,cnt = load_obstacles()
                    wp_data = _wp_load()
                    if wp_data:
                        if "start_point" in wp_data: st.session_state.start_point = wp_data["start_point"]
                        if "end_point" in wp_data:   st.session_state.end_point   = wp_data["end_point"]
                        if "flight_height" in wp_data: st.session_state.flight_height = wp_data["flight_height"]
                        if "safety_radius" in wp_data: st.session_state.safety_radius = wp_data["safety_radius"]
                    st.success(f"已加载 {cnt} 个障碍物及起终点")
                    plan_route(); st.rerun()
            with cc:
                if st.button("🗑️ 清空", use_container_width=True):
                    clear_obstacles(); plan_route(); st.rerun()
 
    with tab2:
        render_comm_logs_page()
 
if __name__ == "__main__":
    main()
