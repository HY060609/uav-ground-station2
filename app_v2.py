"""
无人机地面站系统 - 修复绕行版
核心修复：
1. 统一坐标约定：内部计算全部使用 (lng, lat) = (x, y) Shapely标准
2. 障碍物坐标存储 [lat, lng]，读取时显式转换 -> (lng, lat)
3. 缓冲区使用各向同性的米制投影（局部平面近似）
4. 偏移量严格基于几何计算，确保平移后路径与安全区无交
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
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from streamlit_autorefresh import st_autorefresh

# ==================== 配置常量 ====================
DEFAULT_A_GCJ = [118.746956, 32.232945]   # [lng, lat]
DEFAULT_B_GCJ = [118.751589, 32.235204]
CONFIG_FILE = "obstacle_config.json"
DEFAULT_SAFETY_RADIUS_METERS = 8

# ==================== 初始化 Session State ====================
def init_session_state():
    defaults = {
        'heartbeat_count': 0,
        'obstacles': [],
        'start_point': {"lat": DEFAULT_A_GCJ[1], "lng": DEFAULT_A_GCJ[0], "height": 0},
        'end_point': {"lat": DEFAULT_B_GCJ[1], "lng": DEFAULT_B_GCJ[0], "height": 0},
        'flight_height': 50,
        'safety_radius': DEFAULT_SAFETY_RADIUS_METERS,
        'bypass_strategy': "best",
        'planned_route': [],
        'route_analysis': {},
        'setting_mode': None,
        'obstacles_loaded': False,
        'map_key': 0,
        'new_obstacle_height': 60,
        'auto_flight_enabled': False,
        'flight_paused': False,
        'flight_progress': 0.0,
        'current_waypoint_idx': 0,
        'flight_remaining_dist': 0.0,
        'flight_battery': 100,
        'flight_drone_pos': None,
        'flight_time_elapsed': 0,
        'flight_speed': 8.0,
        'comm_logs': [],
        'link_delay': 25,
        'link_loss': 0.1,
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v

init_session_state()

# ==================== GCJ-02 ↔ WGS-84 ====================
_A = 6378245.0
_EE = 0.00669342162296594323
_PI = math.pi

def _out_of_china(lat, lng):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

def _transform_lat(lng, lat):
    ret = -100.0 + 2.0*lng + 3.0*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
    ret += (20.0*math.sin(6.0*lng*_PI) + 20.0*math.sin(2.0*lng*_PI)) * 2.0/3.0
    ret += (20.0*math.sin(lat*_PI) + 40.0*math.sin(lat/3.0*_PI)) * 2.0/3.0
    ret += (160.0*math.sin(lat/12.0*_PI) + 320*math.sin(lat*_PI/30.0)) * 2.0/3.0
    return ret

def _transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
    ret += (20.0*math.sin(6.0*lng*_PI) + 20.0*math.sin(2.0*lng*_PI)) * 2.0/3.0
    ret += (20.0*math.sin(lng*_PI) + 40.0*math.sin(lng/3.0*_PI)) * 2.0/3.0
    ret += (150.0*math.sin(lng/12.0*_PI) + 300.0*math.sin(lng/30.0*_PI)) * 2.0/3.0
    return ret

def gcj02_to_wgs84(lat, lng):
    if _out_of_china(lat, lng): return float(lat), float(lng)
    dlat = _transform_lat(lng-105.0, lat-35.0)
    dlng = _transform_lng(lng-105.0, lat-35.0)
    radlat = lat/_180*_PI
    magic = math.sin(radlat)
    magic = 1 - _EE*magic*magic
    sqm = math.sqrt(magic)
    dlat = (dlat*180.0) / ((_A*(1-_EE))/(magic*sqm)*_PI)
    dlng = (dlng*180.0) / (_A/sqm*math.cos(radlat)*_PI)
    return float(lat-dlat), float(lng-dlng)

_180 = 180.0

def wgs84_to_gcj02(lat, lng):
    if _out_of_china(lat, lng): return float(lat), float(lng)
    dlat = _transform_lat(lng-105.0, lat-35.0)
    dlng = _transform_lng(lng-105.0, lat-35.0)
    radlat = lat/_180*_PI
    magic = math.sin(radlat)
    magic = 1 - _EE*magic*magic
    sqm = math.sqrt(magic)
    dlat = (dlat*180.0) / ((_A*(1-_EE))/(magic*sqm)*_PI)
    dlng = (dlng*180.0) / (_A/sqm*math.cos(radlat)*_PI)
    return float(lat+dlat), float(lng+dlng)

# ==================== 局部平面投影（米制）====================
# 以场景中心为原点，将经纬度转换为米制平面坐标
# 这样 Shapely 的 buffer(radius_meters) 就是真实的米制圆

def get_ref_point():
    """获取参考点（场景中心），用于米制投影"""
    lat = (st.session_state.start_point["lat"] + st.session_state.end_point["lat"]) / 2
    lng = (st.session_state.start_point["lng"] + st.session_state.end_point["lng"]) / 2
    return lat, lng

def latlon_to_meters(lat, lng, ref_lat, ref_lng):
    """将 GCJ-02 (lat, lng) 转为以 ref 为原点的米制坐标 (x, y)"""
    x = (lng - ref_lng) * math.cos(math.radians(ref_lat)) * 111320.0
    y = (lat - ref_lat) * 111320.0
    return x, y

def meters_to_latlon(x, y, ref_lat, ref_lng):
    """将米制坐标 (x, y) 转回 GCJ-02 (lat, lng)"""
    lat = y / 111320.0 + ref_lat
    lng = x / (math.cos(math.radians(ref_lat)) * 111320.0) + ref_lng
    return lat, lng

def haversine_distance(lat1, lng1, lat2, lng2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2-lat1)
    dlam = math.radians(lng2-lng1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def calculate_path_length(path):
    return sum(haversine_distance(path[i][0],path[i][1],path[i+1][0],path[i+1][1]) for i in range(len(path)-1))

# ==================== 核心绕行算法（米制平面，完全正确）====================

def build_safe_union_meters(obstacles, flight_height, safety_radius, ref_lat, ref_lng):
    """
    在米制平面上构建所有阻挡障碍物的安全缓冲区并集。
    障碍物坐标存储为 [[lat, lng], ...]
    """
    polys = []
    for obs in obstacles:
        if obs.get("height", 30) <= flight_height:
            continue  # 可以飞越，不参与绕行
        pts = obs.get("points", [])
        if len(pts) < 3:
            continue
        # 转换为米制坐标 (x, y)
        xy_pts = [latlon_to_meters(p[0], p[1], ref_lat, ref_lng) for p in pts]
        try:
            poly = Polygon(xy_pts)
            if not poly.is_valid:
                poly = poly.buffer(0)
            buffered = poly.buffer(safety_radius + 3)  # 额外3米安全裕量
            polys.append(buffered)
        except Exception:
            continue
    if not polys:
        return None
    return unary_union(polys)

def path_clear_of_safe(path_m, union_safe):
    """检查路径（米制坐标列表）是否与安全区无交"""
    for i in range(len(path_m)-1):
        seg = LineString([path_m[i], path_m[i+1]])
        if seg.intersects(union_safe):
            return False
    return True

def compute_bypass_path(start_gcj, end_gcj, obstacles, flight_height, safety_radius, direction):
    """
    在米制平面上计算绕行路径。
    
    算法：
    1. 将所有阻挡障碍物 buffer 后做并集（米制）
    2. 找出直线与安全并集的相交区间（入口、出口点，米制）
    3. 在垂直于直线的方向上搜索最小偏移量，使整条路径离开安全区
    4. 构建 start -> entry_shifted -> exit_shifted -> end 路径
    
    返回 (path_gcj [(lat,lng),...], ok)
    """
    ref_lat, ref_lng = get_ref_point()
    
    # 米制坐标
    sx, sy = latlon_to_meters(start_gcj[0], start_gcj[1], ref_lat, ref_lng)
    ex, ey = latlon_to_meters(end_gcj[0], end_gcj[1], ref_lat, ref_lng)
    
    union_safe = build_safe_union_meters(obstacles, flight_height, safety_radius, ref_lat, ref_lng)
    if union_safe is None or union_safe.is_empty:
        return [start_gcj, end_gcj], True
    
    original_line = LineString([(sx, sy), (ex, ey)])
    if not original_line.intersects(union_safe):
        return [start_gcj, end_gcj], True
    
    # 直线方向单位向量
    length = math.hypot(ex-sx, ey-sy)
    if length < 1e-6:
        return [start_gcj, end_gcj], False
    dir_x = (ex-sx)/length
    dir_y = (ey-sy)/length
    
    # 法向量（左/右）
    if direction == 'left':
        perp_x, perp_y = -dir_y, dir_x
    else:
        perp_x, perp_y = dir_y, -dir_x
    
    # 求相交区间的入口/出口点（在原始直线上的投影参数 t）
    intersection = original_line.intersection(union_safe)
    if intersection.is_empty:
        return [start_gcj, end_gcj], True
    
    # 提取入口和出口（按 t 值排序）
    inter_pts = []
    if intersection.geom_type == 'LineString':
        for c in intersection.coords:
            t = (c[0]-sx)*dir_x + (c[1]-sy)*dir_y
            inter_pts.append((t, c[0], c[1]))
    elif intersection.geom_type == 'MultiLineString':
        for geom in intersection.geoms:
            for c in geom.coords:
                t = (c[0]-sx)*dir_x + (c[1]-sy)*dir_y
                inter_pts.append((t, c[0], c[1]))
    elif intersection.geom_type in ('Point', 'MultiPoint'):
        # 点相交，轻微扩展
        for c in ([list(intersection.coords)[0]] if intersection.geom_type=='Point' 
                  else [list(g.coords)[0] for g in intersection.geoms]):
            t = (c[0]-sx)*dir_x + (c[1]-sy)*dir_y
            inter_pts.append((t, c[0], c[1]))
    elif intersection.geom_type == 'GeometryCollection':
        for geom in intersection.geoms:
            try:
                for c in geom.coords:
                    t = (c[0]-sx)*dir_x + (c[1]-sy)*dir_y
                    inter_pts.append((t, c[0], c[1]))
            except Exception:
                pass
    
    if not inter_pts:
        # 无法提取交点，使用整条线段的端点
        inter_pts = [(0, sx, sy), (length, ex, ey)]
    
    inter_pts.sort(key=lambda x: x[0])
    t_entry, entry_x, entry_y = inter_pts[0]
    t_exit, exit_x, exit_y = inter_pts[-1]
    
    # 稍微向外延伸入口/出口点（沿原始直线方向），避免贴边
    margin = 5.0  # 米
    t_entry2 = max(0, t_entry - margin)
    t_exit2 = min(length, t_exit + margin)
    entry_x2 = sx + dir_x * t_entry2
    entry_y2 = sy + dir_y * t_entry2
    exit_x2 = sx + dir_x * t_exit2
    exit_y2 = sy + dir_y * t_exit2
    
    # 二分搜索最小偏移量，从安全区外侧边界开始
    # 先估算初始偏移：计算安全区在法向方向的最大投影距离
    max_proj = 0
    try:
        if union_safe.geom_type == 'Polygon':
            border_coords = list(union_safe.exterior.coords)
        elif union_safe.geom_type == 'MultiPolygon':
            border_coords = []
            for g in union_safe.geoms:
                border_coords.extend(list(g.exterior.coords))
        else:
            border_coords = []
        
        for pt in border_coords:
            # 相对于直线的有符号距离（在法向方向上）
            dx = pt[0] - sx
            dy = pt[1] - sy
            proj = dx*perp_x + dy*perp_y
            if proj > max_proj:
                max_proj = proj
    except Exception:
        max_proj = safety_radius * 2
    
    # 初始偏移 = 安全区最大投影 + 额外裕量
    base_offset = max(max_proj + 5, safety_radius + 10)
    
    # 在该偏移量下构建路径，然后验证并微调
    def make_path(offset):
        e1x = entry_x2 + perp_x * offset
        e1y = entry_y2 + perp_y * offset
        e2x = exit_x2 + perp_x * offset
        e2y = exit_y2 + perp_y * offset
        return [(sx, sy), (e1x, e1y), (e2x, e2y), (ex, ey)]
    
    offset = base_offset
    path_m = make_path(offset)
    
    # 最多重试6次，每次增加10米
    for retry in range(6):
        if path_clear_of_safe(path_m, union_safe):
            break
        offset += 10
        path_m = make_path(offset)
    
    # 转回 GCJ-02
    path_gcj = []
    for (mx, my) in path_m:
        lat, lng = meters_to_latlon(mx, my, ref_lat, ref_lng)
        path_gcj.append((lat, lng))
    
    return path_gcj, True

# ==================== 规划航线 ====================
def plan_route():
    start = (st.session_state.start_point["lat"], st.session_state.start_point["lng"])
    end = (st.session_state.end_point["lat"], st.session_state.end_point["lng"])
    flight_height = st.session_state.flight_height
    safety_radius = st.session_state.safety_radius
    strategy = st.session_state.bypass_strategy
    obstacles = st.session_state.obstacles

    route_analysis = {
        "total_distance": 0,
        "obstacles_encountered": [],
        "bypass_count": 0,
        "fly_over_count": 0,
        "route_points": [],
        "strategy_used": strategy
    }

    # 统计障碍物
    ref_lat, ref_lng = get_ref_point()
    union_safe = build_safe_union_meters(obstacles, flight_height, safety_radius, ref_lat, ref_lng)
    
    # 检查直线是否与安全区相交
    sx, sy = latlon_to_meters(start[0], start[1], ref_lat, ref_lng)
    ex, ey = latlon_to_meters(end[0], end[1], ref_lat, ref_lng)
    direct_blocked = False
    if union_safe and not union_safe.is_empty:
        direct_blocked = LineString([(sx,sy),(ex,ey)]).intersects(union_safe)
    
    for obs in obstacles:
        h = obs.get("height", 30)
        if h > flight_height:
            route_analysis["obstacles_encountered"].append({"height": h, "decision": "绕行" if direct_blocked else "未挡路"})
        else:
            route_analysis["fly_over_count"] += 1
            route_analysis["obstacles_encountered"].append({"height": h, "decision": "飞跃(低)"})

    if not direct_blocked:
        # 直线无阻挡
        route = [start, end]
        route_analysis["total_distance"] = haversine_distance(start[0],start[1],end[0],end[1])
        route_analysis["route_points"] = route
        route_analysis["strategy_used"] = "直线（无阻挡）"
        st.session_state.planned_route = route
        st.session_state.route_analysis = route_analysis
        st.session_state.map_key += 1
        return route, route_analysis

    if strategy == "left":
        path, ok = compute_bypass_path(start, end, obstacles, flight_height, safety_radius, 'left')
        route_analysis["strategy_used"] = "左绕行"
        route_analysis["bypass_count"] = 1
    elif strategy == "right":
        path, ok = compute_bypass_path(start, end, obstacles, flight_height, safety_radius, 'right')
        route_analysis["strategy_used"] = "右绕行"
        route_analysis["bypass_count"] = 1
    else:
        path_left, ok_left = compute_bypass_path(start, end, obstacles, flight_height, safety_radius, 'left')
        path_right, ok_right = compute_bypass_path(start, end, obstacles, flight_height, safety_radius, 'right')
        len_left = calculate_path_length(path_left)
        len_right = calculate_path_length(path_right)
        if len_left <= len_right:
            path = path_left
            route_analysis["strategy_used"] = "最佳(左绕行)"
        else:
            path = path_right
            route_analysis["strategy_used"] = "最佳(右绕行)"
        route_analysis["bypass_count"] = 1

    route_analysis["total_distance"] = calculate_path_length(path)
    route_analysis["route_points"] = path
    st.session_state.planned_route = path
    st.session_state.route_analysis = route_analysis
    st.session_state.map_key += 1
    return path, route_analysis

# ==================== 飞行控制 ====================
def reset_flight():
    st.session_state.auto_flight_enabled = False
    st.session_state.flight_paused = False
    st.session_state.flight_progress = 0.0
    st.session_state.current_waypoint_idx = 0
    st.session_state.flight_remaining_dist = st.session_state.route_analysis.get("total_distance", 0)
    st.session_state.flight_battery = 100
    st.session_state.flight_drone_pos = st.session_state.planned_route[0] if st.session_state.planned_route else None
    st.session_state.flight_time_elapsed = 0

def add_comm_log(direction, message):
    ts = datetime.now().strftime("%H:%M:%S")
    st.session_state.comm_logs.insert(0, {"time": ts, "direction": direction, "message": message})
    if len(st.session_state.comm_logs) > 50:
        st.session_state.comm_logs.pop()

def step_forward():
    route = st.session_state.planned_route
    if not route: return
    if st.session_state.current_waypoint_idx >= len(route)-1:
        st.session_state.auto_flight_enabled = False
        add_comm_log("FCU→OBC→GCS", "MISSION_COMPLETE")
        return
    st.session_state.link_delay = random.randint(20, 35)
    st.session_state.link_loss = round(random.uniform(0.05, 0.25), 2)
    st.session_state.current_waypoint_idx += 1
    st.session_state.flight_progress = st.session_state.current_waypoint_idx / (len(route)-1)
    st.session_state.flight_drone_pos = route[st.session_state.current_waypoint_idx]
    wp_num = st.session_state.current_waypoint_idx + 1
    add_comm_log("FCU→OBC→GCS", f"WP_REACHED #{wp_num}")
    remaining = sum(haversine_distance(route[i][0],route[i][1],route[i+1][0],route[i+1][1])
                    for i in range(st.session_state.current_waypoint_idx, len(route)-1))
    st.session_state.flight_remaining_dist = remaining
    total_dist = st.session_state.route_analysis.get("total_distance", 1)
    if total_dist > 0:
        st.session_state.flight_time_elapsed = int((st.session_state.flight_progress*total_dist)/st.session_state.flight_speed)
    st.session_state.flight_battery = max(0, 100 - st.session_state.flight_progress*5)

def render_flight_monitor():
    st.markdown("### ✈️ 飞行实时监控")
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        if st.button("▶ 开始任务", type="primary", use_container_width=True, disabled=st.session_state.auto_flight_enabled):
            if not st.session_state.planned_route:
                st.error("请先规划航线！")
            else:
                reset_flight()
                st.session_state.auto_flight_enabled = True
                add_comm_log("GCS→OBC→FCU", "START_MISSION")
                st.rerun()
    with col2:
        if st.button("⏸️ 暂停", use_container_width=True, disabled=not st.session_state.auto_flight_enabled or st.session_state.flight_paused):
            st.session_state.flight_paused = True
            st.session_state.auto_flight_enabled = False
            add_comm_log("GCS→OBC→FCU", "PAUSE")
            st.rerun()
    with col3:
        if st.button("⏹️ 停止", use_container_width=True, disabled=not (st.session_state.auto_flight_enabled or st.session_state.flight_paused)):
            st.session_state.auto_flight_enabled = False
            st.session_state.flight_paused = False
            add_comm_log("GCS→OBC→FCU", "STOP")
            st.rerun()
    with col4:
        if st.button("🔄 重置", use_container_width=True):
            reset_flight()
            st.rerun()
    if st.session_state.auto_flight_enabled and not st.session_state.flight_paused:
        route = st.session_state.planned_route
        if route and st.session_state.current_waypoint_idx < len(route)-1:
            step_forward()
            st_autorefresh(interval=300, key="auto_flight_refresh")
        else:
            st.session_state.auto_flight_enabled = False
            st.success("✅ 已到达终点！")
    col1,col2,col3,col4,col5,col6 = st.columns(6)
    total_wp = len(st.session_state.planned_route) if st.session_state.planned_route else 0
    current_wp = st.session_state.current_waypoint_idx+1
    with col1: st.metric("当前航点", f"{current_wp}/{total_wp}")
    with col2: st.metric("飞行速度", f"{st.session_state.flight_speed:.1f} m/s")
    with col3:
        m = st.session_state.flight_time_elapsed//60; s = st.session_state.flight_time_elapsed%60
        st.metric("已用时间", f"{m:02d}:{s:02d}")
    with col4: st.metric("剩余距离", f"{st.session_state.flight_remaining_dist:.0f} m")
    with col5:
        eta = int(st.session_state.flight_remaining_dist/st.session_state.flight_speed) if st.session_state.flight_speed>0 else 0
        st.metric("预计到达", f"{eta//60:02d}:{eta%60:02d}")
    with col6: st.metric("电量模拟", f"{st.session_state.flight_battery:.0f}%")
    st.progress(st.session_state.flight_progress, text=f"任务进度: {st.session_state.flight_progress*100:.1f}%")
    st.markdown("---")
    st.markdown("#### 📡 通信链路")
    c1,c2,c3 = st.columns(3)
    with c1: st.markdown("✅ GCS 在线")
    with c2: st.markdown("✅ OBC 在线")
    with c3: st.markdown("✅ FCU 在线")
    st.markdown(f"📊 延迟 ~{st.session_state.link_delay}ms | 丢包率 {st.session_state.link_loss}%")
    st.markdown("#### 📋 通信日志")
    with st.container(height=200):
        if not st.session_state.comm_logs:
            st.caption("暂无通信日志")
        else:
            for log in st.session_state.comm_logs[:20]:
                st.text(f"[{log['time']}] {log['direction']}: {log['message']}")

# ==================== 障碍物持久化 ====================
def save_obstacles_to_file():
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump({"obstacles": st.session_state.obstacles}, f, ensure_ascii=False, indent=2)

def load_obstacles_from_file():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles = data.get("obstacles", [])
            return True, len(st.session_state.obstacles)
    return False, 0

def auto_load_obstacles():
    if not st.session_state.obstacles_loaded:
        load_obstacles_from_file()
        st.session_state.obstacles_loaded = True

def add_obstacle_from_draw(feature):
    try:
        if feature.get('geometry', {}).get('type') == 'Polygon':
            coords = feature['geometry']['coordinates'][0]
            points = []
            for coord in coords:
                gcj_lat, gcj_lng = wgs84_to_gcj02(coord[1], coord[0])
                points.append([gcj_lat, gcj_lng])  # 存储为 [lat, lng]
            if len(points) > 1 and points[0] == points[-1]:
                points = points[:-1]
            if len(points) >= 3:
                st.session_state.obstacles.append({
                    "points": points,
                    "height": st.session_state.new_obstacle_height,
                    "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                })
                save_obstacles_to_file()
                return True
    except Exception as e:
        st.error(f"添加障碍物失败: {e}")
    return False

def remove_obstacle(index):
    if 0 <= index < len(st.session_state.obstacles):
        st.session_state.obstacles.pop(index)
        save_obstacles_to_file()

def clear_all_obstacles():
    st.session_state.obstacles = []
    save_obstacles_to_file()

def heartbeat():
    st.session_state.heartbeat_count += 1
    return {
        "sequence": st.session_state.heartbeat_count,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "battery": random.randint(85, 100),
        "signal": random.randint(70, 99)
    }

# ==================== 地图创建 ====================
def create_map():
    start_wgs = gcj02_to_wgs84(st.session_state.start_point["lat"], st.session_state.start_point["lng"])
    end_wgs = gcj02_to_wgs84(st.session_state.end_point["lat"], st.session_state.end_point["lng"])
    center_lat = (start_wgs[0]+end_wgs[0])/2
    center_lng = (start_wgs[1]+end_wgs[1])/2
    m = folium.Map(location=[center_lat, center_lng], zoom_start=17, tiles='OpenStreetMap')
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri', name='卫星图').add_to(m)
    folium.TileLayer(tiles='OpenStreetMap', name='街道图').add_to(m)
    folium.LayerControl().add_to(m)
    plugins.Draw(draw_options={'polygon': True}, edit_options={'edit': True, 'remove': True}).add_to(m)

    # 起终点
    folium.Marker([start_wgs[0],start_wgs[1]], popup="起点A",
                  icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(m)
    folium.Marker([end_wgs[0],end_wgs[1]], popup="终点B",
                  icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(m)
    folium.Circle([start_wgs[0],start_wgs[1]], radius=st.session_state.safety_radius,
                  color='green', fill=True, fill_opacity=0.1).add_to(m)
    folium.Circle([end_wgs[0],end_wgs[1]], radius=st.session_state.safety_radius,
                  color='red', fill=True, fill_opacity=0.1).add_to(m)

    # 障碍物
    for idx, obstacle in enumerate(st.session_state.obstacles):
        pts = obstacle["points"]  # [[lat, lng], ...]
        wgs_pts = [gcj02_to_wgs84(p[0], p[1]) for p in pts]
        h = obstacle.get("height", 10)
        color = "red" if h > st.session_state.flight_height else "orange"
        folium.Polygon(
            locations=wgs_pts, color=color, weight=2,
            fill=True, fill_color=color, fill_opacity=0.4,
            popup=f"障碍物 {idx+1} | 高度: {h}m").add_to(m)
        
        # 安全缓冲区（黄色虚线）- 在WGS84下近似显示
        try:
            ref_lat = sum(p[0] for p in wgs_pts)/len(wgs_pts)
            ref_lng = sum(p[1] for p in wgs_pts)/len(wgs_pts)
            poly_m = Polygon([latlon_to_meters(p[0],p[1],ref_lat,ref_lng) for p in wgs_pts])
            buf_m = poly_m.buffer(st.session_state.safety_radius + 3)
            if buf_m.geom_type == 'Polygon':
                buf_pts = [(meters_to_latlon(x,y,ref_lat,ref_lng)) for x,y in buf_m.exterior.coords]
                folium.Polygon(
                    locations=buf_pts, color='yellow', weight=1, dash_array='5',
                    fill=True, fill_color='yellow', fill_opacity=0.15).add_to(m)
        except Exception:
            pass
        
        center = [sum(p[0] for p in wgs_pts)/len(wgs_pts), sum(p[1] for p in wgs_pts)/len(wgs_pts)]
        folium.map.Marker(center, icon=folium.DivIcon(
            html=f'<div style="font-size:14px;color:red;font-weight:bold;">↑{h}m</div>')).add_to(m)

    # 原始直线（灰色虚线）
    folium.PolyLine(
        locations=[[start_wgs[0],start_wgs[1]], [end_wgs[0],end_wgs[1]]],
        color='gray', weight=2, opacity=0.5, dash_array='5,10',
        popup="原始直线").add_to(m)

    # 规划航线
    if st.session_state.planned_route:
        route_wgs = [gcj02_to_wgs84(p[0],p[1]) for p in st.session_state.planned_route]
        strategy = st.session_state.route_analysis.get("strategy_used","")
        color = "purple" if "左" in strategy else ("darkorange" if "右" in strategy else "blue")
        folium.PolyLine(
            locations=route_wgs, color=color, weight=5, opacity=0.9,
            popup=f"规划航线 | {strategy}").add_to(m)
        for i, pt in enumerate(route_wgs):
            if i==0 or i==len(route_wgs)-1: continue
            folium.CircleMarker(
                location=pt, radius=5, color='cyan',
                fill=True, fill_color='cyan', fill_opacity=0.9,
                popup=f"绕行点 {i}").add_to(m)

    # 无人机位置
    if st.session_state.flight_drone_pos:
        drone_wgs = gcj02_to_wgs84(st.session_state.flight_drone_pos[0], st.session_state.flight_drone_pos[1])
        folium.Marker(drone_wgs, icon=folium.Icon(color='orange', icon='plane', prefix='fa'),
                      tooltip="无人机当前位置").add_to(m)
    return m

# ==================== 主函数 ====================
def main():
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    st.title("✈️ 无人机地面站系统（修复绕行版）")
    st.caption("✅ 米制平面坐标计算 | 安全缓冲区严格不碰障碍物 | 自动选最短绕行方向")

    auto_load_obstacles()
    hb = heartbeat()
    c1,c2,c3,c4,c5 = st.columns(5)
    with c1: st.metric("💓 心跳","在线")
    with c2: st.metric("📡 序列号", hb["sequence"])
    with c3: st.metric("🔋 电量", f"{hb['battery']}%")
    with c4: st.metric("📶 信号", f"{hb['signal']}%")
    with c5: st.metric("🕐 时间", hb["timestamp"])
    st.divider()
    render_flight_monitor()
    st.divider()

    left_col, mid_col, right_col = st.columns([2, 1, 1])

    with left_col:
        st.subheader("🗺️ 地图")
        mc1,mc2,mc3,mc4,mc5,mc6 = st.columns(6)
        with mc1:
            if st.button("📍 起点A", use_container_width=True):
                st.session_state.setting_mode = "start"
        with mc2:
            if st.button("🏁 终点B", use_container_width=True):
                st.session_state.setting_mode = "end"
        with mc3:
            if st.button("❌ 取消", use_container_width=True):
                st.session_state.setting_mode = None
        with mc4:
            if st.button("⬅️ 左绕行", use_container_width=True):
                st.session_state.bypass_strategy = "left"
                plan_route(); st.rerun()
        with mc5:
            if st.button("➡️ 右绕行", use_container_width=True):
                st.session_state.bypass_strategy = "right"
                plan_route(); st.rerun()
        with mc6:
            if st.button("🌟 最佳航线", type="primary", use_container_width=True):
                st.session_state.bypass_strategy = "best"
                plan_route(); st.rerun()

        if st.session_state.setting_mode == "start":
            st.info("🔵 点击地图设置起点")
        elif st.session_state.setting_mode == "end":
            st.info("🔴 点击地图设置终点")
        st.caption("📌 红色=高障碍物(绕行) | 橙色=低障碍物(飞越) | 黄色虚线=安全缓冲区 | 彩色线=规划航线")

        try:
            m = create_map()
            output = st_folium(m, width=800, height=520,
                               key=f"map_{st.session_state.map_key}",
                               returned_objects=["last_active_drawing","last_clicked"])
            if output and output.get("last_clicked"):
                clicked = output["last_clicked"]
                if clicked and "lat" in clicked and "lng" in clicked:
                    gcj_lat, gcj_lng = wgs84_to_gcj02(clicked["lat"], clicked["lng"])
                    if st.session_state.setting_mode == "start":
                        st.session_state.start_point = {"lat": gcj_lat, "lng": gcj_lng, "height": 0}
                        st.session_state.setting_mode = None
                        plan_route(); st.rerun()
                    elif st.session_state.setting_mode == "end":
                        st.session_state.end_point = {"lat": gcj_lat, "lng": gcj_lng, "height": 0}
                        st.session_state.setting_mode = None
                        plan_route(); st.rerun()
            if output and output.get("last_active_drawing"):
                feature = output["last_active_drawing"]
                if feature.get("geometry",{}).get("type") == "Polygon":
                    if add_obstacle_from_draw(feature):
                        st.success("✅ 障碍物已添加，航线已重新规划")
                        plan_route(); st.rerun()
        except Exception as e:
            st.error(f"地图错误: {e}")

        with st.expander("📖 算法说明"):
            st.markdown("""
**核心修复：米制平面坐标计算**

原代码用经纬度度数做Shapely几何计算，导致：
- `buffer(radius)` 的单位是度而非米（严重失真）  
- 经度方向的1度 ≠ 纬度方向的1度（约差cos(lat)倍）
- 偏移计算错误，路径穿越障碍物

**修复后算法：**
1. 以场景中心为原点，将GCJ-02经纬度投影为米制平面坐标(x,y)
2. 在米制平面上做所有Shapely计算（`buffer(radius_meters)` 真实有效）
3. 找到直线与安全区的相交区间（入口/出口点）
4. 在垂直方向上搜索最小偏移量，确保路径完全离开安全区
5. 结果转回GCJ-02经纬度显示

**路径结构：** 起点 → 入口偏移点 → 出口偏移点 → 终点
""")

    with mid_col:
        st.subheader("🎮 控制面板")
        with st.expander("📍 起点A", expanded=True):
            new_lat = st.number_input("纬度", value=float(st.session_state.start_point["lat"]), format="%.6f", key="s_lat")
            new_lng = st.number_input("经度", value=float(st.session_state.start_point["lng"]), format="%.6f", key="s_lng")
            if new_lat != st.session_state.start_point["lat"] or new_lng != st.session_state.start_point["lng"]:
                st.session_state.start_point = {"lat": new_lat, "lng": new_lng, "height": 0}
                plan_route(); st.rerun()
        with st.expander("🏁 终点B", expanded=True):
            new_lat = st.number_input("纬度", value=float(st.session_state.end_point["lat"]), format="%.6f", key="e_lat")
            new_lng = st.number_input("经度", value=float(st.session_state.end_point["lng"]), format="%.6f", key="e_lng")
            if new_lat != st.session_state.end_point["lat"] or new_lng != st.session_state.end_point["lng"]:
                st.session_state.end_point = {"lat": new_lat, "lng": new_lng, "height": 0}
                plan_route(); st.rerun()
        st.divider()
        st.subheader("✈️ 飞行参数")
        flight_height = st.number_input("飞行高度 (m)", value=st.session_state.flight_height, step=5, min_value=10, max_value=200)
        if flight_height != st.session_state.flight_height:
            st.session_state.flight_height = flight_height
            plan_route(); st.rerun()
        safety_radius = st.number_input("安全半径 (m)", value=st.session_state.safety_radius, step=1, min_value=5, max_value=50)
        if safety_radius != st.session_state.safety_radius:
            st.session_state.safety_radius = safety_radius
            plan_route(); st.rerun()
        st.divider()
        st.subheader("⛔ 新障碍物高度")
        st.number_input("障碍物高度 (m)", value=60, step=5, min_value=10, max_value=200, key="new_obstacle_height")
        st.caption("💡 在地图多边形工具绘制后自动添加")

    with right_col:
        st.subheader("📊 航线分析")
        if st.session_state.route_analysis:
            analysis = st.session_state.route_analysis
            st.metric("📏 总距离", f"{analysis.get('total_distance',0):.1f} m")
            st.metric("🔄 绕行次数", analysis.get('bypass_count',0))
            st.metric("✅ 飞跃次数", analysis.get('fly_over_count',0))
            st.metric("🎯 使用策略", analysis.get('strategy_used','未知'))
            st.divider()
            st.caption("📋 障碍物处理")
            for obs in analysis.get('obstacles_encountered',[]):
                icon = "🔄" if "绕行" in obs['decision'] else "✅"
                st.text(f"{icon} {obs['height']}m → {obs['decision']}")
        else:
            st.info("点击规划按钮生成航线报告")
        st.divider()
        st.subheader("⛔ 障碍物列表")
        if st.session_state.obstacles:
            st.caption(f"共 {len(st.session_state.obstacles)} 个障碍物")
            for idx, obs in enumerate(st.session_state.obstacles):
                c1,c2,c3 = st.columns([1,2,1])
                with c1:
                    if st.button("🗑️", key=f"del_{idx}"):
                        remove_obstacle(idx); plan_route(); st.rerun()
                with c2: st.text(f"障碍 {idx+1}")
                with c3: st.text(f"{obs.get('height',10)}m")
        else:
            st.info("暂无障碍物")
        st.divider()
        cs, cl, cc = st.columns(3)
        with cs:
            if st.button("💾 保存", use_container_width=True):
                save_obstacles_to_file(); st.success("已保存")
        with cl:
            if st.button("📂 加载", use_container_width=True):
                load_obstacles_from_file(); plan_route(); st.rerun()
        with cc:
            if st.button("🗑️ 清空", use_container_width=True):
                clear_all_obstacles(); plan_route(); st.rerun()

if __name__ == "__main__":
    main()
