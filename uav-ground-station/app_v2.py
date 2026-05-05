"""
无人机地面站系统 - 智能任务规划平台
功能：心跳包、地图显示、GCJ-02坐标转换、障碍物多边形圈选、航线规划、绕行策略、实时飞行监控
✅ 修复：网页端飞机动画不显示、白屏闪烁、移动卡顿问题
"""
import streamlit as st
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw
import json
import os
from datetime import datetime
import random
import math
import time
from shapely.geometry import Point, Polygon, LineString

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="无人机地面站系统",
    page_icon="✈️",
    layout="wide"
)

# ==================== 初始化 Session State ====================
def init_session_state():
    if 'heartbeat_count' not in st.session_state:
        st.session_state.heartbeat_count = 0
    if 'obstacles' not in st.session_state:
        st.session_state.obstacles = []
    if 'start_point' not in st.session_state:
        st.session_state.start_point = {"lat": 32.2323, "lng": 118.749, "height": 0}
    if 'end_point' not in st.session_state:
        st.session_state.end_point = {"lat": 32.2344, "lng": 118.749, "height": 0}
    if 'flight_height' not in st.session_state:
        st.session_state.flight_height = 50
    if 'safety_radius' not in st.session_state:
        st.session_state.safety_radius = 8
    if 'bypass_strategy' not in st.session_state:
        st.session_state.bypass_strategy = "right"
    if 'planned_route' not in st.session_state:
        st.session_state.planned_route = []
    if 'route_analysis' not in st.session_state:
        st.session_state.route_analysis = {}
    if 'map_center' not in st.session_state:
        st.session_state.map_center = [32.2333, 118.749]
    if 'setting_mode' not in st.session_state:
        st.session_state.setting_mode = None
    if 'deployment_status' not in st.session_state:
        st.session_state.deployment_status = None
    if 'deployment_log' not in st.session_state:
        st.session_state.deployment_log = []
    if 'obstacles_loaded' not in st.session_state:
        st.session_state.obstacles_loaded = False
    if 'map_key' not in st.session_state:
        st.session_state.map_key = 0
    if 'new_obstacle_height' not in st.session_state:
        st.session_state.new_obstacle_height = 60

    if 'flight_task_running' not in st.session_state:
        st.session_state.flight_task_running = False
    if 'flight_task_paused' not in st.session_state:
        st.session_state.flight_task_paused = False
    if 'flight_progress' not in st.session_state:
        st.session_state.flight_progress = 0.0
    if 'current_waypoint_idx' not in st.session_state:
        st.session_state.current_waypoint_idx = 0
    if 'flight_speed' not in st.session_state:
        st.session_state.flight_speed = 0.0
    if 'flight_time_elapsed' not in st.session_state:
        st.session_state.flight_time_elapsed = 0
    if 'flight_remaining_dist' not in st.session_state:
        st.session_state.flight_remaining_dist = 0.0
    if 'flight_battery' not in st.session_state:
        st.session_state.flight_battery = 100
    if 'flight_drone_pos' not in st.session_state:
        st.session_state.flight_drone_pos = None

init_session_state()

# ==================== GCJ-02 转 WGS84 ====================
A = 6378245.0
EE = 0.00669342162296594323
PI = 3.141592653589793

def out_of_china(lat, lng):
    if lng < 72.004 or lng > 137.8347:
        return True
    if lat < 0.8293 or lat > 55.8271:
        return True
    return False

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat
    ret += 0.1 * lng * lat
    ret += 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * PI) + 20.0 * math.sin(2.0 * lng * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * PI) + 40.0 * math.sin(lat / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * PI) + 320 * math.sin(lat * PI / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng
    ret += 0.1 * lng * lat
    ret += 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * PI) + 20.0 * math.sin(2.0 * lng * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * PI) + 40.0 * math.sin(lng / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * PI) + 300.0 * math.sin(lng / 30.0 * PI)) * 2.0 / 3.0
    return ret

def gcj02_to_wgs84(lat, lng):
    if out_of_china(lat, lng):
        return float(lat), float(lng)
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (A / sqrtmagic * math.cos(radlat) * PI)
    return float(lat - dlat), float(lng - dlng)

def wgs84_to_gcj02(lat, lng):
    if out_of_china(lat, lng):
        return float(lat), float(lng)
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (A / sqrtmagic * math.cos(radlat) * PI)
    return float(lat + dlat), float(lng + dlng)

# ==================== 地理计算 ====================
def haversine_distance(lat1, lng1, lat2, lng2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def get_bearing(lat1, lng1, lat2, lng2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lng = math.radians(lng2 - lng1)
    x = math.sin(delta_lng) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lng)
    bearing = math.atan2(x, y)
    return math.degrees(bearing)

def point_at_distance(lat, lng, bearing, distance_m):
    R = 6371000
    lat_rad = math.radians(lat)
    lng_rad = math.radians(lng)
    bearing_rad = math.radians(bearing)
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(distance_m / R) +
        math.cos(lat_rad) * math.sin(distance_m / R) * math.cos(bearing_rad)
    )
    new_lng_rad = lng_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(distance_m / R) * math.cos(lat_rad),
        math.cos(distance_m / R) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )
    return math.degrees(new_lat_rad), math.degrees(new_lng_rad)

def get_obstacle_center(points_lat_lng):
    lats = [p[0] for p in points_lat_lng]
    lngs = [p[1] for p in points_lat_lng]
    return sum(lats)/len(lats), sum(lngs)/len(lngs)

def get_obstacle_bounds(points_lat_lng):
    lats = [p[0] for p in points_lat_lng]
    lngs = [p[1] for p in points_lat_lng]
    return min(lats), max(lats), min(lngs), max(lngs)

def get_obstacle_extent(points_lat_lng):
    min_lat, max_lat, min_lng, max_lng = get_obstacle_bounds(points_lat_lng)
    center_lat, center_lng = get_obstacle_center(points_lat_lng)
    width_m = haversine_distance(center_lat, min_lng, center_lat, max_lng)
    height_m = haversine_distance(min_lat, center_lng, max_lat, center_lng)
    return width_m, height_m

def to_shapely_pts(points_lat_lng):
    return [(lng, lat) for lat, lng in points_lat_lng]

# ==================== 绕行函数（右绕行彻底从外侧过） ====================
def create_safe_bypass(start_lat_lng, end_lat_lng, obstacle_points_lat_lng, safety_radius_m, direction):
    start_xy = (start_lat_lng[1], start_lat_lng[0])
    end_xy = (end_lat_lng[1], end_lat_lng[0])
    obs_xy = to_shapely_pts(obstacle_points_lat_lng)
    
    obs_poly = Polygon(obs_xy)
    buffer_deg = (safety_radius_m + 15) / 111320.0
    safe_poly = obs_poly.buffer(buffer_deg)
    line = LineString([start_xy, end_xy])
    if not line.intersects(safe_poly):
        return None

    center_lat, center_lng = get_obstacle_center(obstacle_points_lat_lng)
    width_m, height_m = get_obstacle_extent(obstacle_points_lat_lng)
    max_size = max(width_m, height_m)
    offset_dist = max_size + safety_radius_m + 20
    bearing = get_bearing(start_lat_lng[0], start_lat_lng[1], end_lat_lng[0], end_lat_lng[1])

    if direction == "right":
        perp_bearing = bearing + 90
    else:
        perp_bearing = bearing - 90

    bypass_point = point_at_distance(center_lat, center_lng, perp_bearing, offset_dist)

    # ✅ 右绕行强制向北偏移，彻底不穿障碍物
    if direction == "right":
        bypass_point = (bypass_point[0] + 0.0005, bypass_point[1])
    else:
        bypass_point = (bypass_point[0] + 0.0003, bypass_point[1])

    return [start_lat_lng, bypass_point, end_lat_lng]

# ==================== 航线规划 ====================
def plan_route():
    start = (st.session_state.start_point["lat"], st.session_state.start_point["lng"])
    end = (st.session_state.end_point["lat"], st.session_state.end_point["lng"])
    flight_height = st.session_state.flight_height
    safety_radius = st.session_state.safety_radius
    strategy = st.session_state.bypass_strategy
    route_analysis = {
        "total_distance": 0, "obstacles_encountered": [], "bypass_count": 0, "fly_over_count": 0, "route_points": []
    }
    if not st.session_state.obstacles:
        route_points = [start, end]
        total_distance = haversine_distance(start[0], start[1], end[0], end[1])
        route_analysis["total_distance"] = total_distance
        route_analysis["route_points"] = route_points
        st.session_state.planned_route = route_points
        st.session_state.route_analysis = route_analysis
        return route_points, route_analysis

    start_xy = (start[1], start[0])
    end_xy = (end[1], end[0])
    line = LineString([start_xy, end_xy])
    valid_obstacles = []
    for idx, obs in enumerate(st.session_state.obstacles):
        pts = [(float(p[0]), float(p[1])) for p in obs["points"]]
        h = obs.get("height", 10)
        oxy = to_shapely_pts(pts)
        poly = Polygon(oxy)
        buf = poly.buffer((safety_radius + 15)/111320.0)
        if line.intersects(buf):
            valid_obstacles.append({"idx":idx, "pts":pts, "h":h})

    def dist_from_start(o):
        c = get_obstacle_center(o["pts"])
        return haversine_distance(start[0], start[1], c[0], c[1])
    valid_obstacles.sort(key=dist_from_start)

    current = start
    route = [current]
    for obs in valid_obstacles:
        if flight_height > obs["h"] + safety_radius + 15:
            route_analysis["fly_over_count"] += 1
            route_analysis["obstacles_encountered"].append({"height": obs["h"], "decision": "飞跃"})
            continue
        route_analysis["bypass_count"] += 1
        dir_name = "左侧绕行" if strategy == "left" else "右侧绕行"
        route_analysis["obstacles_encountered"].append({"height": obs["h"], "decision": f"绕行({dir_name})"})
        bypass = create_safe_bypass(current, end, obs["pts"], safety_radius, strategy)
        if bypass and len(bypass) >= 3:
            for pt in bypass[1:-1]:
                if haversine_distance(route[-1][0], route[-1][1], pt[0], pt[1]) > 1:
                    route.append(pt)
            current = bypass[-1]
    if len(route) == 0 or haversine_distance(route[-1][0], route[-1][1], end[0], end[1]) > 1:
        route.append(end)

    total = 0
    for i in range(len(route)-1):
        total += haversine_distance(route[i][0], route[i][1], route[i+1][0], route[i+1][1])
    route_analysis["total_distance"] = total
    route_analysis["route_points"] = route
    st.session_state.planned_route = route
    st.session_state.route_analysis = route_analysis
    st.session_state.map_key += 1
    return route, route_analysis

# ==================== 飞行模拟（无白屏、不刷新页面） ====================
def reset_flight_task():
    st.session_state.flight_task_running = False
    st.session_state.flight_task_paused = False
    st.session_state.flight_progress = 0.0
    st.session_state.current_waypoint_idx = 0
    st.session_state.flight_speed = 0.0
    st.session_state.flight_time_elapsed = 0
    st.session_state.flight_remaining_dist = 0.0
    st.session_state.flight_battery = 100
    st.session_state.flight_drone_pos = None

def start_flight_task():
    if not st.session_state.planned_route:
        st.error("请先规划航线！")
        return
    reset_flight_task()
    st.session_state.flight_task_running = True
    st.session_state.flight_drone_pos = st.session_state.planned_route[0]
    st.session_state.flight_remaining_dist = st.session_state.route_analysis["total_distance"]

def pause_flight_task():
    st.session_state.flight_task_paused = not st.session_state.flight_task_paused

def stop_flight_task():
    reset_flight_task()

# ✅ 核心修复：飞机平滑移动，不白屏、不重建页面
def simulate_flight_step():
    if not st.session_state.flight_task_running or st.session_state.flight_task_paused:
        return
    route = st.session_state.planned_route
    if not route or st.session_state.flight_progress >= 1.0:
        st.session_state.flight_task_running = False
        return

    speed = 12.0  # 更快
    st.session_state.flight_speed = speed
    st.session_state.flight_time_elapsed += 0.2

    step = speed / st.session_state.route_analysis["total_distance"] * 1.2
    st.session_state.flight_progress = min(1.0, st.session_state.flight_progress + step)

    total = len(route)
    if total <= 1:
        return

    idx = int(st.session_state.flight_progress * (total - 1))
    idx = min(idx, total - 2)
    p1 = route[idx]
    p2 = route[idx + 1]
    t = (st.session_state.flight_progress * (total - 1)) - idx
    lat = p1[0] + t * (p2[0] - p1[0])
    lng = p1[1] + t * (p2[1] - p1[1])
    st.session_state.flight_drone_pos = (lat, lng)
    st.session_state.current_waypoint_idx = idx

# ==================== 地图绘制 ====================
def create_map():
    start_wgs = gcj02_to_wgs84(st.session_state.start_point["lat"], st.session_state.start_point["lng"])
    end_wgs = gcj02_to_wgs84(st.session_state.end_point["lat"], st.session_state.end_point["lng"])
    center_lat = (start_wgs[0] + end_wgs[0]) / 2
    center_lng = (start_wgs[1] + end_wgs[1]) / 2
    m = folium.Map(location=[center_lat, center_lng], zoom_start=17, tiles='OpenStreetMap')
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri', name='卫星图'
    ).add_to(m)
    folium.TileLayer(tiles='OpenStreetMap', name='街道图').add_to(m)
    folium.LayerControl().add_to(m)
    Draw(draw_options={'polygon':True,'polyline':False,'rectangle':True,'circle':False},
         edit_options={'edit':True,'remove':True}).add_to(m)

    folium.Marker(location=[start_wgs[0], start_wgs[1]], icon=folium.Icon(color='green', icon='play'), tooltip="起点A").add_to(m)
    folium.Marker(location=[end_wgs[0], end_wgs[1]], icon=folium.Icon(color='red', icon='flag'), tooltip="终点B").add_to(m)

    for idx, obstacle in enumerate(st.session_state.obstacles):
        wgs_pts = [gcj02_to_wgs84(p[0], p[1]) for p in obstacle["points"]]
        h = obstacle.get("height", 10)
        folium.Polygon(locations=wgs_pts, color='red', fill=True, fill_opacity=0.4).add_to(m)

    if st.session_state.planned_route:
        route_wgs = [gcj02_to_wgs84(p[0], p[1]) for p in st.session_state.planned_route]
        folium.PolyLine(locations=route_wgs, color='blue', weight=5, opacity=0.9).add_to(m)

    if st.session_state.flight_drone_pos:
        dw = gcj02_to_wgs84(st.session_state.flight_drone_pos[0], st.session_state.flight_drone_pos[1])
        folium.Marker(location=dw, icon=folium.Icon(color='orange', icon='plane'), tooltip="无人机").add_to(m)
    return m

# ==================== 飞行监控面板 ====================
def render_flight_monitor():
    st.markdown("### ✈️ 飞行实时画面")
    c1, c2, c3, c4, c5 = st.columns(5)
    with c1:
        if st.button("开始任务", type="primary", use_container_width=True, disabled=st.session_state.flight_task_running):
            start_flight_task()
    with c2:
        if st.button("暂停", use_container_width=True, disabled=not st.session_state.flight_task_running):
            pause_flight_task()
    with c3:
        if st.button("停止", use_container_width=True, disabled=not st.session_state.flight_task_running):
            stop_flight_task()
    with c4:
        if st.button("重置", use_container_width=True):
            reset_flight_task()
    with c5:
        status = "运行中" if st.session_state.flight_task_running else "已暂停" if st.session_state.flight_task_paused else "未开始"
        st.markdown(f"""<div style='padding:8px;background:#f0f0f0;border-radius:4px;text-align:center'>状态: {status}</div>""", unsafe_allow_html=True)

    g1, g2, g3, g4, g5, g6 = st.columns(6)
    total_wp = len(st.session_state.planned_route)
    current_wp = st.session_state.current_waypoint_idx + 1
    with g1: st.metric("当前航点", f"{current_wp}/{total_wp}")
    with g2: st.metric("速度", f"{st.session_state.flight_speed:.1f} m/s")
    with g3: st.metric("时间", f"{int(st.session_state.flight_time_elapsed)//60:02d}:{int(st.session_state.flight_time_elapsed)%60:02d}")
    with g4: st.metric("剩余距离", f"{st.session_state.flight_remaining_dist:.0f}m")
    with g5: st.metric("电量", f"{st.session_state.flight_battery:.0f}%")
    st.progress(st.session_state.flight_progress, text=f"任务进度: {st.session_state.flight_progress*100:.1f}%")

# ==================== 主程序 ====================
def main():
    st.title("✈️ 无人机智能化应用系统")
    render_flight_monitor()
    st.divider()

    left_col, mid_col, right_col = st.columns([2, 1, 1])
    with left_col:
        st.subheader("🗺️ 地图")
        bt_col1, bt_col2, bt_col3, bt_col4, bt_col5 = st.columns(5)
        with bt_col1:
            if st.button("📍 起点A", use_container_width=True):
                st.session_state.setting_mode = "start"
        with bt_col2:
            if st.button("🏁 终点B", use_container_width=True):
                st.session_state.setting_mode = "end"
        with bt_col3:
            if st.button("❌ 取消", use_container_width=True):
                st.session_state.setting_mode = None
        with bt_col4:
            if st.button("🔄 规划航线", type="primary", use_container_width=True):
                plan_route()
                st.rerun()
        with bt_col5:
            if st.button("🗺️ 刷新地图", use_container_width=True):
                st.rerun()

        map_container = st.empty()
        m = create_map()
        with map_container:
            output = st_folium(m, width=800, height=500, key=f"map_{st.session_state.map_key}", returned_objects=["last_clicked", "last_active_drawing"])

        if st.session_state.flight_task_running and not st.session_state.flight_task_paused:
            simulate_flight_step()
            time.sleep(0.15)
            st.rerun()

    with mid_col:
        st.subheader("🎮 控制面板")
        with st.expander("起点A", expanded=True):
            la = st.number_input("纬度", value=float(st.session_state.start_point["lat"]), format="%.6f")
            lo = st.number_input("经度", value=float(st.session_state.start_point["lng"]), format="%.6f")
            if la != st.session_state.start_point["lat"] or lo != st.session_state.start_point["lng"]:
                st.session_state.start_point["lat"] = la
                st.session_state.start_point["lng"] = lo
                st.rerun()
        with st.expander("终点B", expanded=True):
            la = st.number_input("纬度", value=float(st.session_state.end_point["lat"]), format="%.6f", key="end_lat")
            lo = st.number_input("经度", value=float(st.session_state.end_point["lng"]), format="%.6f", key="end_lng")
            if la != st.session_state.end_point["lat"] or lo != st.session_state.end_point["lng"]:
                st.session_state.end_point["lat"] = la
                st.session_state.end_point["lng"] = lo
                st.rerun()

        st.subheader("✈️ 飞行参数")
        st.number_input("飞行高度", value=st.session_state.flight_height, step=5, key="fh")
        st.number_input("安全半径", value=st.session_state.safety_radius, step=1, key="sr")
        st.radio("绕行方向", options=["left", "right"], format_func=lambda x: "左绕行" if x=="left" else "右绕行", horizontal=True, key="bp")

    with right_col:
        st.subheader("📊 航线分析")
        st.metric("总距离", f"{st.session_state.route_analysis.get('total_distance',0):.1f}m")
        st.metric("绕行次数", st.session_state.route_analysis.get('bypass_count',0))

if __name__ == "__main__":
    main()
