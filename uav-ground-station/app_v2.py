"""
无人机地面站系统 - 智能任务规划平台
功能：心跳包、地图显示、GCJ-02坐标转换、障碍物多边形圈选、航线规划、绕行策略、实时飞行监控（自动连续飞行）
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
from shapely.geometry import Point, Polygon, LineString
from streamlit_autorefresh import st_autorefresh  # 需要安装：pip install streamlit-autorefresh

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
    # 飞行监控相关状态（自动飞行模式）
    if 'auto_flight_enabled' not in st.session_state:
        st.session_state.auto_flight_enabled = False    # 是否允许自动前进
    if 'flight_progress' not in st.session_state:
        st.session_state.flight_progress = 0.0
    if 'current_waypoint_idx' not in st.session_state:
        st.session_state.current_waypoint_idx = 0
    if 'flight_remaining_dist' not in st.session_state:
        st.session_state.flight_remaining_dist = 0.0
    if 'flight_battery' not in st.session_state:
        st.session_state.flight_battery = 100
    if 'flight_drone_pos' not in st.session_state:
        st.session_state.flight_drone_pos = None
    if 'flight_time_elapsed' not in st.session_state:
        st.session_state.flight_time_elapsed = 0
    if 'flight_speed' not in st.session_state:
        st.session_state.flight_speed = 8.0   # 模拟速度 m/s

init_session_state()

# ==================== GCJ-02 转 WGS84 坐标系转换 ====================
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

# ==================== 地理计算工具函数 ====================
def haversine_distance(lat1, lng1, lat2, lng2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def meters_to_degrees(meters, lat):
    lat_deg = meters / 111320.0
    lng_deg = meters / (111320.0 * math.cos(math.radians(lat)))
    return lat_deg, lng_deg

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

def from_shapely_pt(pt_xy):
    return (pt_xy[1], pt_xy[0])

# ==================== 精确安全绕行 ====================
def create_safe_bypass(start_lat_lng, end_lat_lng, obstacle_points_lat_lng, safety_radius_m, direction):
    start_xy = (start_lat_lng[1], start_lat_lng[0])
    end_xy = (end_lat_lng[1], end_lat_lng[0])
    obs_xy = to_shapely_pts(obstacle_points_lat_lng)
    
    obs_poly = Polygon(obs_xy)
    buffer_deg = (safety_radius_m + 3) / 111320.0
    safe_poly = obs_poly.buffer(buffer_deg)
    
    line = LineString([start_xy, end_xy])
    if not line.intersects(safe_poly):
        return None

    center_lat, center_lng = get_obstacle_center(obstacle_points_lat_lng)
    width_m, height_m = get_obstacle_extent(obstacle_points_lat_lng)
    max_size = max(width_m, height_m)
    offset_dist = max_size + safety_radius_m + 6

    bearing = get_bearing(start_lat_lng[0], start_lat_lng[1], end_lat_lng[0], end_lat_lng[1])
    perp_bearing = bearing + 90 if direction == "right" else bearing - 90

    bypass_point = point_at_distance(center_lat, center_lng, perp_bearing, offset_dist)
    return [start_lat_lng, bypass_point, end_lat_lng]

# ==================== 规划航线 ====================
def plan_route():
    start = (st.session_state.start_point["lat"], st.session_state.start_point["lng"])
    end = (st.session_state.end_point["lat"], st.session_state.end_point["lng"])
    flight_height = st.session_state.flight_height
    safety_radius = st.session_state.safety_radius
    strategy = st.session_state.bypass_strategy

    route_analysis = {
        "total_distance": 0,
        "obstacles_encountered": [],
        "bypass_count": 0,
        "fly_over_count": 0,
        "route_points": []
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
        buf = poly.buffer((safety_radius + 2)/111320.0)
        if line.intersects(buf):
            valid_obstacles.append({"idx":idx, "pts":pts, "h":h})

    def dist_from_start(o):
        c = get_obstacle_center(o["pts"])
        return haversine_distance(start[0], start[1], c[0], c[1])
    valid_obstacles.sort(key=dist_from_start)

    current = start
    route = [current]

    for obs in valid_obstacles:
        if flight_height > obs["h"] + safety_radius + 5:
            route_analysis["fly_over_count"] += 1
            route_analysis["obstacles_encountered"].append({
                "height": obs["h"], "decision": "飞跃"
            })
            continue

        route_analysis["bypass_count"] += 1
        dir_name = "左侧绕行" if strategy == "left" else "右侧绕行"
        route_analysis["obstacles_encountered"].append({
            "height": obs["h"], "decision": f"绕行({dir_name})"
        })

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

# ==================== 自动飞行控制 ====================
def reset_flight():
    """重置飞行进度"""
    st.session_state.auto_flight_enabled = False
    st.session_state.flight_progress = 0.0
    st.session_state.current_waypoint_idx = 0
    st.session_state.flight_remaining_dist = st.session_state.route_analysis.get("total_distance", 0)
    st.session_state.flight_battery = 100
    st.session_state.flight_drone_pos = st.session_state.planned_route[0] if st.session_state.planned_route else None
    st.session_state.flight_time_elapsed = 0

def step_forward():
    """前进一个航点（由自动刷新驱动）"""
    route = st.session_state.planned_route
    if not route:
        return
    if st.session_state.current_waypoint_idx >= len(route) - 1:
        # 已到达终点，停止自动飞行
        st.session_state.auto_flight_enabled = False
        return

    st.session_state.current_waypoint_idx += 1
    # 更新进度
    st.session_state.flight_progress = st.session_state.current_waypoint_idx / (len(route) - 1)
    # 更新无人机位置
    st.session_state.flight_drone_pos = route[st.session_state.current_waypoint_idx]
    # 更新剩余距离
    remaining = 0
    for i in range(st.session_state.current_waypoint_idx, len(route)-1):
        remaining += haversine_distance(route[i][0], route[i][1], route[i+1][0], route[i+1][1])
    st.session_state.flight_remaining_dist = remaining
    # 更新时间
    total_dist = st.session_state.route_analysis["total_distance"]
    if total_dist > 0:
        st.session_state.flight_time_elapsed = int((st.session_state.flight_progress * total_dist) / st.session_state.flight_speed)
    # 更新电量
    st.session_state.flight_battery = max(0, 100 - st.session_state.flight_progress * 5)

def render_flight_monitor():
    st.markdown("### ✈️ 飞行实时画面 - 自动连续飞行")

    # 控制按钮
    col1, col2, col3 = st.columns([1,1,1])
    with col1:
        if st.button("▶ 开始自动飞行", type="primary", use_container_width=True, disabled=st.session_state.auto_flight_enabled):
            if not st.session_state.planned_route:
                st.error("请先规划航线！")
            else:
                reset_flight()
                st.session_state.auto_flight_enabled = True
                st.rerun()
    with col2:
        if st.button("⏹️ 停止飞行", use_container_width=True, disabled=not st.session_state.auto_flight_enabled):
            st.session_state.auto_flight_enabled = False
            st.rerun()
    with col3:
        if st.button("🔄 重置", use_container_width=True):
            reset_flight()
            st.rerun()

    # 自动飞行逻辑：如果启用且未完成，则每300ms自动前进一个航点
    if st.session_state.auto_flight_enabled:
        route = st.session_state.planned_route
        if route and st.session_state.current_waypoint_idx < len(route) - 1:
            step_forward()
            # 使用 st_autorefresh 实现自动重绘（300ms后重新执行脚本）
            st_autorefresh(interval=300, key="auto_flight_refresh")
        else:
            # 飞行完成，自动关闭飞行标志
            st.session_state.auto_flight_enabled = False
            st.success("✅ 已到达终点！")

    # 显示飞行状态指标
    col1, col2, col3, col4, col5, col6 = st.columns(6)
    total_wp = len(st.session_state.planned_route) if st.session_state.planned_route else 0
    current_wp = st.session_state.current_waypoint_idx + 1
    with col1:
        st.metric("当前航点", f"{current_wp}/{total_wp}")
    with col2:
        st.metric("飞行速度", f"{st.session_state.flight_speed:.1f} m/s")
    with col3:
        minutes = st.session_state.flight_time_elapsed // 60
        seconds = st.session_state.flight_time_elapsed % 60
        st.metric("已用时间", f"{minutes:02d}:{seconds:02d}")
    with col4:
        st.metric("剩余距离", f"{st.session_state.flight_remaining_dist:.0f} m")
    with col5:
        eta_seconds = int(st.session_state.flight_remaining_dist / st.session_state.flight_speed) if st.session_state.flight_speed > 0 else 0
        st.metric("预计到达", f"{eta_seconds//60:02d}:{eta_seconds%60:02d}")
    with col6:
        st.metric("电量模拟", f"{st.session_state.flight_battery:.0f}%")

    st.progress(st.session_state.flight_progress, text=f"任务进度: {st.session_state.flight_progress*100:.1f}%")

    # 通信链路拓扑（保持不变）
    st.markdown("---")
    st.markdown("#### 📡 通信链路拓扑与数据流")
    col_status1, col_status2, col_status3 = st.columns(3)
    with col_status1:
        st.markdown("""<div style="display:flex;align-items:center;gap:8px;"><span style="color:red;">✅</span><span>GCS 在线</span></div>""", unsafe_allow_html=True)
    with col_status2:
        st.markdown("""<div style="display:flex;align-items:center;gap:8px;"><span style="color:red;">✅</span><span>OBC 在线</span></div>""", unsafe_allow_html=True)
    with col_status3:
        st.markdown("""<div style="display:flex;align-items:center;gap:8px;"><span style="color:red;">✅</span><span>FCU 在线</span></div>""", unsafe_allow_html=True)

    col_gcs, conn1, col_obc, conn2, col_fcu = st.columns([2,1,2,1,2])
    with col_gcs:
        st.markdown("""<div style="border:2px solid #4A90E2; border-radius:8px; padding:15px; text-align:center; background-color:#f0f8ff;"><div style="font-size:24px;">🖥️</div><div style="font-weight:bold;">GCS</div><div style="font-size:12px;">地面站</div><div style="font-size:12px;">192.168.1.100</div></div>""", unsafe_allow_html=True)
    with conn1:
        st.markdown("""<div style="display:flex;flex-direction:column;align-items:center;justify-content:center;height:100%;"><div style="font-size:20px;">⬆️⬇️</div><div style="font-size:12px;">UDP:14550</div><div style="font-size:12px; color:green;">🟢 已连接</div></div>""", unsafe_allow_html=True)
    with col_obc:
        st.markdown("""<div style="border:2px solid #F5C767; border-radius:8px; padding:15px; text-align:center; background-color:#fff9e6;"><div style="font-size:24px;">🧠</div><div style="font-weight:bold;">OBC</div><div style="font-size:12px;">机载计算机</div><div style="font-size:12px;">Raspberry Pi 4</div></div>""", unsafe_allow_html=True)
    with conn2:
        st.markdown("""<div style="display:flex;flex-direction:column;align-items:center;justify-content:center;height:100%;"><div style="font-size:20px;">⬆️⬇️</div><div style="font-size:12px;">MAVLink</div><div style="font-size:12px; color:green;">🟢 已连接</div></div>""", unsafe_allow_html=True)
    with col_fcu:
        st.markdown("""<div style="border:2px solid #C274D0; border-radius:8px; padding:15px; text-align:center; background-color:#f9f0ff;"><div style="font-size:24px;">⚙️</div><div style="font-weight:bold;">FCU</div><div style="font-size:12px;">飞控</div><div style="font-size:12px;">PX4 / ArduPilot</div></div>""", unsafe_allow_html=True)

    st.markdown("""<div style="padding:10px; background-color:#f8f9fa; border-radius:4px; margin-top:10px; font-size:13px;"><span style="font-weight:bold;">📊 链路统计:</span> &nbsp; GCS↔OBC: 正常 &nbsp;|&nbsp; OBC↔FCU: 正常 &nbsp;|&nbsp; 延迟: ~25ms &nbsp;|&nbsp; 丢包率: 0.1%</div>""", unsafe_allow_html=True)

# ==================== 其他功能（部署、心跳、持久化、地图、主界面） ====================
def deploy_route_to_uav():
    if not st.session_state.planned_route:
        return {"success":False,"message":"❌ 没有可部署的航线，请先规划航线","commands":[]}
    route = st.session_state.planned_route
    analysis = st.session_state.route_analysis
    commands = []
    commands.append({"seq":1,"command":"TAKEOFF","params":[0,0,0,0,st.session_state.flight_height,0,0],"description":f"起飞至 {st.session_state.flight_height}m"})
    for i, point in enumerate(route):
        commands.append({"seq":i+2,"command":"WAYPOINT","params":[0,0,0,0,st.session_state.flight_height,point[0],point[1]],"description":f"航点 {i+1}"})
    commands.append({"seq":len(route)+2,"command":"LAND","params":[0,0,0,0,0,0,0],"description":"降落"})
    deployment_report = {
        "success":True,
        "message":"✅ 航线指令已成功部署到无人机！",
        "deploy_time":datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "total_waypoints":len(route),
        "estimated_distance":analysis.get("total_distance",0),
        "flight_height":st.session_state.flight_height
    }
    st.session_state.deployment_status = deployment_report
    st.session_state.deployment_log.append({"time":deployment_report["deploy_time"],"waypoints":len(route),"distance":analysis.get("total_distance",0)})
    return deployment_report

def heartbeat():
    st.session_state.heartbeat_count += 1
    return {"status":"online","sequence":st.session_state.heartbeat_count,"timestamp":datetime.now().strftime("%H:%M:%S"),"battery":random.randint(85,100),"signal":random.randint(70,99)}

OBSTACLE_FILE = "obstacle_config.json"

def save_obstacles_to_file():
    data = {"version":"v12.2","timestamp":datetime.now().strftime("%Y-%m-%d %H:%M:%S"),"obstacles":st.session_state.obstacles}
    try:
        with open(OBSTACLE_FILE,"w",encoding="utf-8") as f:
            json.dump(data,f,ensure_ascii=False,indent=2)
        return True
    except Exception:
        return False

def load_obstacles_from_file():
    if os.path.exists(OBSTACLE_FILE):
        try:
            with open(OBSTACLE_FILE,"r",encoding="utf-8") as f:
                data = json.load(f)
                st.session_state.obstacles = data.get("obstacles",[])
                return True,len(st.session_state.obstacles),data.get("timestamp","未知")
        except Exception:
            return False,0,None
    return False,0,None

def auto_load_obstacles():
    if not st.session_state.obstacles_loaded:
        success,count,timestamp = load_obstacles_from_file()
        st.session_state.obstacles_loaded = True
        if success and count>0:
            return True,count,timestamp
    return False,0,None

def add_obstacle_from_draw(feature):
    try:
        if feature.get('geometry',{}).get('type') == 'Polygon':
            coords = feature['geometry']['coordinates'][0]
            points = []
            for coord in coords:
                gcj_lat,gcj_lng = wgs84_to_gcj02(coord[1],coord[0])
                points.append([gcj_lat,gcj_lng])
            if len(points)>1 and points[0]==points[-1]:
                points = points[:-1]
            obstacle_height = st.session_state.new_obstacle_height
            st.session_state.obstacles.append({"points":points,"height":obstacle_height,"created_at":datetime.now().strftime("%Y-%m-%d %H:%M:%S")})
            save_obstacles_to_file()
            return True
    except Exception as e:
        st.error(f"添加障碍物失败: {e}")
    return False

def remove_obstacle(index):
    if 0<=index<len(st.session_state.obstacles):
        st.session_state.obstacles.pop(index)
        save_obstacles_to_file()

def clear_all_obstacles():
    st.session_state.obstacles = []
    save_obstacles_to_file()

def create_map():
    start_wgs = gcj02_to_wgs84(st.session_state.start_point["lat"],st.session_state.start_point["lng"])
    end_wgs = gcj02_to_wgs84(st.session_state.end_point["lat"],st.session_state.end_point["lng"])
    center_lat = (start_wgs[0]+end_wgs[0])/2
    center_lng = (start_wgs[1]+end_wgs[1])/2
    m = folium.Map(location=[center_lat,center_lng],zoom_start=17,tiles='OpenStreetMap')
    folium.TileLayer(tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',attr='Esri',name='卫星图').add_to(m)
    folium.TileLayer(tiles='OpenStreetMap',name='街道图').add_to(m)
    folium.LayerControl().add_to(m)

    draw = Draw(draw_options={'polygon':True,'polyline':False,'rectangle':True,'circle':False,'marker':False,'circlemarker':False},edit_options={'edit':True,'remove':True})
    draw.add_to(m)

    folium.Marker(location=[start_wgs[0],start_wgs[1]],popup=f"起点A<br>纬度: {st.session_state.start_point['lat']:.6f}<br>经度: {st.session_state.start_point['lng']:.6f}",icon=folium.Icon(color='green',icon='play',prefix='fa'),tooltip="起点A").add_to(m)
    folium.Marker(location=[end_wgs[0],end_wgs[1]],popup=f"终点B<br>纬度: {st.session_state.end_point['lat']:.6f}<br>经度: {st.session_state.end_point['lng']:.6f}",icon=folium.Icon(color='red',icon='flag-checkered',prefix='fa'),tooltip="终点B").add_to(m)

    folium.Circle(location=[start_wgs[0],start_wgs[1]],radius=st.session_state.safety_radius,color='green',fill=True,fill_opacity=0.1,popup=f"起点安全区 R={st.session_state.safety_radius}m").add_to(m)
    folium.Circle(location=[end_wgs[0],end_wgs[1]],radius=st.session_state.safety_radius,color='red',fill=True,fill_opacity=0.1,popup=f"终点安全区 R={st.session_state.safety_radius}m").add_to(m)

    for idx,obstacle in enumerate(st.session_state.obstacles):
        wgs_points = []
        for point in obstacle["points"]:
            wgs = gcj02_to_wgs84(point[0],point[1])
            wgs_points.append([wgs[0],wgs[1]])
        obstacle_height = obstacle.get("height",10)
        folium.Polygon(locations=wgs_points,color='red',weight=2,fill=True,fill_color='red',fill_opacity=0.4,popup=f"障碍物 {idx+1} | 高度: {obstacle_height}m").add_to(m)
        try:
            poly = Polygon([(p[0],p[1]) for p in wgs_points])
            buffer_deg = st.session_state.safety_radius / 111320.0
            buffered = poly.buffer(buffer_deg)
            if buffered.geom_type == 'Polygon':
                buffer_coords = list(buffered.exterior.coords)
                folium.Polygon(locations=[(lat,lng) for lng,lat in buffer_coords],color='yellow',weight=1,fill=True,fill_color='yellow',fill_opacity=0.15,popup=f"安全区 R={st.session_state.safety_radius}m").add_to(m)
        except:
            pass
        center = [sum(p[0] for p in wgs_points)/len(wgs_points), sum(p[1] for p in wgs_points)/len(wgs_points)]
        folium.map.Marker(center,icon=folium.DivIcon(html=f'<div style="font-size:14px;color:red;font-weight:bold;">↑{obstacle_height}m</div>')).add_to(m)

    if st.session_state.planned_route:
        route_wgs = []
        for point in st.session_state.planned_route:
            wgs = gcj02_to_wgs84(point[0],point[1])
            route_wgs.append([wgs[0],wgs[1]])
        folium.PolyLine(locations=route_wgs,color='blue',weight=5,opacity=0.9,popup=f"规划航线 | 距离: {st.session_state.route_analysis.get('total_distance',0):.1f}m").add_to(m)
        for i,point in enumerate(route_wgs):
            if i==0 or i==len(route_wgs)-1:
                continue
            folium.CircleMarker(location=point,radius=4,color='cyan',fill=True,fill_color='cyan',fill_opacity=0.8,popup=f"航点 {i+1}").add_to(m)

    if st.session_state.flight_drone_pos:
        drone_wgs = gcj02_to_wgs84(st.session_state.flight_drone_pos[0], st.session_state.flight_drone_pos[1])
        folium.Marker(location=drone_wgs, icon=folium.Icon(color='orange', icon='plane', prefix='fa'), tooltip="无人机当前位置").add_to(m)

    folium.PolyLine(locations=[[start_wgs[0],start_wgs[1]],[end_wgs[0],end_wgs[1]]],color='gray',weight=2,opacity=0.6,dash_array='5,10',popup="原始直线").add_to(m)
    return m

def main():
    st.title("✈️ 无人机智能化应用系统")
    st.caption("魏坤的《无人机智能化应用2451》 | 分组作业4-项目Demo")
    loaded,count,timestamp = auto_load_obstacles()
    if loaded:
        st.success(f"💾 已加载 {count} 个障碍物")

    heartbeat_data = heartbeat()
    col1,col2,col3,col4,col5 = st.columns(5)
    with col1: st.metric("💓 心跳","在线")
    with col2: st.metric("📡 序列号",heartbeat_data["sequence"])
    with col3: st.metric("🔋 电量",f"{heartbeat_data['battery']}%")
    with col4: st.metric("📶 信号",f"{heartbeat_data['signal']}%")
    with col5: st.metric("🕐 心跳时间",heartbeat_data["timestamp"])
    st.divider()

    render_flight_monitor()
    st.divider()

    left_col,mid_col,right_col = st.columns([2,1,1])
    with left_col:
        st.subheader("🗺️ 地图")
        mode_col1,mode_col2,mode_col3,mode_col4,mode_col5 = st.columns(5)
        with mode_col1:
            if st.button("📍 起点A",use_container_width=True):
                st.session_state.setting_mode = "start"
        with mode_col2:
            if st.button("🏁 终点B",use_container_width=True):
                st.session_state.setting_mode = "end"
        with mode_col3:
            if st.button("❌ 取消",use_container_width=True):
                st.session_state.setting_mode = None
        with mode_col4:
            if st.button("🔄 规划航线",type="primary",use_container_width=True):
                with st.spinner("规划中..."):
                    route_points,analysis = plan_route()
                    st.success(f"完成！距离: {analysis['total_distance']:.1f}m | 绕行: {analysis['bypass_count']}次 | 飞跃: {analysis['fly_over_count']}次")
                    st.rerun()
        with mode_col5:
            if st.button("🗺️ 刷新地图",use_container_width=True):
                st.rerun()

        if st.session_state.setting_mode == "start":
            st.info("🔵 点击地图设置起点（绿色标记）")
        elif st.session_state.setting_mode == "end":
            st.info("🔴 点击地图设置终点（红色标记）")
        st.caption("📌 红色区域为障碍物 | 黄色区域为安全缓冲区 | 蓝色线为规划航线 | 灰色虚线为原始直线 | 橙色飞机为无人机实时位置")

        try:
            m = create_map()
            output = st_folium(m,width=800,height=500,key=f"map_{st.session_state.map_key}",returned_objects=["last_active_drawing","last_clicked"])
            if output and output.get("last_clicked"):
                clicked = output["last_clicked"]
                if clicked and "lat" in clicked and "lng" in clicked:
                    wgs_lat = clicked["lat"]
                    wgs_lng = clicked["lng"]
                    gcj_lat,gcj_lng = wgs84_to_gcj02(wgs_lat,wgs_lng)
                    if st.session_state.setting_mode == "start":
                        st.session_state.start_point = {"lat":gcj_lat,"lng":gcj_lng,"height":0}
                        st.session_state.setting_mode = None
                        st.success(f"起点已设置: ({gcj_lat:.6f}, {gcj_lng:.6f})")
                        st.rerun()
                    elif st.session_state.setting_mode == "end":
                        st.session_state.end_point = {"lat":gcj_lat,"lng":gcj_lng,"height":0}
                        st.session_state.setting_mode = None
                        st.success(f"终点已设置: ({gcj_lat:.6f}, {gcj_lng:.6f})")
                        st.rerun()
            if output and output.get("last_active_drawing"):
                feature = output["last_active_drawing"]
                if feature.get("geometry",{}).get("type") == "Polygon":
                    if add_obstacle_from_draw(feature):
                        st.success("障碍物已添加，并自动保存")
                        st.rerun()
        except Exception as e:
            st.error(f"地图错误: {e}")

        with st.expander("📖 操作说明"):
            st.markdown("""
**基本操作**
- **设置起点/终点**: 点击对应按钮，再点击地图上的位置
- **绘制障碍物**: 点击左上角多边形图标，画完后双击完成
- **规划航线**: 点击「规划航线」按钮
- **自动飞行**: 点击「开始自动飞行」，飞机会自动沿航线逐点移动（每0.3秒前进一个航点），直到终点
- **停止飞行**: 点击「停止飞行」可随时暂停，点击「重置」可重新开始

**视觉说明**
- 🔴 红色区域: 障碍物本体
- 🟡 黄色区域: 安全缓冲区
- 🔵 蓝色粗线: 规划航线
- ⚪ 灰色虚线: 原始直线
- 🟠 橙色飞机: 无人机实时位置
""")

    with mid_col:
        st.subheader("🎮 控制面板")
        with st.expander("📍 起点A",expanded=True):
            new_lat = st.number_input("纬度",value=float(st.session_state.start_point["lat"]),format="%.6f")
            new_lng = st.number_input("经度",value=float(st.session_state.start_point["lng"]),format="%.6f")
            if new_lat != st.session_state.start_point["lat"] or new_lng != st.session_state.start_point["lng"]:
                st.session_state.start_point["lat"] = new_lat
                st.session_state.start_point["lng"] = new_lng
                st.rerun()
        with st.expander("🏁 终点B",expanded=True):
            new_lat = st.number_input("纬度",value=float(st.session_state.end_point["lat"]),format="%.6f",key="end_lat")
            new_lng = st.number_input("经度",value=float(st.session_state.end_point["lng"]),format="%.6f",key="end_lng")
            if new_lat != st.session_state.end_point["lat"] or new_lng != st.session_state.end_point["lng"]:
                st.session_state.end_point["lat"] = new_lat
                st.session_state.end_point["lng"] = new_lng
                st.rerun()
        st.divider()
        st.subheader("✈️ 飞行参数")
        flight_height = st.number_input("飞行高度 (m)",value=st.session_state.flight_height,step=5,min_value=10,max_value=200)
        if flight_height != st.session_state.flight_height:
            st.session_state.flight_height = flight_height
        safety_radius = st.number_input("安全半径 (m)",value=st.session_state.safety_radius,step=1,min_value=5,max_value=50)
        if safety_radius != st.session_state.safety_radius:
            st.session_state.safety_radius = safety_radius
        bypass_options = {"left":"⬅️ 向左绕行","right":"➡️ 向右绕行"}
        selected = st.radio("绕行策略",options=list(bypass_options.keys()),format_func=lambda x: bypass_options[x],horizontal=True)
        st.session_state.bypass_strategy = selected
        st.divider()
        st.subheader("⛔ 添加障碍物")
        st.number_input("障碍物高度 (m)",value=60,step=5,min_value=10,max_value=200,key="new_obstacle_height")
        st.caption("💡 在地图上使用多边形工具绘制区域后自动添加")

    with right_col:
        st.subheader("📊 航线分析")
        if st.session_state.route_analysis:
            analysis = st.session_state.route_analysis
            st.metric("📏 总距离",f"{analysis.get('total_distance',0):.1f} m")
            st.metric("🔄 绕行次数",analysis.get('bypass_count',0))
            st.metric("✅ 飞跃次数",analysis.get('fly_over_count',0))
            if analysis.get('obstacles_encountered'):
                st.divider()
                st.caption("📋 障碍物处理详情")
                for obs in analysis.get('obstacles_encountered',[]):
                    icon = "🔄" if "绕行" in obs['decision'] else "✅"
                    st.text(f"{icon} {obs['height']}m → {obs['decision']}")
        else:
            st.info("点击「规划航线」生成报告")
        st.divider()
        st.subheader("⛔ 障碍物列表")
        if st.session_state.obstacles:
            st.caption(f"共 {len(st.session_state.obstacles)} 个障碍物")
            for idx,obs in enumerate(st.session_state.obstacles):
                col1,col2,col3 = st.columns([1,2,1])
                with col1:
                    if st.button("🗑️",key=f"del_{idx}"):
                        remove_obstacle(idx)
                        st.rerun()
                with col2: st.text(f"障碍 {idx+1}")
                with col3: st.text(f"{obs.get('height',10)}m")
        else:
            st.info("暂无障碍物")
        st.divider()
        col_s,col_l,col_c = st.columns(3)
        with col_s:
            if st.button("💾 保存障碍物",use_container_width=True):
                save_obstacles_to_file()
                st.success("已保存")
        with col_l:
            if st.button("📂 加载障碍物",use_container_width=True):
                load_obstacles_from_file()
                st.success(f"已加载 {len(st.session_state.obstacles)} 个障碍物")
                st.rerun()
        with col_c:
            if st.button("🗑️ 清空所有",use_container_width=True):
                clear_all_obstacles()
                st.success("已清空")
                st.rerun()
        st.divider()
        st.subheader("🚁 任务部署")
        if st.button("🚀 部署航线到无人机",type="primary",use_container_width=True):
            result = deploy_route_to_uav()
            if result["success"]:
                st.success(result["message"])
                st.balloons()
                st.json({
                    "航点数":result["total_waypoints"],
                    "总距离(m)":f"{result['estimated_distance']:.1f}",
                    "飞行高度(m)":result["flight_height"],
                    "部署时间":result["deploy_time"]
                })
            else:
                st.error(result["message"])

if __name__ == "__main__":
    main()
