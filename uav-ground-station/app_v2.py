"""
无人机地面站系统 - 智能任务规划平台
功能：心跳包、地图显示、GCJ-02坐标转换、障碍物多边形圈选、航线规划、绕行策略
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
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="无人机地面站系统",
    page_icon="✈️",
    layout="wide"
)

# ==================== 初始化 Session State ====================
def init_session_state():
    """初始化所有会话变量"""
    if 'heartbeat_count' not in st.session_state:
        st.session_state.heartbeat_count = 0
    if 'obstacles' not in st.session_state:
        st.session_state.obstacles = []
    if 'start_point' not in st.session_state:
        st.session_state.start_point = {"lat": 32.2323, "lng": 118.749, "height": 0}
    if 'end_point' not in st.session_state:
        st.session_state.end_point = {"lat": 32.2344, "lng": 118.749, "height": 0}
    if 'flight_height' not in st.session_state:
        st.session_state.flight_height = 50  # 默认飞行高度50米
    if 'safety_radius' not in st.session_state:
        st.session_state.safety_radius = 5  # 默认安全半径5米
    if 'bypass_strategy' not in st.session_state:
        st.session_state.bypass_strategy = "best"  # left, right, best
    if 'planned_route' not in st.session_state:
        st.session_state.planned_route = []
    if 'route_analysis' not in st.session_state:
        st.session_state.route_analysis = {}
    if 'map_center' not in st.session_state:
        st.session_state.map_center = [32.2333, 118.749]

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
    """计算两点之间的距离（米）"""
    R = 6371000  # 地球半径（米）
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    
    a = math.sin(delta_phi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def calculate_bearing(lat1, lng1, lat2, lng2):
    """计算方位角（度）"""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lng = math.radians(lng2 - lng1)
    
    x = math.sin(delta_lng) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lng)
    
    bearing = math.atan2(x, y)
    return math.degrees(bearing)

def get_point_at_distance(lat, lng, bearing, distance):
    """根据起点、方位角和距离获取目标点"""
    R = 6371000
    bearing_rad = math.radians(bearing)
    lat_rad = math.radians(lat)
    lng_rad = math.radians(lng)
    
    lat2_rad = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                          math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    
    lng2_rad = lng_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                     math.cos(distance / R) - math.sin(lat_rad) * math.sin(lat2_rad))
    
    return math.degrees(lat2_rad), math.degrees(lng2_rad)

# ==================== 障碍物与航线规划 ====================
def get_obstacle_max_height(obstacle):
    """获取障碍物的最大高度（默认10米，可配置）"""
    return obstacle.get("height", 10)

def check_obstacle_intersection(start, end, obstacle_polygon, safety_radius):
    """检查航线是否与障碍物相交"""
    line = LineString([(start[1], start[0]), (end[1], end[0])])
    buffer = obstacle_polygon.buffer(safety_radius / 111000)  # 转换为度
    return line.intersects(buffer)

def calculate_bypass_points(start, end, obstacle_polygon, direction, safety_radius):
    """计算绕行航点"""
    # 获取障碍物边界
    bounds = obstacle_polygon.bounds
    center = obstacle_polygon.centroid
    
    # 计算绕行偏移量
    offset = safety_radius / 111000
    
    if direction == "left":
        # 向左绕行
        bypass1 = (center.y - offset, bounds[0] - offset)
        bypass2 = (center.y - offset, bounds[2] + offset)
    elif direction == "right":
        # 向右绕行
        bypass1 = (center.y + offset, bounds[0] - offset)
        bypass2 = (center.y + offset, bounds[2] + offset)
    else:
        # 最佳航线（最短路径）
        # 找到障碍物上距离航线最近的点
        line = LineString([(start[1], start[0]), (end[1], end[0])])
        nearest = nearest_points(line, obstacle_polygon)[1]
        bypass1 = (nearest.y + offset, nearest.x)
        bypass2 = (nearest.y - offset, nearest.x)
    
    return [(bypass1[0], bypass1[1]), (bypass2[0], bypass2[1])]

def plan_route():
    """规划航线，考虑障碍物和飞行高度"""
    start = (st.session_state.start_point["lat"], st.session_state.start_point["lng"])
    end = (st.session_state.end_point["lat"], st.session_state.end_point["lng"])
    flight_height = st.session_state.flight_height
    safety_radius = st.session_state.safety_radius
    
    route_points = [start]
    route_analysis = {
        "total_distance": 0,
        "obstacles_encountered": [],
        "bypass_count": 0,
        "fly_over_count": 0
    }
    
    # 构建障碍物多边形列表
    obstacle_polygons = []
    for obs in st.session_state.obstacles:
        points = [(p[1], p[0]) for p in obs["points"]]  # (lng, lat) for shapely
        if len(points) >= 3:
            poly = Polygon(points)
            obstacle_polygons.append({
                "polygon": poly,
                "height": obs.get("height", 10),
                "points": obs["points"]
            })
    
    current_point = start
    remaining_obstacles = obstacle_polygons.copy()
    
    # 简化的航线规划：检查每个障碍物
    for obs_data in obstacle_polygons:
        poly = obs_data["polygon"]
        obs_height = obs_data["height"]
        
        # 检查当前点到终点的线段是否与障碍物相交
        line = LineString([(current_point[1], current_point[0]), (end[1], end[0])])
        
        if line.intersects(poly.buffer(safety_radius / 111000)):
            route_analysis["obstacles_encountered"].append({
                "height": obs_height,
                "decision": ""
            })
            
            # 决策：飞跃还是绕行
            if flight_height > obs_height + safety_radius:
                # 直接飞跃
                route_analysis["fly_over_count"] += 1
                route_analysis["obstacles_encountered"][-1]["decision"] = "飞跃"
            else:
                # 需要绕行
                route_analysis["bypass_count"] += 1
                strategy = st.session_state.bypass_strategy
                
                if strategy == "left":
                    bypass_points = calculate_bypass_points(current_point, end, poly, "left", safety_radius)
                elif strategy == "right":
                    bypass_points = calculate_bypass_points(current_point, end, poly, "right", safety_radius)
                else:
                    # 最佳航线：计算左右绕行距离，选择较短的
                    left_points = calculate_bypass_points(current_point, end, poly, "left", safety_radius)
                    right_points = calculate_bypass_points(current_point, end, poly, "right", safety_radius)
                    
                    left_dist = haversine_distance(current_point[0], current_point[1], left_points[0][0], left_points[0][1]) + \
                                haversine_distance(left_points[0][0], left_points[0][1], left_points[1][0], left_points[1][1]) + \
                                haversine_distance(left_points[1][0], left_points[1][1], end[0], end[1])
                    
                    right_dist = haversine_distance(current_point[0], current_point[1], right_points[0][0], right_points[0][1]) + \
                                 haversine_distance(right_points[0][0], right_points[0][1], right_points[1][0], right_points[1][1]) + \
                                 haversine_distance(right_points[1][0], right_points[1][1], end[0], end[1])
                    
                    if left_dist <= right_dist:
                        bypass_points = left_points
                        strategy = "left"
                    else:
                        bypass_points = right_points
                        strategy = "right"
                
                route_analysis["obstacles_encountered"][-1]["decision"] = f"绕行({strategy})"
                
                # 添加绕行点
                route_points.extend(bypass_points)
                current_point = bypass_points[-1]
    
    # 添加终点
    if route_points[-1] != end:
        route_points.append(end)
    
    # 计算总距离
    total_distance = 0
    for i in range(len(route_points) - 1):
        total_distance += haversine_distance(
            route_points[i][0], route_points[i][1],
            route_points[i + 1][0], route_points[i + 1][1]
        )
    
    route_analysis["total_distance"] = total_distance
    route_analysis["route_points"] = route_points
    
    st.session_state.planned_route = route_points
    st.session_state.route_analysis = route_analysis
    
    return route_points, route_analysis

# ==================== 心跳包模拟 ====================
def heartbeat():
    st.session_state.heartbeat_count += 1
    return {
        "status": "online",
        "sequence": st.session_state.heartbeat_count,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "battery": random.randint(85, 100),
        "signal": random.randint(70, 99)
    }

# ==================== 障碍物持久化 ====================
OBSTACLE_FILE = "obstacle_config.json"

def save_obstacles_to_file():
    data = {
        "version": "v12.2",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "obstacles": st.session_state.obstacles
    }
    with open(OBSTACLE_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    return True

def load_obstacles_from_file():
    if os.path.exists(OBSTACLE_FILE):
        with open(OBSTACLE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles = data.get("obstacles", [])
            return True
    return False

def add_obstacle_from_draw(feature):
    """从绘制的多边形添加障碍物"""
    try:
        if feature.get('geometry', {}).get('type') == 'Polygon':
            coords = feature['geometry']['coordinates'][0]
            points = []
            for coord in coords:
                gcj_lat, gcj_lng = wgs84_to_gcj02(coord[1], coord[0])
                points.append([gcj_lat, gcj_lng])
            
            if len(points) > 1 and points[0] == points[-1]:
                points = points[:-1]
            
            # 添加障碍物高度（默认10米）
            obstacle_height = st.session_state.get("new_obstacle_height", 10)
            
            st.session_state.obstacles.append({
                "points": points,
                "height": obstacle_height,
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

# ==================== 地图创建 ====================
def create_map():
    """创建带绘图工具的 Folium 地图"""
    start_wgs = gcj02_to_wgs84(
        float(st.session_state.start_point["lat"]),
        float(st.session_state.start_point["lng"])
    )
    end_wgs = gcj02_to_wgs84(
        float(st.session_state.end_point["lat"]),
        float(st.session_state.end_point["lng"])
    )
    
    center_lat = (start_wgs[0] + end_wgs[0]) / 2
    center_lng = (start_wgs[1] + end_wgs[1]) / 2
    
    if math.isnan(center_lat) or math.isnan(center_lng):
        center_lat = 32.2333
        center_lng = 118.749
    
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=16,
        tiles='OpenStreetMap'
    )
    
    # 添加卫星图层
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri',
        name='卫星图'
    ).add_to(m)
    
    folium.TileLayer(
        tiles='OpenStreetMap',
        name='街道图'
    ).add_to(m)
    
    folium.LayerControl().add_to(m)
    
    # 添加绘图工具
    draw = Draw(
        draw_options={
            'polygon': True,
            'polyline': False,
            'rectangle': False,
            'circle': False,
            'marker': False,
            'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    draw.add_to(m)
    
    # 起点标记
    folium.Marker(
        location=[start_wgs[0], start_wgs[1]],
        popup=f"起点A (高度: {st.session_state.start_point.get('height', 0)}m)",
        icon=folium.Icon(color='green', icon='play', prefix='fa'),
        tooltip="起点A"
    ).add_to(m)
    
    # 终点标记
    folium.Marker(
        location=[end_wgs[0], end_wgs[1]],
        popup=f"终点B (高度: {st.session_state.end_point.get('height', 0)}m)",
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'),
        tooltip="终点B"
    ).add_to(m)
    
    # 绘制已保存的障碍物多边形
    for idx, obstacle in enumerate(st.session_state.obstacles):
        wgs_points = []
        for point in obstacle["points"]:
            wgs = gcj02_to_wgs84(float(point[0]), float(point[1]))
            wgs_points.append([wgs[0], wgs[1]])
        
        obstacle_height = obstacle.get("height", 10)
        
        folium.Polygon(
            locations=wgs_points,
            color='red',
            weight=2,
            fill=True,
            fill_color='red',
            fill_opacity=0.3,
            popup=f"障碍物 {idx + 1} | 高度: {obstacle_height}m"
        ).add_to(m)
        
        # 添加高度标注
        center = [sum(p[0] for p in wgs_points) / len(wgs_points),
                  sum(p[1] for p in wgs_points) / len(wgs_points)]
        folium.map.Marker(
            center,
            icon=folium.DivIcon(html=f'<div style="font-size: 10px; color: red;">↑{obstacle_height}m</div>')
        ).add_to(m)
    
    # 绘制规划航线
    if st.session_state.planned_route:
        route_wgs = []
        for point in st.session_state.planned_route:
            wgs = gcj02_to_wgs84(point[0], point[1])
            route_wgs.append([wgs[0], wgs[1]])
        
        # 航线
        folium.PolyLine(
            locations=route_wgs,
            color='purple',
            weight=4,
            opacity=0.9,
            popup=f"规划航线 | 总距离: {st.session_state.route_analysis.get('total_distance', 0):.1f}m"
        ).add_to(m)
        
        # 航点标记
        for i, point in enumerate(route_wgs[1:-1], 1):
            folium.CircleMarker(
                location=point,
                radius=5,
                color='orange',
                fill=True,
                popup=f"航点 {i}"
            ).add_to(m)
    
    # 原始直线航线（对比）
    folium.PolyLine(
        locations=[[start_wgs[0], start_wgs[1]], [end_wgs[0], end_wgs[1]]],
        color='gray',
        weight=2,
        opacity=0.5,
        dash_array='5, 5',
        popup="原始直线航线"
    ).add_to(m)
    
    return m

# ==================== 主界面 ====================
def main():
    st.title("✈️ 无人机智能化应用系统")
    st.caption("魏坤的《无人机智能化应用2451》 | 分组作业4-项目Demo | 智能航线规划系统")
    
    # 心跳包状态栏
    heartbeat_data = heartbeat()
    col1, col2, col3, col4, col5 = st.columns(5)
    with col1:
        st.metric("💓 心跳状态", "在线")
    with col2:
        st.metric("📡 序列号", heartbeat_data["sequence"])
    with col3:
        st.metric("🔋 电量", f"{heartbeat_data['battery']}%")
    with col4:
        st.metric("📶 信号强度", f"{heartbeat_data['signal']}%")
    with col5:
        st.metric("🕐 最后心跳", heartbeat_data["timestamp"])
    
    st.divider()
    
    # 三栏布局
    left_col, mid_col, right_col = st.columns([2, 1, 1])
    
    with left_col:
        st.subheader("🗺️ 地图显示 (OpenStreetMap)")
        st.caption("📍 使用左侧工具栏的【多边形】按钮圈选障碍物 | 紫色线为规划航线 | 灰色虚线为原始直线")
        
        try:
            m = create_map()
            output = st_folium(
                m, 
                width=800, 
                height=550,
                returned_objects=["last_active_drawing"]
            )
            
            if output and output.get("last_active_drawing"):
                feature = output["last_active_drawing"]
                if feature.get("geometry", {}).get("type") == "Polygon":
                    if add_obstacle_from_draw(feature):
                        st.success("✅ 障碍物已添加！")
                        st.rerun()
                        
        except Exception as e:
            st.error(f"地图加载出错: {e}")
            st.info("请刷新页面重试")
        
        with st.expander("📖 地图操作说明"):
            st.markdown("""
            - **缩放**: 鼠标滚轮
            - **移动**: 拖拽地图
            - **圈选障碍物**: 点击地图左上角的【多边形】图标 ✏️
            - **绘制多边形**: 在地图上依次点击顶点，双击完成绘制
            - **切换图层**: 点击右上角图层按钮
            - **紫色线**: 智能规划航线
            - **灰色虚线**: 原始直线航线
            """)
    
    with mid_col:
        st.subheader("🎮 控制面板")
        
        # 起点设置
        with st.expander("📍 起点A (GCJ-02)", expanded=True):
            col_a1, col_a2 = st.columns(2)
            with col_a1:
                start_lat = st.number_input(
                    "纬度", value=float(st.session_state.start_point["lat"]),
                    format="%.6f", key="start_lat"
                )
            with col_a2:
                start_lng = st.number_input(
                    "经度", value=float(st.session_state.start_point["lng"]),
                    format="%.6f", key="start_lng"
                )
            start_height = st.number_input("起点高度(m)", value=0, step=1, key="start_height")
            if st.button("设置A点", use_container_width=True):
                st.session_state.start_point = {"lat": float(start_lat), "lng": float(start_lng), "height": start_height}
                st.success(f"起点已设置")
                st.rerun()
        
        # 终点设置
        with st.expander("🏁 终点B (GCJ-02)", expanded=True):
            col_b1, col_b2 = st.columns(2)
            with col_b1:
                end_lat = st.number_input(
                    "纬度", value=float(st.session_state.end_point["lat"]),
                    format="%.6f", key="end_lat"
                )
            with col_b2:
                end_lng = st.number_input(
                    "经度", value=float(st.session_state.end_point["lng"]),
                    format="%.6f", key="end_lng"
                )
            end_height = st.number_input("终点高度(m)", value=0, step=1, key="end_height")
            if st.button("设置B点", use_container_width=True):
                st.session_state.end_point = {"lat": float(end_lat), "lng": float(end_lng), "height": end_height}
                st.success(f"终点已设置")
                st.rerun()
        
        st.divider()
        
        # 航线规划参数
        st.subheader("✈️ 航线规划参数")
        
        flight_height = st.number_input(
            "无人机飞行高度 (m)", 
            value=st.session_state.flight_height,
            step=5,
            key="flight_height_input",
            help="无人机巡航高度，高于障碍物时直接飞跃"
        )
        
        safety_radius = st.number_input(
            "安全半径 (m)", 
            value=st.session_state.safety_radius,
            step=1,
            min_value=1,
            max_value=50,
            key="safety_radius_input",
            help="无人机与障碍物的安全距离"
        )
        
        st.session_state.flight_height = flight_height
        st.session_state.safety_radius = safety_radius
        
        # 绕行策略
        bypass_options = {
            "left": "⬅️ 向左绕行",
            "right": "➡️ 向右绕行",
            "best": "⭐ 最佳航线（自动选择）"
        }
        
        selected_bypass = st.radio(
            "绕行策略",
            options=list(bypass_options.keys()),
            format_func=lambda x: bypass_options[x],
            index=list(bypass_options.keys()).index(st.session_state.bypass_strategy),
            key="bypass_strategy_radio"
        )
        st.session_state.bypass_strategy = selected_bypass
        
        # 规划航线按钮
        if st.button("🚀 开始规划航线", type="primary", use_container_width=True):
            with st.spinner("正在规划航线..."):
                route_points, analysis = plan_route()
                st.success(f"✅ 航线规划完成！总距离: {analysis['total_distance']:.1f}m")
                st.rerun()
    
    with right_col:
        # 航线分析报告
        st.subheader("📊 航线分析报告")
        
        if st.session_state.route_analysis:
            analysis = st.session_state.route_analysis
            
            st.metric("📏 总飞行距离", f"{analysis.get('total_distance', 0):.1f} m")
            
            col_a, col_b = st.columns(2)
            with col_a:
                st.metric("🔄 绕行次数", analysis.get('bypass_count', 0))
            with col_b:
                st.metric("✈️ 飞跃次数", analysis.get('fly_over_count', 0))
            
            st.divider()
            
            st.caption("📋 障碍物处理详情")
            for i, obs in enumerate(analysis.get('obstacles_encountered', [])):
                st.caption(f"障碍物 {i+1}: {obs['height']}m → {obs['decision']}")
            
            # 显示航点信息
            if analysis.get('route_points'):
                st.divider()
                st.caption(f"📍 规划航点数: {len(analysis['route_points'])}")
        else:
            st.info("点击「开始规划航线」生成航线分析")
        
        st.divider()
        
        # 障碍物管理
        st.subheader("⛔ 障碍物管理")
        
        # 新障碍物高度预设
        st.caption("绘制新障碍物时预设高度:")
        new_obs_height = st.number_input("障碍物高度(m)", value=10, step=5, key="new_obstacle_height")
        
        # 显示当前障碍物列表
        if st.session_state.obstacles:
            st.caption(f"共 {len(st.session_state.obstacles)} 个障碍物")
            for idx, obs in enumerate(st.session_state.obstacles):
                col_del, col_info = st.columns([1, 4])
                with col_del:
                    if st.button("❌", key=f"del_{idx}"):
                        remove_obstacle(idx)
                        st.rerun()
                with col_info:
                    obs_height = obs.get('height', 10)
                    st.caption(f"障碍物 {idx+1}: {len(obs['points'])}点 | 高{obs_height}m")
        else:
            st.info("暂无障碍物")
        
        st.divider()
        
        # 配置持久化按钮
        col_save, col_load, col_clear = st.columns(3)
        with col_save:
            if st.button("💾 保存", use_container_width=True):
                save_obstacles_to_file()
                st.success("已保存")
        with col_load:
            if st.button("📂 加载", use_container_width=True):
                if load_obstacles_from_file():
                    st.success(f"加载 {len(st.session_state.obstacles)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("无配置文件")
        with col_clear:
            if st.button("🗑️ 清空", use_container_width=True):
                clear_all_obstacles()
                st.rerun()
        
        if st.button("🔄 刷新地图", use_container_width=True):
            st.rerun()
        
        st.divider()
        st.caption(f"📁 配置: {OBSTACLE_FILE}")

if __name__ == "__main__":
    main()
