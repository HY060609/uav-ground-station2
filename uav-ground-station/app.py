"""
无人机地面站系统 - 智能任务规划平台
功能：心跳包、地图显示、GCJ-02坐标转换、障碍物多边形圈选、持久化存储
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
        st.session_state.start_point = {"lat": 32.2323, "lng": 118.749}
    if 'end_point' not in st.session_state:
        st.session_state.end_point = {"lat": 32.2344, "lng": 118.749}
    if 'flight_height' not in st.session_state:
        st.session_state.flight_height = 10
    if 'map_center' not in st.session_state:
        st.session_state.map_center = [32.2333, 118.749]
    if 'drawn_polygons' not in st.session_state:
        st.session_state.drawn_polygons = []  # 存储绘制的多边形数据

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
            # 转换坐标格式: [lng, lat] -> [lat, lng] (GCJ-02存储)
            points = []
            for coord in coords:
                # 从地图获取的是WGS84，需要转换为GCJ-02存储
                gcj_lat, gcj_lng = wgs84_to_gcj02(coord[1], coord[0])
                points.append([gcj_lat, gcj_lng])
            
            # 去重（首尾相同）
            if len(points) > 1 and points[0] == points[-1]:
                points = points[:-1]
            
            st.session_state.obstacles.append({
                "points": points,
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

# ==================== 地图创建（带绘图工具）====================
def create_map():
    """创建带绘图工具的 Folium 地图"""
    # 转换起点和终点坐标
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
    
    # 创建地图
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
    
    # 添加绘图工具（多边形圈选）
    draw = Draw(
        draw_options={
            'polygon': True,      # 启用多边形绘制
            'polyline': False,
            'rectangle': False,
            'circle': False,
            'marker': False,
            'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    draw.add_to(m)
    
    # 添加起点标记
    folium.Marker(
        location=[start_wgs[0], start_wgs[1]],
        popup=f"起点A (GCJ-02)",
        icon=folium.Icon(color='green', icon='play', prefix='fa'),
        tooltip="起点A"
    ).add_to(m)
    
    # 添加终点标记
    folium.Marker(
        location=[end_wgs[0], end_wgs[1]],
        popup=f"终点B (GCJ-02)",
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'),
        tooltip="终点B"
    ).add_to(m)
    
    # 绘制航线
    folium.PolyLine(
        locations=[[start_wgs[0], start_wgs[1]], [end_wgs[0], end_wgs[1]]],
        color='blue',
        weight=3,
        opacity=0.8,
        popup=f"航线 | 高度: {st.session_state.flight_height}m"
    ).add_to(m)
    
    # 绘制已保存的障碍物多边形
    for idx, obstacle in enumerate(st.session_state.obstacles):
        wgs_points = []
        for point in obstacle["points"]:
            wgs = gcj02_to_wgs84(float(point[0]), float(point[1]))
            wgs_points.append([wgs[0], wgs[1]])
        
        folium.Polygon(
            locations=wgs_points,
            color='red',
            weight=2,
            fill=True,
            fill_color='red',
            fill_opacity=0.3,
            popup=f"障碍物 {idx + 1}"
        ).add_to(m)
    
    return m

# ==================== 主界面 ====================
def main():
    # 标题栏
    st.title("✈️ 无人机智能化应用系统")
    st.caption("魏坤的《无人机智能化应用2451》 | 分组作业4-项目Demo")
    
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
    
    # 左右两栏布局
    left_col, right_col = st.columns([2, 1])
    
    with left_col:
        st.subheader("🗺️ 地图显示 (OpenStreetMap)")
        st.caption("📍 使用左侧工具栏的【多边形】按钮在地图上圈选障碍物 | 坐标系: GCJ-02 → WGS84")
        
        # 创建并显示地图
        try:
            m = create_map()
            # 获取地图交互数据（包括绘制的图形）
            output = st_folium(
                m, 
                width=800, 
                height=500,
                returned_objects=["last_active_drawing", "all_drawings"]
            )
            
            # 处理新绘制的多边形
            if output and output.get("last_active_drawing"):
                feature = output["last_active_drawing"]
                if feature.get("geometry", {}).get("type") == "Polygon":
                    if add_obstacle_from_draw(feature):
                        st.success("✅ 障碍物已添加！")
                        st.rerun()
                        
        except Exception as e:
            st.error(f"地图加载出错: {e}")
            st.info("请刷新页面重试")
        
        # 地图操作说明
        with st.expander("📖 地图操作说明"):
            st.markdown("""
            - **缩放**: 鼠标滚轮
            - **移动**: 拖拽地图
            - **圈选障碍物**: 点击地图左上角的【多边形】图标 ✏️
            - **绘制多边形**: 在地图上依次点击顶点，双击完成绘制
            - **切换图层**: 点击右上角图层按钮
            """)
    
    with right_col:
        # 控制面板
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
            if st.button("设置A点", use_container_width=True):
                st.session_state.start_point = {"lat": float(start_lat), "lng": float(start_lng)}
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
            if st.button("设置B点", use_container_width=True):
                st.session_state.end_point = {"lat": float(end_lat), "lng": float(end_lng)}
                st.success(f"终点已设置")
                st.rerun()
        
        # 飞行参数
        with st.expander("⚙️ 飞行参数", expanded=True):
            flight_h = st.number_input(
                "设定飞行高度 (m)", 
                value=st.session_state.flight_height,
                step=5,
                key="flight_height_input"
            )
            if st.button("更新参数", use_container_width=True):
                st.session_state.flight_height = flight_h
                st.success(f"飞行高度已设为 {flight_h}m")
        
        # 障碍物管理
        st.subheader("⛔ 障碍物管理")
        
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
                    st.caption(f"障碍物 {idx+1}: {len(obs['points'])} 个顶点 | {obs['created_at']}")
        else:
            st.info("暂无障碍物，请在地图上使用多边形工具圈选")
        
        st.divider()
        
        # 障碍物配置持久化按钮
        col_save, col_load, col_clear = st.columns(3)
        with col_save:
            if st.button("💾 保存到文件", use_container_width=True):
                save_obstacles_to_file()
                st.success(f"已保存到 {OBSTACLE_FILE}")
        with col_load:
            if st.button("📂 从文件加载", use_container_width=True):
                if load_obstacles_from_file():
                    st.success(f"加载成功，共 {len(st.session_state.obstacles)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("未找到配置文件")
        with col_clear:
            if st.button("🗑️ 清除全部", use_container_width=True):
                clear_all_obstacles()
                st.success("已清除所有障碍物")
                st.rerun()
        
        # 一键部署按钮
        if st.button("🚀 一键部署", type="primary", use_container_width=True):
            st.success("部署完成！地图已更新")
            st.rerun()
        
        # 配置文件状态
        st.divider()
        st.caption(f"📁 配置文件: {OBSTACLE_FILE}")
        if os.path.exists(OBSTACLE_FILE):
            with open(OBSTACLE_FILE, 'r', encoding='utf-8') as f:
                try:
                    data = json.load(f)
                    st.caption(f"✅ 共 {len(data.get('obstacles', []))} 个障碍物 | 版本 v12.2")
                except:
                    st.caption(f"✅ 文件存在 | 版本 v12.2")
        else:
            st.caption("⚠️ 暂无配置文件")

if __name__ == "__main__":
    main()