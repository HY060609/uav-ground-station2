"""
无人机地面站系统 - v4
修改要点：
1. 地图只保留 Esri World Imagery 卫星实况图，去掉所有其他图层切换
2. 规划阶段：蓝色虚线 + 蓝色航点圆圈（未飞）
3. 飞行阶段：已飞段显示绿色实线，未飞段保留蓝色虚线，无人机橙色图标在当前位置
4. 通信日志三标签页样式
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
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from streamlit_autorefresh import st_autorefresh
 
# ==================== 配置常量 ====================
DEFAULT_A_GCJ = [118.746956, 32.232945]   # [lng, lat]
DEFAULT_B_GCJ  = [118.751589, 32.235204]
CONFIG_FILE    = "obstacle_config.json"
DEFAULT_SAFETY_RADIUS_METERS = 8
 
# ==================== Session State 初始化 ====================
def init_session_state():
    defaults = {
        'heartbeat_count':      0,
        'obstacles':            [],
        'start_point':          {"lat": DEFAULT_A_GCJ[1], "lng": DEFAULT_A_GCJ[0], "height": 0},
        'end_point':            {"lat": DEFAULT_B_GCJ[1], "lng": DEFAULT_B_GCJ[0], "height": 0},
        'flight_height':        50,
        'safety_radius':        DEFAULT_SAFETY_RADIUS_METERS,
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
        'mission_started':      False,   # 任务是否已开始过（控制线条样式）
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v
 
init_session_state()
 
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
 
# ==================== 绕行算法 ====================
def build_safe_union(obstacles, flight_height, safety_radius, ref_lat, ref_lng):
    polys = []
    for obs in obstacles:
        if obs.get("height", 30) <= flight_height: continue
        pts = obs.get("points", [])
        if len(pts) < 3: continue
        xy = [latlon_to_meters(p[0], p[1], ref_lat, ref_lng) for p in pts]
        try:
            poly = Polygon(xy)
            if not poly.is_valid: poly = poly.buffer(0)
            polys.append(poly.buffer(safety_radius + 3))
        except: continue
    return unary_union(polys) if polys else None
 
def path_clear(path_m, union_safe):
    for i in range(len(path_m)-1):
        if LineString([path_m[i], path_m[i+1]]).intersects(union_safe): return False
    return True
 
def compute_bypass(start_gcj, end_gcj, obstacles, fh, sr, direction):
    ref_lat, ref_lng = get_ref_point()
    sx, sy = latlon_to_meters(start_gcj[0], start_gcj[1], ref_lat, ref_lng)
    ex, ey = latlon_to_meters(end_gcj[0],   end_gcj[1],   ref_lat, ref_lng)
    union  = build_safe_union(obstacles, fh, sr, ref_lat, ref_lng)
    if union is None or union.is_empty: return [start_gcj, end_gcj], True
    orig = LineString([(sx,sy),(ex,ey)])
    if not orig.intersects(union): return [start_gcj, end_gcj], True
    length = math.hypot(ex-sx, ey-sy)
    if length < 1e-6: return [start_gcj, end_gcj], False
    dx, dy = (ex-sx)/length, (ey-sy)/length
    px, py = (-dy, dx) if direction=='left' else (dy, -dx)
    inter  = orig.intersection(union)
    if inter.is_empty: return [start_gcj, end_gcj], True
    pts = []
    def _collect(geom):
        try:
            for c in geom.coords:
                t = (c[0]-sx)*dx + (c[1]-sy)*dy
                pts.append((t, c[0], c[1]))
        except: pass
    if inter.geom_type == 'GeometryCollection':
        for g in inter.geoms: _collect(g)
    else: _collect(inter)
    if not pts: pts = [(0,sx,sy),(length,ex,ey)]
    pts.sort()
    margin = 5.0
    te = max(0, pts[0][0]-margin);  ex2 = sx+dx*te;  ey2 = sy+dy*te
    tx = min(length, pts[-1][0]+margin); xx2 = sx+dx*tx; xy2 = sy+dy*tx
    # 最大投影距离
    max_proj = 0
    try:
        coords = list(union.exterior.coords) if union.geom_type=='Polygon' else \
                 [c for g in union.geoms for c in g.exterior.coords]
        for c in coords:
            proj = (c[0]-sx)*px + (c[1]-sy)*py
            if proj > max_proj: max_proj = proj
    except: max_proj = sr*2
    offset   = max(max_proj+5, sr+10)
    def make(off):
        return [(sx,sy),(ex2+px*off,ey2+py*off),(xx2+px*off,xy2+py*off),(ex,ey)]
    path_m = make(offset)
    for _ in range(6):
        if path_clear(path_m, union): break
        offset += 10; path_m = make(offset)
    return [meters_to_latlon(mx,my,ref_lat,ref_lng) for mx,my in path_m], True
 
# ==================== 规划航线 ====================
def plan_route():
    start = (st.session_state.start_point["lat"], st.session_state.start_point["lng"])
    end   = (st.session_state.end_point["lat"],   st.session_state.end_point["lng"])
    fh    = st.session_state.flight_height
    sr    = st.session_state.safety_radius
    strat = st.session_state.bypass_strategy
    obs   = st.session_state.obstacles
 
    analysis = {"total_distance":0,"obstacles_encountered":[],"bypass_count":0,
                "fly_over_count":0,"route_points":[],"strategy_used":strat}
 
    ref_lat, ref_lng = get_ref_point()
    union = build_safe_union(obs, fh, sr, ref_lat, ref_lng)
    sx,sy = latlon_to_meters(start[0],start[1],ref_lat,ref_lng)
    ex,ey = latlon_to_meters(end[0],  end[1],  ref_lat,ref_lng)
    blocked = bool(union and not union.is_empty and LineString([(sx,sy),(ex,ey)]).intersects(union))
 
    for o in obs:
        h = o.get("height",30)
        if h > fh: analysis["obstacles_encountered"].append({"height":h,"decision":"绕行" if blocked else "未挡路"})
        else:      analysis["fly_over_count"]+=1; analysis["obstacles_encountered"].append({"height":h,"decision":"飞跃(低)"})
 
    if not blocked:
        route = [start, end]
        analysis["total_distance"] = haversine(*start,*end)
        analysis["strategy_used"]  = "直线（无阻挡）"
        analysis["route_points"]   = route
        st.session_state.planned_route  = route
        st.session_state.route_analysis = analysis
        st.session_state.map_key += 1
        return route, analysis
 
    if strat == "left":
        path,_ = compute_bypass(start,end,obs,fh,sr,'left');  analysis["strategy_used"]="左绕行"; analysis["bypass_count"]=1
    elif strat == "right":
        path,_ = compute_bypass(start,end,obs,fh,sr,'right'); analysis["strategy_used"]="右绕行"; analysis["bypass_count"]=1
    else:
        pl,_ = compute_bypass(start,end,obs,fh,sr,'left')
        pr,_ = compute_bypass(start,end,obs,fh,sr,'right')
        if path_length(pl) <= path_length(pr):
            path=pl; analysis["strategy_used"]="最佳(左绕行)"
        else:
            path=pr; analysis["strategy_used"]="最佳(右绕行)"
        analysis["bypass_count"] = 1
 
    analysis["total_distance"] = path_length(path)
    analysis["route_points"]   = path
    st.session_state.planned_route  = path
    st.session_state.route_analysis = analysis
    st.session_state.map_key += 1
    return path, analysis
 
# ==================== 飞行控制 ====================
def reset_flight():
    ss = st.session_state
    ss.auto_flight_enabled  = False
    ss.flight_paused        = False
    ss.flight_progress      = 0.0
    ss.current_waypoint_idx = 0
    ss.flight_remaining_dist= ss.route_analysis.get("total_distance",0)
    ss.flight_battery       = 100
    ss.flight_drone_pos     = ss.planned_route[0] if ss.planned_route else None
    ss.flight_time_elapsed  = 0
    ss.mission_started      = False
 
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
        add_comm_log("FCU→OBC→GCS", "MISSION_COMPLETE")
        return
    ss.link_delay           = random.randint(20,35)
    ss.link_loss            = round(random.uniform(0.05,0.25),2)
    ss.current_waypoint_idx += 1
    ss.flight_progress      = ss.current_waypoint_idx / (len(route)-1)
    ss.flight_drone_pos     = route[ss.current_waypoint_idx]
    wp = ss.current_waypoint_idx+1
    add_comm_log("FCU→OBC→GCS", f"WP_REACHED #{wp}")
    remaining = sum(haversine(route[i][0],route[i][1],route[i+1][0],route[i+1][1])
                    for i in range(ss.current_waypoint_idx, len(route)-1))
    ss.flight_remaining_dist = remaining
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
 
    # ---- 只用 Esri 卫星图，无图层切换 ----
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=18,
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri World Imagery',
        control_scale=True,
        prefer_canvas=True
    )
 
    # 绘图工具（障碍物圈选）
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
 
    # 比例尺
    plugins.MeasureControl(position='bottomleft', primary_length_unit='meters').add_to(m)
 
    # ---- 起终点标记 ----
    folium.Marker(
        [start_wgs[0], start_wgs[1]],
        popup=f"起点A ({ss.start_point['lat']:.5f}, {ss.start_point['lng']:.5f})",
        icon=folium.Icon(color='green', icon='play', prefix='fa'),
        tooltip="起点 A"
    ).add_to(m)
    folium.Marker(
        [end_wgs[0], end_wgs[1]],
        popup=f"终点B ({ss.end_point['lat']:.5f}, {ss.end_point['lng']:.5f})",
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'),
        tooltip="终点 B"
    ).add_to(m)
 
    # ---- 障碍物 ----
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
 
        # 安全缓冲区虚线
        try:
            rl = sum(p[0] for p in wgs_pts)/len(wgs_pts)
            rg = sum(p[1] for p in wgs_pts)/len(wgs_pts)
            pm = Polygon([latlon_to_meters(p[0],p[1],rl,rg) for p in wgs_pts])
            bm = pm.buffer(ss.safety_radius+3)
            if bm.geom_type=='Polygon':
                bp = [meters_to_latlon(x,y,rl,rg) for x,y in bm.exterior.coords]
                folium.Polygon(locations=bp,color='#ffff00',weight=1,dash_array='5,4',
                               fill=True,fill_color='#ffff00',fill_opacity=0.08).add_to(m)
        except: pass
 
        # 高度标注
        cl = sum(p[0] for p in wgs_pts)/len(wgs_pts)
        cg = sum(p[1] for p in wgs_pts)/len(wgs_pts)
        folium.map.Marker([cl,cg], icon=folium.DivIcon(
            html=f'<div style="background:rgba(0,0,0,.72);color:#fff;font-size:11px;'
                 f'font-weight:bold;padding:2px 6px;border-radius:4px;'
                 f'border:1px solid {fc};white-space:nowrap;">↑{h}m</div>',
            icon_size=(58,22), icon_anchor=(29,11)
        )).add_to(m)
 
    # ---- 航线绘制：根据任务状态改变线型 ----
    route = ss.planned_route
    if route:
        route_wgs = [gcj02_to_wgs84(p[0],p[1]) for p in route]
        wp_idx    = ss.current_waypoint_idx          # 当前已到达的航点下标
        in_flight = ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
 
        if not in_flight:
            # ===== 规划阶段：全程蓝色虚线 =====
            folium.PolyLine(
                locations=route_wgs,
                color='#1e90ff', weight=4, opacity=0.9,
                dash_array='12,8',
                tooltip="规划航线（待飞）"
            ).add_to(m)
            # 蓝色航点圆圈
            for i, pt in enumerate(route_wgs):
                if i==0 or i==len(route_wgs)-1: continue
                folium.CircleMarker(
                    location=pt, radius=6,
                    color='#1e90ff', weight=2,
                    fill=True, fill_color='white', fill_opacity=0.9,
                    tooltip=f"航点 {i}"
                ).add_to(m)
 
        else:
            # ===== 飞行阶段 =====
            # 已飞段：绿色实线（从起点到当前位置）
            if wp_idx >= 1:
                flown = route_wgs[:wp_idx+1]
                folium.PolyLine(
                    locations=flown,
                    color='#00dd44', weight=5, opacity=1.0,
                    tooltip="已飞轨迹"
                ).add_to(m)
                # 绿色已飞航点
                for i in range(1, wp_idx):
                    folium.CircleMarker(
                        location=route_wgs[i], radius=5,
                        color='#00aa33', weight=2,
                        fill=True, fill_color='#00ff55', fill_opacity=1.0,
                        tooltip=f"已过航点 {i}"
                    ).add_to(m)
 
            # 未飞段：蓝色虚线（从当前位置到终点）
            if wp_idx < len(route_wgs)-1:
                remaining_seg = route_wgs[wp_idx:]
                folium.PolyLine(
                    locations=remaining_seg,
                    color='#1e90ff', weight=3, opacity=0.75,
                    dash_array='10,7',
                    tooltip="待飞航线"
                ).add_to(m)
                # 未飞航点蓝色圆圈
                for i in range(wp_idx+1, len(route_wgs)-1):
                    folium.CircleMarker(
                        location=route_wgs[i], radius=5,
                        color='#1e90ff', weight=2,
                        fill=True, fill_color='white', fill_opacity=0.85,
                        tooltip=f"待飞航点 {i}"
                    ).add_to(m)
 
        # ---- 无人机当前位置（飞行中才显示）----
        if ss.flight_drone_pos and in_flight:
            drone_wgs = gcj02_to_wgs84(ss.flight_drone_pos[0], ss.flight_drone_pos[1])
            # 光晕
            folium.CircleMarker(
                location=drone_wgs, radius=22,
                color='#ff8c00', weight=1,
                fill=True, fill_color='#ff8c00', fill_opacity=0.18
            ).add_to(m)
            # 无人机图标（橙色）
            folium.Marker(
                drone_wgs,
                icon=folium.DivIcon(
                    html='''<div style="
                        width:32px;height:32px;
                        background:#ff6600;
                        border:3px solid white;
                        border-radius:50%;
                        display:flex;align-items:center;justify-content:center;
                        font-size:16px;
                        box-shadow:0 0 8px rgba(255,102,0,.7);
                    ">✈</div>''',
                    icon_size=(32,32), icon_anchor=(16,16)
                ),
                tooltip="🚁 无人机当前位置"
            ).add_to(m)
 
    return m
 
# ==================== 通信日志渲染 ====================
def render_comm_logs():
    ss = st.session_state
    st.markdown("---")
    st.markdown("### 📡 通信链路拓扑与数据流")
 
    c1,c2,c3 = st.columns(3)
    with c1: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🖥️ GCS 在线</span>',unsafe_allow_html=True)
    with c2: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ 🧠 OBC 在线</span>',unsafe_allow_html=True)
    with c3: st.markdown('<span style="color:#4CAF50;font-size:15px;">✅ ⚙️ FCU 在线</span>',unsafe_allow_html=True)
 
    st.markdown(f"""
<style>
.lrow{{display:flex;align-items:center;gap:12px;background:#f5f7fa;border-radius:12px;padding:14px 18px;margin:10px 0;flex-wrap:wrap;}}
.lcard{{border-radius:10px;padding:10px 18px;text-align:center;min-width:96px;background:white;}}
.lcard.gcs{{border:2px solid #2196F3;}}.lcard.obc{{border:2px solid #FF9800;}}.lcard.fcu{{border:2px solid #9C27B0;}}
.lb{{font-weight:bold;font-size:15px;}}.ls{{font-size:11px;color:#888;}}
.larr{{text-align:center;font-size:13px;color:#555;}}
.lstat{{font-size:12px;color:#4CAF50;}}
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
 
    # 三标签页日志
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
                # FCU → OBC 块
                html  = '<div style="background:#fff8e1;border-radius:6px;padding:8px;font-size:12px;font-family:monospace;margin-bottom:8px;">'
                html += '<div style="color:#e65100;font-weight:bold;margin-bottom:4px;">📥 FCU → OBC</div>'
                for log in fcu2gcs_logs:
                    html += (f'<div style="border-bottom:1px dashed #ece;padding:2px 0;">'
                             f'<span style="color:#888;">[{log["time"]}]</span> FCU→OBC→GCS: <b>{log["message"]}</b></div>')
                html += '</div>'
                # OBC → GCS 块
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
                ss.auto_flight_enabled = True
                ss.mission_started     = True
                add_comm_log("GCS→OBC→FCU", "START_MISSION | Mode: AUTO")
                add_comm_log("业务流程",     "任务开始")
                st.rerun()
    with c2:
        if st.button("⏸️ 暂停", use_container_width=True,
                     disabled=not ss.auto_flight_enabled or ss.flight_paused):
            ss.flight_paused       = True
            ss.auto_flight_enabled = False
            add_comm_log("GCS→OBC→FCU", "PAUSE")
            st.rerun()
    with c3:
        if st.button("⏹️ 停止", use_container_width=True,
                     disabled=not (ss.auto_flight_enabled or ss.flight_paused)):
            ss.auto_flight_enabled = False
            ss.flight_paused       = False
            add_comm_log("GCS→OBC→FCU", "STOP")
            st.rerun()
    with c4:
        if st.button("🔄 重置", use_container_width=True):
            reset_flight()
            st.rerun()
 
    # 自动推进
    if ss.auto_flight_enabled and not ss.flight_paused:
        route = ss.planned_route
        if route and ss.current_waypoint_idx < len(route)-1:
            step_forward()
            st_autorefresh(interval=350, key="afr")
        else:
            ss.auto_flight_enabled = False
            st.success("✅ 已到达终点！任务完成")
 
    # 指标行
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
        load_obstacles()
        st.session_state.obstacles_loaded = True
 
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
    st.set_page_config(page_title="无人机地面站 v4", layout="wide", page_icon="✈️")
 
    # 标题
    st.title("✈️ 无人机地面站系统")
    st.caption("卫星实况地图 | 实时飞行监控 | 智能绕行规划")
 
    auto_load_obstacles()
    hb = heartbeat()
 
    # 心跳状态栏
    c1,c2,c3,c4,c5 = st.columns(5)
    with c1: st.metric("💓 心跳","在线")
    with c2: st.metric("📡 序列号",hb["sequence"])
    with c3: st.metric("🔋 电量",f"{hb['battery']}%")
    with c4: st.metric("📶 信号",f"{hb['signal']}%")
    with c5: st.metric("🕐 时间",hb["timestamp"])
    st.divider()
 
    render_flight_monitor()
    st.divider()
 
    left_col, mid_col, right_col = st.columns([2,1,1])
 
    # ==================== 左栏：地图 ====================
    with left_col:
        st.subheader("🛰️ 实时飞行地图（卫星）")
 
        mc1,mc2,mc3,mc4,mc5,mc6 = st.columns(6)
        with mc1:
            if st.button("📍 起点A", use_container_width=True):
                st.session_state.setting_mode="start"
        with mc2:
            if st.button("🏁 终点B", use_container_width=True):
                st.session_state.setting_mode="end"
        with mc3:
            if st.button("❌ 取消", use_container_width=True):
                st.session_state.setting_mode=None
        with mc4:
            if st.button("⬅️ 左绕行", use_container_width=True):
                st.session_state.bypass_strategy="left"; plan_route(); st.rerun()
        with mc5:
            if st.button("➡️ 右绕行", use_container_width=True):
                st.session_state.bypass_strategy="right"; plan_route(); st.rerun()
        with mc6:
            if st.button("🌟 最佳航线", type="primary", use_container_width=True):
                st.session_state.bypass_strategy="best"; plan_route(); st.rerun()
 
        if st.session_state.setting_mode=="start":   st.info("🔵 点击地图设置起点A")
        elif st.session_state.setting_mode=="end":   st.info("🔴 点击地图设置终点B")
 
        # 图例说明
        in_flight = st.session_state.auto_flight_enabled or st.session_state.flight_paused or st.session_state.mission_started
        if in_flight:
            st.caption("🟢 绿色实线=已飞轨迹 | 🔵 蓝色虚线=待飞航线 | 🟠 橙色图标=无人机当前位置")
        else:
            st.caption("🔵 蓝色虚线=规划航线（待飞）| 🔴 红色=高障碍物(绕行) | 🟠 橙色=低障碍物(飞越)")
 
        try:
            m      = create_map()
            output = st_folium(m, width=820, height=560,
                               key=f"map_{st.session_state.map_key}",
                               returned_objects=["last_active_drawing","last_clicked"])
 
            if output and output.get("last_clicked"):
                ck = output["last_clicked"]
                if ck and "lat" in ck and "lng" in ck:
                    glat,glng = wgs84_to_gcj02(ck["lat"],ck["lng"])
                    if st.session_state.setting_mode=="start":
                        st.session_state.start_point={"lat":glat,"lng":glng,"height":0}
                        st.session_state.setting_mode=None
                        add_comm_log("GCS→OBC→FCU",f"SET_START ({glat:.5f},{glng:.5f})")
                        plan_route(); st.rerun()
                    elif st.session_state.setting_mode=="end":
                        st.session_state.end_point={"lat":glat,"lng":glng,"height":0}
                        st.session_state.setting_mode=None
                        add_comm_log("GCS→OBC→FCU",f"SET_END ({glat:.5f},{glng:.5f})")
                        plan_route(); st.rerun()
 
            if output and output.get("last_active_drawing"):
                feat = output["last_active_drawing"]
                if feat.get("geometry",{}).get("type")=="Polygon":
                    if add_obstacle_from_draw(feat):
                        st.success("✅ 障碍物已添加，航线已重新规划")
                        plan_route(); st.rerun()
        except Exception as e:
            st.error(f"地图错误: {e}")
            import traceback; st.code(traceback.format_exc())
 
        # 通信日志（地图下方）
        render_comm_logs()
 
    # ==================== 中栏：控制面板 ====================
    with mid_col:
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
 
        sr = st.number_input("安全半径 (m)", value=st.session_state.safety_radius, step=1, min_value=5, max_value=50)
        if sr!=st.session_state.safety_radius: st.session_state.safety_radius=sr; plan_route(); st.rerun()
 
        spd = st.slider("飞行速度 (m/s)", 1.0, 20.0, value=st.session_state.flight_speed, step=0.5)
        if spd!=st.session_state.flight_speed: st.session_state.flight_speed=spd
 
        st.divider()
        st.subheader("⛔ 新障碍物高度")
        st.number_input("障碍物高度 (m)", value=60, step=5, min_value=10, max_value=200, key="new_obstacle_height")
        st.caption("💡 在地图上用多边形工具绘制障碍物区域")
 
    # ==================== 右栏：航线分析 + 障碍物列表 ====================
    with right_col:
        st.subheader("📊 航线分析")
        if st.session_state.route_analysis:
            a = st.session_state.route_analysis
            st.metric("📏 总距离",  f"{a.get('total_distance',0):.1f} m")
            st.metric("🔄 绕行次数", a.get('bypass_count',0))
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
                save_obstacles(); st.success("已保存")
        with cl:
            if st.button("📂 加载", use_container_width=True):
                ok,cnt = load_obstacles()
                if ok: st.success(f"加载{cnt}个"); plan_route(); st.rerun()
                else:  st.warning("无保存文件")
        with cc:
            if st.button("🗑️ 清空", use_container_width=True):
                clear_obstacles(); plan_route(); st.rerun()
 
if __name__ == "__main__":
    main()
 
