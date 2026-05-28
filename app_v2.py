"""
无人机地面站系统 - v8 (修复起终点持久化 - 文件存储版)
核心改进：
- 放弃不稳定的 st.query_params，改用文件存储起终点和飞行参数
- 左绕：只用障碍物缓冲区左侧顶点构建可见性图
- 右绕：只用右侧顶点
- 最佳：左右都算，取路径最短的
- 贴边走：路径紧贴安全缓冲区边缘
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

# ==================== 文件持久化 ====================
OBS_FILE = "obstacles.json"
WP_FILE = "waypoints.json"      # 存储起终点和飞行参数

def _wp_save():
    """保存起终点和飞行参数到文件"""
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
    """从文件加载起终点和飞行参数"""
    if os.path.exists(WP_FILE):
        try:
            with open(WP_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return None

def _obs_save():
    try:
        with open(OBS_FILE, "w", encoding="utf-8") as f:
            json.dump(st.session_state.obstacles, f, ensure_ascii=False, indent=2)
    except Exception:
        pass

def _obs_load():
    if os.path.exists(OBS_FILE):
        try:
            with open(OBS_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return []

# ==================== 默认值 ====================
DEFAULT_A  = {"lat": 32.232945, "lng": 118.746956, "height": 0}
DEFAULT_B  = {"lat": 32.235204, "lng": 118.751589, "height": 0}
DEFAULT_FH = 50
DEFAULT_SR = 8

# ==================== Session State ====================
def init_session_state():
    if "inited" in st.session_state:
        return
    # 加载起终点和飞行参数
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
    st.session_state.setting_mode         = None
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

# ==================== GCJ-02 <-> WGS-84 ====================
_AE, _EE, _PI = 6378245.0, 0.00669342162296594323, math.pi

def _ooc(lat, lng):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

def _tlat(lng, lat):
    r  = -100+2*lng+3*lat+0.2*lat**2+0.1*lng*lat+0.2*math.sqrt(abs(lng))
    r += (20*math.sin(6*lng*_PI)+20*math.sin(2*lng*_PI))*2/3
    r += (20*math.sin(lat*_PI)+40*math.sin(lat/3*_PI))*2/3
    r += (160*math.sin(lat/12*_PI)+320*math.sin(lat*_PI/30))*2/3
    return r

def _tlng(lng, lat):
    r  = 300+lng+2*lat+0.1*lng**2+0.1*lng*lat+0.1*math.sqrt(abs(lng))
    r += (20*math.sin(6*lng*_PI)+20*math.sin(2*lng*_PI))*2/3
    r += (20*math.sin(lng*_PI)+40*math.sin(lng/3*_PI))*2/3
    r += (150*math.sin(lng/12*_PI)+300*math.sin(lng/30*_PI))*2/3
    return r

def _delta(lat, lng):
    dl=_tlat(lng-105,lat-35); dg=_tlng(lng-105,lat-35)
    mg=math.sin(lat*_PI/180); mg=1-_EE*mg*mg; sq=math.sqrt(mg)
    dl=dl*180/((_AE*(1-_EE))/(mg*sq)*_PI)
    dg=dg*180/(_AE/sq*math.cos(lat*_PI/180)*_PI)
    return dl,dg

def gcj2wgs(lat,lng):
    if _ooc(lat,lng): return float(lat),float(lng)
    dl,dg=_delta(lat,lng); return float(lat-dl),float(lng-dg)

def wgs2gcj(lat,lng):
    if _ooc(lat,lng): return float(lat),float(lng)
    dl,dg=_delta(lat,lng); return float(lat+dl),float(lng+dg)

# ==================== 米制投影 ====================
def get_ref():
    ss=st.session_state
    return ((ss.start_point["lat"]+ss.end_point["lat"])/2,
            (ss.start_point["lng"]+ss.end_point["lng"])/2)

def ll2m(lat,lng,rl,rg):
    return ((lng-rg)*math.cos(math.radians(rl))*111320,(lat-rl)*111320)

def m2ll(x,y,rl,rg):
    return (y/111320+rl, x/(math.cos(math.radians(rl))*111320)+rg)

def hdist(lat1,lng1,lat2,lng2):
    R=6371000; f1,f2=math.radians(lat1),math.radians(lat2)
    dp=math.radians(lat2-lat1); dl=math.radians(lng2-lng1)
    a=math.sin(dp/2)**2+math.cos(f1)*math.cos(f2)*math.sin(dl/2)**2
    return R*2*math.atan2(math.sqrt(a),math.sqrt(1-a))

def plen(path):
    return sum(hdist(path[i][0],path[i][1],path[i+1][0],path[i+1][1]) for i in range(len(path)-1))

# ==================== 安全缓冲区 ====================
def build_union(obstacles, fh, sr, rl, rg):
    polys=[]
    for obs in obstacles:
        if obs.get("height",30)<=fh: continue
        pts=obs.get("points",[])
        if len(pts)<3: continue
        try:
            xy=[ll2m(p[0],p[1],rl,rg) for p in pts]
            poly=Polygon(xy)
            if not poly.is_valid: poly=poly.buffer(0)
            if poly.is_valid and poly.area>0:
                polys.append(poly.buffer(float(sr)))
        except: continue
    if not polys: return None
    u=unary_union(polys)
    return u if not u.is_empty else None

# ==================== 线段安全检测 ====================
def seg_free(ax,ay,bx,by,union_geom,margin=0.15):
    seg=LineString([(ax,ay),(bx,by)])
    L=seg.length
    if L<margin*2:
        return not Point((ax+bx)/2,(ay+by)/2).within(union_geom)
    p1=seg.interpolate(margin)
    p2=seg.interpolate(L-margin)
    return not LineString([p1,p2]).intersects(union_geom)

# ==================== 方向感知 Visibility Graph ====================
def _side_of_line(px,py,ax,ay,bx,by):
    return (bx-ax)*(py-ay)-(by-ay)*(px-ax)

def extract_nodes_sided(union_geom, sx,sy,ex,ey, side):
    nodes=[(sx,sy),(ex,ey)]
    geoms=[union_geom] if union_geom.geom_type=='Polygon' else \
          [g for g in union_geom.geoms if g.geom_type=='Polygon']
    for g in geoms:
        for coord in list(g.exterior.coords)[:-1]:
            cx,cy=coord
            s=_side_of_line(cx,cy,sx,sy,ex,ey)
            if side=='left'  and s>-0.1: nodes.append((cx,cy))
            elif side=='right' and s< 0.1: nodes.append((cx,cy))
            elif side=='both': nodes.append((cx,cy))
    # 去重
    seen=set(); unique=[]
    for n in nodes:
        k=(round(n[0],3),round(n[1],3))
        if k not in seen: seen.add(k); unique.append(n)
    return unique

def dijkstra_path(nodes, union_geom):
    n=len(nodes)
    if n<2: return None
    adj=[[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1,n):
            ax,ay=nodes[i]; bx,by=nodes[j]
            if seg_free(ax,ay,bx,by,union_geom):
                d=math.hypot(bx-ax,by-ay)
                adj[i].append((j,d)); adj[j].append((i,d))
    INF=float('inf'); dist=[INF]*n; prev=[-1]*n; dist[0]=0.0
    heap=[(0.0,0)]
    while heap:
        cost,u=heapq.heappop(heap)
        if cost>dist[u]: continue
        if u==1: break
        for v,w in adj[u]:
            nc=cost+w
            if nc<dist[v]:
                dist[v]=nc; prev[v]=u; heapq.heappush(heap,(nc,v))
    if dist[1]==INF: return None
    path=[]; cur=1
    while cur!=-1: path.append(cur); cur=prev[cur]
    path.reverse()
    return [nodes[i] for i in path]

def smooth_path(path_m, union_geom):
    if len(path_m)<=2: return path_m
    out=[path_m[0]]; i=0
    while i<len(path_m)-1:
        j=len(path_m)-1
        while j>i+1:
            if seg_free(*path_m[i],*path_m[j],union_geom): break
            j-=1
        out.append(path_m[j]); i=j
    return out

def push_outside(px,py,toward_x,toward_y,union_geom):
    if not union_geom.contains(Point(px,py)): return px,py
    vl=math.hypot(toward_x-px,toward_y-py) or 1.0
    vx=(toward_x-px)/vl; vy=(toward_y-py)/vl
    for d in range(1,120):
        npx=px-vx*d; npy=py-vy*d
        if not union_geom.contains(Point(npx,npy)): return npx,npy
    return px,py

def plan_one_side(sx,sy,ex,ey,union_geom,side):
    sx2,sy2=push_outside(sx,sy,ex,ey,union_geom)
    ex2,ey2=push_outside(ex,ey,sx,sy,union_geom)
    nodes=extract_nodes_sided(union_geom,sx2,sy2,ex2,ey2,side)
    if len(nodes)<2: return None
    path_m=dijkstra_path(nodes,union_geom)
    if not path_m or len(path_m)<2: return None
    path_m=smooth_path(path_m,union_geom)
    path_m[0]=(sx,sy); path_m[-1]=(ex,ey)
    return path_m

def _fallback_bypass(sx,sy,ex,ey,union):
    L=math.hypot(ex-sx,ey-sy)
    if L<1e-6: return [(sx,sy),(ex,ey)]
    dx,dy=(ex-sx)/L,(ey-sy)/L
    best,best_l=None,float('inf')
    for px,py in [(-dy,dx),(dy,-dx)]:
        max_proj=0.0
        try:
            geoms=[union] if union.geom_type=='Polygon' else list(union.geoms)
            for g in geoms:
                for c in (g.exterior.coords if g.geom_type=='Polygon' else []):
                    proj=(c[0]-sx)*px+(c[1]-sy)*py
                    if proj>max_proj: max_proj=proj
        except: max_proj=30.0
        off=max_proj+10.0
        for _ in range(12):
            cand=[(sx,sy),(sx+dx*L*0.33+px*off,sy+dy*L*0.33+py*off),
                          (sx+dx*L*0.67+px*off,sy+dy*L*0.67+py*off),(ex,ey)]
            if all(seg_free(cand[k][0],cand[k][1],cand[k+1][0],cand[k+1][1],union)
                   for k in range(3)):
                tl=sum(math.hypot(cand[k+1][0]-cand[k][0],cand[k+1][1]-cand[k][1]) for k in range(3))
                if tl<best_l: best_l=tl; best=cand
                break
            off+=15.0
    return best or [(sx,sy),(ex,ey)]

def plan_route_core(start_gcj, end_gcj, obstacles, fh, sr, strategy):
    rl,rg=get_ref()
    sx,sy=ll2m(start_gcj[0],start_gcj[1],rl,rg)
    ex,ey=ll2m(end_gcj[0],  end_gcj[1],  rl,rg)

    analysis={
        "total_distance":0,"obstacles_encountered":[],"bypass_count":0,
        "fly_over_count":0,"route_points":[],"strategy_used":"","waypoint_count":0,
    }
    for obs in obstacles:
        h=obs.get("height",30)
        if h>fh: analysis["obstacles_encountered"].append({"height":h,"decision":"绕行"})
        else:
            analysis["fly_over_count"]+=1
            analysis["obstacles_encountered"].append({"height":h,"decision":"飞跃"})

    union=build_union(obstacles,fh,sr,rl,rg)

    if union is None:
        path=[start_gcj,end_gcj]
        analysis.update(total_distance=hdist(*start_gcj,*end_gcj),
                        strategy_used="直线（无需绕行）",waypoint_count=2,route_points=path)
        return path,analysis

    direct=LineString([(sx,sy),(ex,ey)])
    if not direct.intersects(union):
        path=[start_gcj,end_gcj]
        analysis.update(total_distance=hdist(*start_gcj,*end_gcj),
                        strategy_used="直线（不碰障碍物）",waypoint_count=2,route_points=path)
        return path,analysis

    path_m=None
    strat_name=""

    if strategy=="left":
        path_m=plan_one_side(sx,sy,ex,ey,union,"left")
        strat_name="左侧绕行"
    elif strategy=="right":
        path_m=plan_one_side(sx,sy,ex,ey,union,"right")
        strat_name="右侧绕行"
    else:  # best
        pm_l=plan_one_side(sx,sy,ex,ey,union,"left")
        pm_r=plan_one_side(sx,sy,ex,ey,union,"right")
        len_l=sum(math.hypot(pm_l[i+1][0]-pm_l[i][0],pm_l[i+1][1]-pm_l[i][1])
                  for i in range(len(pm_l)-1)) if pm_l else float('inf')
        len_r=sum(math.hypot(pm_r[i+1][0]-pm_r[i][0],pm_r[i+1][1]-pm_r[i][1])
                  for i in range(len(pm_r)-1)) if pm_r else float('inf')
        if pm_l and len_l<=len_r:
            path_m=pm_l; strat_name=f"最佳（左侧更短 {len_l:.0f}m vs {len_r:.0f}m）"
        elif pm_r:
            path_m=pm_r; strat_name=f"最佳（右侧更短 {len_r:.0f}m vs {len_l:.0f}m）"

    if path_m is None:
        path_m=plan_one_side(sx,sy,ex,ey,union,"both")
        strat_name+="（全方向兜底）"

    if path_m is None:
        path_m=_fallback_bypass(sx,sy,ex,ey,union)
        strat_name="简单偏移兜底"

    path_gcj=[m2ll(x,y,rl,rg) for x,y in path_m]
    nbp=max(0,len(path_gcj)-2)
    analysis.update(total_distance=plen(path_gcj),bypass_count=nbp,
                    waypoint_count=len(path_gcj),
                    strategy_used=f"{strat_name}（{nbp}个绕行点）",
                    route_points=path_gcj)
    return path_gcj,analysis

def plan_route():
    ss=st.session_state
    start=(ss.start_point["lat"],ss.start_point["lng"])
    end  =(ss.end_point["lat"],  ss.end_point["lng"])
    path,analysis=plan_route_core(start,end,ss.obstacles,
                                   ss.flight_height,ss.safety_radius,
                                   ss.bypass_strategy)
    ss.planned_route =path
    ss.route_analysis=analysis
    ss.map_key+=1
    return path,analysis

# ==================== 飞行控制 ====================
def reset_flight():
    ss=st.session_state
    ss.auto_flight_enabled=False; ss.flight_paused=False
    ss.flight_progress=0.0; ss.current_waypoint_idx=0
    ss.flight_remaining_dist=ss.route_analysis.get("total_distance",0)
    ss.flight_battery=100; ss.flight_time_elapsed=0; ss.mission_started=False
    ss.flight_drone_pos=ss.planned_route[0] if ss.planned_route else None

def add_log(direction,message):
    ts=datetime.now().strftime("%H:%M:%S")
    st.session_state.comm_logs.insert(0,{"time":ts,"direction":direction,"message":message})
    if len(st.session_state.comm_logs)>100: st.session_state.comm_logs.pop()

def step_forward():
    ss=st.session_state; route=ss.planned_route
    if not route: return
    if ss.current_waypoint_idx>=len(route)-1:
        ss.auto_flight_enabled=False; add_log("FCU→OBC→GCS","MISSION_COMPLETE"); return
    ss.link_delay=random.randint(20,35); ss.link_loss=round(random.uniform(0.05,0.25),2)
    ss.current_waypoint_idx+=1
    ss.flight_progress=ss.current_waypoint_idx/(len(route)-1)
    ss.flight_drone_pos=route[ss.current_waypoint_idx]
    add_log("FCU→OBC→GCS",f"WP_REACHED #{ss.current_waypoint_idx+1}")
    ss.flight_remaining_dist=sum(hdist(route[i][0],route[i][1],route[i+1][0],route[i+1][1])
                                 for i in range(ss.current_waypoint_idx,len(route)-1))
    total=ss.route_analysis.get("total_distance",1)
    if total>0: ss.flight_time_elapsed=int((ss.flight_progress*total)/ss.flight_speed)
    ss.flight_battery=max(0,100-ss.flight_progress*5)

# ==================== 地图 ====================
def create_map():
    ss=st.session_state
    sw=gcj2wgs(ss.start_point["lat"],ss.start_point["lng"])
    ew=gcj2wgs(ss.end_point["lat"],  ss.end_point["lng"])
    clat=(sw[0]+ew[0])/2; clng=(sw[1]+ew[1])/2

    m=folium.Map(location=[clat,clng],zoom_start=18,
                 tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                 attr='Esri World Imagery',control_scale=True,prefer_canvas=True)
    plugins.Draw(
        draw_options={'polygon':{'allowIntersection':False,'showArea':True,
                                 'shapeOptions':{'color':'#ff3333','fillOpacity':0.35}},
                      'rectangle':{'shapeOptions':{'color':'#ff3333','fillOpacity':0.35}},
                      'polyline':False,'circle':False,'marker':False,'circlemarker':False},
        edit_options={'edit':True,'remove':True}
    ).add_to(m)
    plugins.MeasureControl(position='bottomleft',primary_length_unit='meters').add_to(m)

    folium.Marker(sw,popup="起点A",tooltip="起点 A",
                  icon=folium.Icon(color='green',icon='play',prefix='fa')).add_to(m)
    folium.Marker(ew,popup="终点B",tooltip="终点 B",
                  icon=folium.Icon(color='red',icon='flag-checkered',prefix='fa')).add_to(m)

    rl,rg=get_ref()
    for idx,obs in enumerate(ss.obstacles):
        pts=obs["points"]; wpts=[gcj2wgs(p[0],p[1]) for p in pts]
        h=obs.get("height",10); high=h>ss.flight_height
        fc="#ff2222" if high else "#ff9900"; bc="#cc0000" if high else "#cc7700"
        folium.Polygon(locations=wpts,color=bc,weight=2,fill=True,
                       fill_color=fc,fill_opacity=0.55,
                       popup=f"障碍物{idx+1}|{'⛔绕行' if high else '✅飞越'} {h}m",
                       tooltip=f"障碍物{idx+1}|{h}m").add_to(m)
        if high:
            try:
                xy=[ll2m(p[0],p[1],rl,rg) for p in wpts]
                buf=Polygon(xy).buffer(float(ss.safety_radius))
                if buf.geom_type=='Polygon':
                    bp=[m2ll(x,y,rl,rg) for x,y in buf.exterior.coords]
                    folium.Polygon(locations=bp,color='#ffdd00',weight=1.5,
                                   dash_array='6,4',fill=True,fill_color='#ffdd00',
                                   fill_opacity=0.10,tooltip="安全缓冲区").add_to(m)
            except: pass
        cl=sum(p[0] for p in wpts)/len(wpts); cg=sum(p[1] for p in wpts)/len(wpts)
        folium.map.Marker([cl,cg],icon=folium.DivIcon(
            html=f'<div style="background:rgba(0,0,0,.75);color:#fff;font-size:11px;'
                 f'font-weight:bold;padding:2px 6px;border-radius:4px;'
                 f'border:1px solid {fc};white-space:nowrap;">↑{h}m</div>',
            icon_size=(58,22),icon_anchor=(29,11))).add_to(m)

    route=ss.planned_route
    in_flight=ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
    if route:
        rwgs=[gcj2wgs(p[0],p[1]) for p in route]; wpi=ss.current_waypoint_idx
        if not in_flight:
            folium.PolyLine(rwgs,color='#1e90ff',weight=4,opacity=0.9,
                            dash_array='12,8',tooltip="规划航线（待飞）").add_to(m)
            for i,pt in enumerate(rwgs):
                if i==0 or i==len(rwgs)-1: continue
                folium.CircleMarker(pt,radius=6,color='#1e90ff',weight=2,
                                    fill=True,fill_color='white',fill_opacity=0.9,
                                    tooltip=f"绕行航点 {i}").add_to(m)
        else:
            if wpi>=1:
                folium.PolyLine(rwgs[:wpi+1],color='#00dd44',weight=5,opacity=1.0,
                                tooltip="已飞轨迹").add_to(m)
                for i in range(1,wpi):
                    folium.CircleMarker(rwgs[i],radius=5,color='#00aa33',weight=2,
                                        fill=True,fill_color='#00ff55',fill_opacity=1.0).add_to(m)
            if wpi<len(rwgs)-1:
                folium.PolyLine(rwgs[wpi:],color='#1e90ff',weight=3,opacity=0.75,
                                dash_array='10,7',tooltip="待飞航线").add_to(m)
                for i in range(wpi+1,len(rwgs)-1):
                    folium.CircleMarker(rwgs[i],radius=5,color='#1e90ff',weight=2,
                                        fill=True,fill_color='white',fill_opacity=0.85).add_to(m)
            if ss.flight_drone_pos:
                dw=gcj2wgs(ss.flight_drone_pos[0],ss.flight_drone_pos[1])
                folium.CircleMarker(dw,radius=22,color='#ff8c00',weight=1,
                                    fill=True,fill_color='#ff8c00',fill_opacity=0.18).add_to(m)
                folium.Marker(dw,tooltip="🚁 无人机当前位置",
                              icon=folium.DivIcon(
                    html='<div style="width:32px;height:32px;background:#ff6600;'
                         'border:3px solid white;border-radius:50%;display:flex;'
                         'align-items:center;justify-content:center;font-size:16px;'
                         'box-shadow:0 0 8px rgba(255,102,0,.7);">✈</div>',
                    icon_size=(32,32),icon_anchor=(16,16))).add_to(m)
    return m

# ==================== 通信日志 ====================
def render_comm_logs():
    ss=st.session_state
    st.markdown("---")
    st.markdown("### 📡 通信链路拓扑与数据流")
    c1,c2,c3=st.columns(3)
    with c1: st.markdown('<span style="color:#4CAF50;">✅ 🖥️ GCS 在线</span>',unsafe_allow_html=True)
    with c2: st.markdown('<span style="color:#4CAF50;">✅ 🧠 OBC 在线</span>',unsafe_allow_html=True)
    with c3: st.markdown('<span style="color:#4CAF50;">✅ ⚙️ FCU 在线</span>',unsafe_allow_html=True)
    good=ss.link_delay<50
    st.markdown(f"""
<style>
.lrow{{display:flex;align-items:center;gap:10px;background:#f5f7fa;border-radius:12px;padding:12px 16px;margin:8px 0;flex-wrap:wrap;}}
.lcard{{border-radius:10px;padding:8px 14px;text-align:center;min-width:90px;background:white;}}
.lcard.gcs{{border:2px solid #2196F3;}}.lcard.obc{{border:2px solid #FF9800;}}.lcard.fcu{{border:2px solid #9C27B0;}}
.lb{{font-weight:bold;font-size:14px;}}.ls{{font-size:10px;color:#888;}}
.larr{{text-align:center;font-size:12px;color:#555;}}.lstat{{font-size:11px;color:#4CAF50;}}
</style>
<div class="lrow">
  <div class="lcard gcs"><div>🖥️</div><div class="lb">GCS</div><div class="ls">地面站</div><div class="ls">192.168.1.100</div></div>
  <div class="larr"><div>↑↓</div><div class="lstat">UDP:14550</div><div class="lstat">● 已连接</div></div>
  <div class="lcard obc"><div>🧠</div><div class="lb">OBC</div><div class="ls">机载计算机</div><div class="ls">Raspberry Pi 4</div></div>
  <div class="larr"><div>↑↓</div><div class="lstat">MAVLink</div><div class="lstat">● 已连接</div></div>
  <div class="lcard fcu"><div>⚙️</div><div class="lb">FCU</div><div class="ls">飞控</div><div class="ls">PX4/ArduPilot</div></div>
</div>
<p style="font-size:12px;color:#555;margin:4px 0;">
📊 <b>链路统计：</b>GCS↔OBC:{"正常" if good else "延迟高"} OBC↔FCU:{"正常" if good else "延迟高"}
延迟:~{ss.link_delay}ms 丢包率:{ss.link_loss}%</p>
""",unsafe_allow_html=True)
    st.markdown("---"); st.markdown("### 📋 通信日志")
    logs=ss.comm_logs
    g2f=[l for l in logs if l["direction"]=="GCS→OBC→FCU"]
    f2g=[l for l in logs if l["direction"]=="FCU→OBC→GCS"]
    t1,t2,t3=st.tabs(["🔄 业务流程","📤 GCS→OBC→FCU","📥 FCU→OBC→GCS"])

    with t1:
        with st.container(height=250):
            biz=[l for l in logs if any(k in l["message"] for k in ["规划","MISSION","START","SET_","任务"])]
            nav=[l for l in logs if "WP_REACHED" in l["message"]]
            wpc=len(ss.planned_route); rl2=ss.route_analysis.get("total_distance",0)
            sp=ss.start_point; ep=ss.end_point
            if biz:
                st.markdown('<span style="color:#4CAF50;font-weight:bold;">✅ 航线规划</span>',unsafe_allow_html=True)
                for l in biz[:5]:
                    st.markdown(f'<div style="background:#f0fff4;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                                f'[{l["time"]}] 航线规划完成 | 航点数:{wpc} | 路径:{rl2:.1f}m'
                                f'<br><span style="color:#888;font-size:11px;">🔵 OBC 内部</span></div>',unsafe_allow_html=True)
            if nav:
                st.markdown('<span style="color:#2196F3;font-weight:bold;">ℹ️ 导航目标</span>',unsafe_allow_html=True)
                for l in nav[:3]:
                    st.markdown(f'<div style="background:#e3f2fd;border-radius:5px;padding:4px 8px;margin:2px 0;font-size:12px;">'
                                f'[{l["time"]}] 起:({sp["lat"]:.5f},{sp["lng"]:.5f})→终:({ep["lat"]:.5f},{ep["lng"]:.5f}) | {ss.flight_height}m'
                                f'<br><span style="color:#888;font-size:11px;">🟢 GCS→🔵 OBC</span></div>',unsafe_allow_html=True)
            if not biz and not nav:
                if logs:
                    h='<div style="font-size:12px;font-family:monospace;line-height:1.9;">'
                    for l in logs[:10]:
                        bg="#f0fff4" if "GCS" in l["direction"] else "#fff8e1"
                        h+=f'<div style="background:{bg};border-radius:4px;padding:2px 6px;margin:2px 0;">[{l["time"]}] {l["direction"]}: <b>{l["message"]}</b></div>'
                    st.markdown(h+'</div>',unsafe_allow_html=True)
                else:
                    st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>',unsafe_allow_html=True)

    with t2:
        with st.container(height=250):
            if not g2f:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>',unsafe_allow_html=True)
            else:
                h='<div style="font-size:12px;font-family:monospace;line-height:2;">'
                h+='<div style="color:#e65100;font-weight:bold;margin-bottom:4px;">📤 GCS→OBC→FCU</div>'
                for l in g2f:
                    h+=(f'<div style="border-bottom:1px solid #eee;padding:2px 0;">'
                        f'<span style="color:#888;">[{l["time"]}]</span> '
                        f'<span style="color:#e65100;">GCS→OBC→FCU:</span> <b>{l["message"]}</b></div>')
                st.markdown(h+'</div>',unsafe_allow_html=True)

    with t3:
        with st.container(height=250):
            if not f2g:
                st.markdown('<span style="color:#aaa;font-size:13px;">暂无日志</span>',unsafe_allow_html=True)
            else:
                h='<div style="background:#fff8e1;border-radius:6px;padding:8px;font-size:12px;font-family:monospace;margin-bottom:8px;">'
                h+='<div style="color:#e65100;font-weight:bold;margin-bottom:4px;">📥 FCU→OBC</div>'
                for l in f2g:
                    h+=(f'<div style="border-bottom:1px dashed #eee;padding:2px 0;">'
                        f'<span style="color:#888;">[{l["time"]}]</span> FCU→OBC→GCS: <b>{l["message"]}</b></div>')
                h+='</div><div style="background:#fff8e1;border-radius:6px;padding:8px;font-size:12px;font-family:monospace;">'
                h+='<div style="color:#7B1FA2;font-weight:bold;margin-bottom:4px;">📤 OBC→GCS</div>'
                for l in f2g:
                    h+=(f'<div style="border-bottom:1px dashed #eee;padding:2px 0;">'
                        f'<span style="color:#888;">[{l["time"]}]</span> FCU→OBC→GCS: <b>{l["message"]}</b></div>')
                st.markdown(h+'</div>',unsafe_allow_html=True)

# ==================== 飞行监控 ====================
def render_flight_monitor():
    ss=st.session_state
    st.markdown("### ✈️ 飞行实时画面 - 任务执行监控")
    c1,c2,c3,c4=st.columns(4)
    with c1:
        if st.button("▶ 开始任务",type="primary",use_container_width=True,disabled=ss.auto_flight_enabled):
            if not ss.planned_route: st.error("请先规划航线！")
            else:
                reset_flight(); ss.auto_flight_enabled=True; ss.mission_started=True
                add_log("GCS→OBC→FCU","START_MISSION | Mode: AUTO"); add_log("业务流程","任务开始"); st.rerun()
    with c2:
        if st.button("⏸️ 暂停",use_container_width=True,disabled=not ss.auto_flight_enabled or ss.flight_paused):
            ss.flight_paused=True; ss.auto_flight_enabled=False; add_log("GCS→OBC→FCU","PAUSE"); st.rerun()
    with c3:
        if st.button("⏹️ 停止",use_container_width=True,disabled=not(ss.auto_flight_enabled or ss.flight_paused)):
            ss.auto_flight_enabled=False; ss.flight_paused=False; add_log("GCS→OBC→FCU","STOP"); st.rerun()
    with c4:
        if st.button("🔄 重置",use_container_width=True): reset_flight(); st.rerun()
    if ss.auto_flight_enabled and not ss.flight_paused:
        if ss.planned_route and ss.current_waypoint_idx<len(ss.planned_route)-1:
            step_forward(); st_autorefresh(interval=350,key="afr")
        else:
            ss.auto_flight_enabled=False; st.success("✅ 已到达终点！任务完成")
    twp=len(ss.planned_route) if ss.planned_route else 0; cwp=ss.current_waypoint_idx+1
    c1,c2,c3,c4,c5,c6=st.columns(6)
    with c1: st.metric("当前航点",f"{cwp}/{twp}")
    with c2: st.metric("飞行速度",f"{ss.flight_speed:.1f} m/s")
    with c3:
        mv,sv=ss.flight_time_elapsed//60,ss.flight_time_elapsed%60
        st.metric("已用时间",f"{mv:02d}:{sv:02d}")
    with c4: st.metric("剩余距离",f"{ss.flight_remaining_dist:.0f} m")
    with c5:
        eta=int(ss.flight_remaining_dist/ss.flight_speed) if ss.flight_speed>0 else 0
        st.metric("预计到达",f"{eta//60:02d}:{eta%60:02d}")
    with c6:
        b=ss.flight_battery
        st.metric("电量模拟",f"{'🟢' if b>50 else '🟡' if b>20 else '🔴'} {b:.0f}%")
    st.progress(ss.flight_progress,text=f"任务进度:{ss.flight_progress*100:.1f}% | {cwp}/{twp} 航点")

# ==================== 主函数 ====================
def main():
    st.set_page_config(page_title="无人机地面站 v8",layout="wide",page_icon="✈️")
    st.title("✈️ 无人机地面站系统")
    st.caption("卫星实况地图 | 方向感知 Visibility Graph 最短路径 | 实时飞行监控")
    ss=st.session_state
    ss.heartbeat_count+=1
    hb={"seq":ss.heartbeat_count,"ts":datetime.now().strftime("%H:%M:%S"),
        "bat":random.randint(85,100),"sig":random.randint(70,99)}
    c1,c2,c3,c4,c5=st.columns(5)
    with c1: st.metric("💓 心跳","在线")
    with c2: st.metric("📡 序列",hb["seq"])
    with c3: st.metric("🔋 电量",f"{hb['bat']}%")
    with c4: st.metric("📶 信号",f"{hb['sig']}%")
    with c5: st.metric("🕐 时间",hb["ts"])
    st.divider()
    render_flight_monitor()
    st.divider()

    left,mid,right=st.columns([2,1,1])

    with left:
        st.subheader("🛰️ 实时飞行地图（卫星）")
        mc1,mc2,mc3,mc4,mc5,mc6=st.columns(6)
        with mc1:
            if st.button("📍 起点A",use_container_width=True): ss.setting_mode="start"
        with mc2:
            if st.button("🏁 终点B",use_container_width=True): ss.setting_mode="end"
        with mc3:
            if st.button("❌ 取消", use_container_width=True): ss.setting_mode=None
        with mc4:
            if st.button("⬅️ 左绕行",use_container_width=True):
                ss.bypass_strategy="left"; plan_route(); st.rerun()
        with mc5:
            if st.button("➡️ 右绕行",use_container_width=True):
                ss.bypass_strategy="right"; plan_route(); st.rerun()
        with mc6:
            if st.button("🌟 最佳航线",type="primary",use_container_width=True):
                ss.bypass_strategy="best"; plan_route(); st.rerun()

        if ss.setting_mode=="start": st.info("🔵 点击地图设置起点A")
        elif ss.setting_mode=="end": st.info("🔴 点击地图设置终点B")

        in_f=ss.auto_flight_enabled or ss.flight_paused or ss.mission_started
        strat_display={"left":"⬅️ 左绕行","right":"➡️ 右绕行","best":"🌟 最佳航线"}.get(ss.bypass_strategy,"")
        st.caption(f"当前策略：{strat_display} | " +
                   ("🟢已飞=绿实线|🔵待飞=蓝虚线|🟠无人机=橙圆" if in_f else
                    "🔵规划=蓝虚线|🔴高障碍=红|🟠低障碍=橙|黄虚线=安全缓冲区"))

        try:
            mp=create_map()
            out=st_folium(mp,width=820,height=560,
                          key=f"map_{ss.map_key}",
                          returned_objects=["last_active_drawing","last_clicked"])
            if out and out.get("last_clicked"):
                ck=out["last_clicked"]
                if ck and "lat" in ck and "lng" in ck:
                    gl,gg=wgs2gcj(ck["lat"],ck["lng"])
                    if ss.setting_mode=="start":
                        ss.start_point={"lat":gl,"lng":gg,"height":0}; ss.setting_mode=None
                        add_log("GCS→OBC→FCU",f"SET_START ({gl:.5f},{gg:.5f})")
                        _wp_save(); plan_route(); st.rerun()
                    elif ss.setting_mode=="end":
                        ss.end_point={"lat":gl,"lng":gg,"height":0}; ss.setting_mode=None
                        add_log("GCS→OBC→FCU",f"SET_END ({gl:.5f},{gg:.5f})")
                        _wp_save(); plan_route(); st.rerun()
            if out and out.get("last_active_drawing"):
                feat=out["last_active_drawing"]
                if feat.get("geometry",{}).get("type")=="Polygon":
                    coords=feat['geometry']['coordinates'][0]; pts=[]
                    for c in coords:
                        gl2,gg2=wgs2gcj(c[1],c[0]); pts.append([gl2,gg2])
                    if len(pts)>1 and pts[0]==pts[-1]: pts=pts[:-1]
                    if len(pts)>=3:
                        h=ss.new_obstacle_height
                        ss.obstacles.append({"points":pts,"height":h,
                                             "created_at":datetime.now().strftime("%Y-%m-%d %H:%M:%S")})
                        _obs_save(); add_log("业务流程",f"障碍物已添加|高度:{h}m")
                        st.success("✅ 障碍物已添加，正在重新规划航线…")
                        plan_route(); st.rerun()
        except Exception as e:
            st.error(f"地图错误: {e}")
            import traceback; st.code(traceback.format_exc())
        render_comm_logs()

    with mid:
        st.subheader("🎮 控制面板")
        with st.expander("📍 起点 A",expanded=True):
            sl=st.number_input("纬度",value=float(ss.start_point["lat"]),format="%.6f",key="sl")
            sg=st.number_input("经度",value=float(ss.start_point["lng"]),format="%.6f",key="sg")
            if abs(sl-ss.start_point["lat"])>1e-8 or abs(sg-ss.start_point["lng"])>1e-8:
                ss.start_point={"lat":sl,"lng":sg,"height":0}; _wp_save(); plan_route(); st.rerun()
        with st.expander("🏁 终点 B",expanded=True):
            el=st.number_input("纬度",value=float(ss.end_point["lat"]),format="%.6f",key="el")
            eg=st.number_input("经度",value=float(ss.end_point["lng"]),format="%.6f",key="eg")
            if abs(el-ss.end_point["lat"])>1e-8 or abs(eg-ss.end_point["lng"])>1e-8:
                ss.end_point={"lat":el,"lng":eg,"height":0}; _wp_save(); plan_route(); st.rerun()
        st.divider()
        st.subheader("✈️ 飞行参数")
        fh=st.number_input("飞行高度 (m)",value=int(ss.flight_height),step=5,min_value=10,max_value=200)
        if fh!=ss.flight_height:
            ss.flight_height=fh; _wp_save(); plan_route(); st.rerun()
        sr=st.number_input("安全半径 (m)",value=int(ss.safety_radius),step=1,min_value=3,max_value=50)
        if sr!=ss.safety_radius:
            ss.safety_radius=sr; _wp_save(); plan_route(); st.rerun()
        spd=st.slider("飞行速度 (m/s)",1.0,20.0,value=float(ss.flight_speed),step=0.5)
        if abs(spd-ss.flight_speed)>0.01: ss.flight_speed=spd
        st.divider()
        st.subheader("⛔ 新障碍物高度")
        st.number_input("障碍物高度 (m)",value=60,step=5,min_value=10,max_value=200,key="new_obstacle_height")
        st.caption("💡 在地图上用多边形工具绘制障碍物区域")
        st.divider()
        with st.expander("📐 算法说明"):
            st.markdown("""
**方向感知 Visibility Graph + Dijkstra**

| 按钮 | 逻辑 |
|------|------|
| ⬅️ 左绕行 | 只取障碍物左侧轮廓顶点构图，Dijkstra 最短左侧路径 |
| ➡️ 右绕行 | 只取右侧顶点，Dijkstra 最短右侧路径 |
| 🌟 最佳航线 | 左右都算，选总距离更短的那条 |

三条路径**真正不同**，贴着障碍物安全缓冲区边缘走，不绕远。
""")

    with right:
        st.subheader("📊 航线分析")
        if ss.route_analysis:
            a=ss.route_analysis
            st.metric("📏 总距离",  f"{a.get('total_distance',0):.1f} m")
            st.metric("📍 总航点数",a.get('waypoint_count',0))
            st.metric("🔄 绕行节点",a.get('bypass_count',0))
            st.metric("✅ 飞越次数",a.get('fly_over_count',0))
            st.metric("🎯 规划策略",a.get('strategy_used','未知'))
            if a.get("obstacles_encountered"):
                st.divider(); st.caption("📋 障碍物处理")
                for obs in a["obstacles_encountered"]:
                    st.text(f"{'🔄' if '绕行' in obs['decision'] else '✅'} {obs['height']}m → {obs['decision']}")
        else:
            st.info("点击规划按钮生成报告")
        st.divider()
        st.subheader("⛔ 障碍物列表")
        if ss.obstacles:
            st.caption(f"共 {len(ss.obstacles)} 个障碍物")
            for idx,obs in enumerate(ss.obstacles):
                ca,cb,cc=st.columns([1,2,1])
                with ca:
                    if st.button("🗑️",key=f"d{idx}"):
                        ss.obstacles.pop(idx); _obs_save(); plan_route(); st.rerun()
                with cb:
                    st.text(f"障碍 {idx+1}")
                    if obs.get('created_at'): st.caption(obs['created_at'][:10])
                with cc:
                    h_=obs.get('height',10)
                    st.text(f"{'🔴' if h_>ss.flight_height else '🟠'} {h_}m")
        else:
            st.info("暂无障碍物\n在地图绘制多边形添加")
        st.divider()
        cs,cl,cc2=st.columns(3)
        with cs:
            if st.button("💾 保存",use_container_width=True):
                _obs_save(); _wp_save(); st.success("已保存")
        with cl:
            if st.button("📂 加载",use_container_width=True):
                ss.obstacles=_obs_load()
                wp_data=_wp_load()
                if wp_data:
                    if "start_point" in wp_data: ss.start_point=wp_data["start_point"]
                    if "end_point" in wp_data:   ss.end_point=wp_data["end_point"]
                    if "flight_height" in wp_data: ss.flight_height=wp_data["flight_height"]
                    if "safety_radius" in wp_data: ss.safety_radius=wp_data["safety_radius"]
                st.success(f"已加载 {len(ss.obstacles)} 个障碍物及起终点"); plan_route(); st.rerun()
        with cc2:
            if st.button("🗑️ 清空",use_container_width=True):
                ss.obstacles=[]; _obs_save(); plan_route(); st.rerun()

if __name__=="__main__":
    main()
