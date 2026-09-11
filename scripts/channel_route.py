#!/usr/bin/env python3
import json, math, os, subprocess, sys, xml.etree.ElementTree as ET
import pcbnew

root=sys.argv[1]
board_path=os.path.join(root,"pcbgolf.kicad_pcb")
pro_path=os.path.join(root,"pcbgolf.kicad_pro")
schematics=["pcbgolf.kicad_sch","pcbgolf_2.kicad_sch","pcbgolf_3.kicad_sch","pcbgolf_4.kicad_sch","pcbgolf_5.kicad_sch"]

MM=pcbnew.FromMM
TRACK=0.09
CLEAR=0.09
VIA_D=0.25
VIA_DRILL=0.15
PIN_PITCH=0.40
BUS_PITCH=0.30
CELL_GAP=1.0
ZONE_GAP=5.0
MARGIN=4.0
PAD_LANE_SEP=0.28

# export source netlists
nets={}
nodes=[]
for sch in schematics:
    out=os.path.join(root,sch+".xml")
    subprocess.run(["kicad-cli","sch","export","netlist","--format","kicadxml","-o",out,os.path.join(root,sch)],check=True)
    tree=ET.parse(out)
    for net in tree.findall(".//nets/net"):
        name=net.attrib.get("name","")
        if not name: continue
        nets.setdefault(name,[])
        for n in net.findall("node"):
            ref=n.attrib.get("ref"); pin=n.attrib.get("pin")
            if ref and pin:
                nets[name].append((ref,pin))
                nodes.append((name,ref,pin))

b=pcbnew.LoadBoard(board_path)
b.SetCopperLayerCount(6)
ds=b.GetDesignSettings()
for attr,val in [("m_AllowBlindBuriedVias",True),("m_AllowMicroVias",True)]:
    if hasattr(ds,attr): setattr(ds,attr,val)
for meth in ["SetAllowBlindBuriedVias","SetAllowMicroVias"]:
    if hasattr(ds,meth):
        try: getattr(ds,meth)(True)
        except Exception: pass
ds.SetBoardThickness(MM(0.8))

# wipe previous routing and outline
for t in list(b.GetTracks()): b.Remove(t)
for d in list(b.Drawings()):
    if d.GetLayer()==pcbnew.Edge_Cuts: b.Remove(d)

# ensure nets and forward annotate every schematic node
name_to_net={}
for name in sorted(nets):
    ni=b.FindNet(name)
    if not ni:
        ni=pcbnew.NETINFO_ITEM(b,name); b.Add(ni)
    name_to_net[name]=ni
fps={fp.GetReference():fp for fp in b.GetFootprints()}
assigned=0; missing=[]
for name, entries in nets.items():
    ni=name_to_net[name]
    for ref,pin in entries:
        fp=fps.get(ref)
        if not fp:
            missing.append((name,ref,pin,"ref")); continue
        pads=list(fp.Pads())
        matches=[p for p in pads if p.GetNumber()==str(pin)]
        if not matches:
            missing.append((name,ref,pin,"pad")); continue
        for pad in matches:
            pad.SetNet(ni)
            assigned += 1

# enlarge narrow SMD lands to >= 0.25 mm for HDI via-in-pad
for fp in b.GetFootprints():
    for pad in fp.Pads():
        if pad.GetNetCode()==0: continue
        try:
            if not pad.HasHole():
                sz=pad.GetSize()
                pad.SetSize(pcbnew.VECTOR2I(max(sz.x,MM(0.25)),max(sz.y,MM(0.25))))
        except Exception:
            pass

fan_layers=[b.GetLayerID("In1.Cu"),b.GetLayerID("In2.Cu"),b.GetLayerID("In3.Cu")]
vert_layer=b.GetLayerID("In4.Cu")
bus_layer=b.GetLayerID("B.Cu")
front_layer=b.GetLayerID("F.Cu")

def via_type_blind():
    return getattr(pcbnew,"VIATYPE_BLIND_BURIED",1)

def add_via(pos, net, l1, l2):
    v=pcbnew.PCB_VIA(b)
    v.SetPosition(pos)
    v.SetWidth(MM(VIA_D))
    v.SetDrill(MM(VIA_DRILL))
    v.SetViaType(via_type_blind())
    v.SetLayerPair(l1,l2)
    v.SetNet(net)
    b.Add(v)
    return v

def add_track(a,c,net,layer,width=TRACK):
    if a.x==c.x and a.y==c.y: return None
    t=pcbnew.PCB_TRACK(b)
    t.SetStart(a); t.SetEnd(c)
    t.SetWidth(MM(width)); t.SetLayer(layer); t.SetNet(net)
    b.Add(t)
    return t

def pad_positions(fp):
    return [p.GetPosition() for p in fp.Pads() if p.GetNetCode()!=0]

def depth_assignment(pads_or_points, sep=PAD_LANE_SEP):
    """Assign 1..N fanout depth. Left pads go deeper than nearby right pads,
    so a horizontal track never crosses a via barrel that reaches its layer."""
    pts=[]
    for idx,obj in enumerate(pads_or_points):
        if hasattr(obj,"GetPosition"):
            p=obj.GetPosition()
            pts.append((p.x,p.y,idx,obj))
        else:
            pts.append((obj.x,obj.y,idx,obj))
    # right to left; stable y/index tie-break makes equal-x close tracks use different layers
    order=sorted(pts,key=lambda q:(-q[0],q[1],q[2]))
    out={}
    done=[]
    for x,y,idx,obj in order:
        d=1
        for x2,y2,idx2,obj2,d2 in done:
            if abs(y-y2)<MM(sep) and (x2>x or (x2==x and idx2<idx)):
                d=max(d,d2+1)
        out[idx]=d
        done.append((x,y,idx,obj,d))
    return out,max(out.values(),default=1)

# choose an angle per footprint so <=3 horizontal fanout layers suffice
items=[]
for fp in b.GetFootprints():
    best=None
    for k in range(50,851,2): # 5.0..85.0 deg, 0.2 degree steps
        ang=k/10.0
        fp.SetOrientationDegrees(ang)
        pads=[p for p in fp.Pads() if p.GetNetCode()!=0]
        _,need=depth_assignment(pads) if len(pads)>1 else ({},1)
        bb=fp.GetBoundingBox()
        candidate=(need,bb.GetHeight(),bb.GetWidth(),ang)
        if best is None or candidate<best:
            best=candidate
            if need==1 and k>100:
                # still continue a little would only optimize height marginally
                pass
    fp.SetOrientationDegrees(best[3])
    bb=fp.GetBoundingBox()
    items.append({"fp":fp,"need":best[0],"angle":best[3],"h":bb.GetHeight(),"w":bb.GetWidth(),
                  "pads":sum(1 for p in fp.Pads() if p.GetNetCode()!=0)})

max_need=max(x["need"] for x in items)
if max_need>3:
    raise RuntimeError(f"fanout needs {max_need} layers")

# balanced 3-zone vertical placement. Each zone has its own x-local pin-channel block.
zones=[{"items":[],"h":0,"pins":0,"pins_by_depth":{1:0,2:0,3:0}} for _ in range(3)]
for it in sorted(items,key=lambda x:x["h"],reverse=True):
    z=min(zones,key=lambda z:z["h"])
    z["items"].append(it)
    z["h"] += it["h"] + MM(CELL_GAP)
    z["pins"] += it["pads"]
    # exact depth counts are recomputed after placement; reserve conservatively here

max_comp_w=max(it["w"] for it in items)+MM(2.0)
x_cursor=MM(MARGIN)
for zi,z in enumerate(zones):
    z["comp_left"]=x_cursor
    # reserve all pins; after placement we split this block by depth (3 nearest, 1 farthest)
    z["pin_start"]=x_cursor+max_comp_w+MM(2.0)
    z["pin_width"]=MM(PIN_PITCH)*max(z["pins"],1)+MM(2.0)
    z["right"]=z["pin_start"]+z["pin_width"]+MM(2.0)
    x_cursor=z["right"]+MM(ZONE_GAP)

# place each zone's footprints top-to-bottom
max_component_bottom=0
for z in zones:
    y=MM(MARGIN)
    for it in z["items"]:
        fp=it["fp"]
        fp.SetOrientationDegrees(it["angle"])
        bb=fp.GetBoundingBox()
        dx=z["comp_left"]-bb.GetLeft()
        dy=y-bb.GetTop()
        fp.Move(pcbnew.VECTOR2I(dx,dy))
        bb=fp.GetBoundingBox()
        it["top"]=bb.GetTop(); it["bottom"]=bb.GetBottom()
        y=bb.GetBottom()+MM(CELL_GAP)
        max_component_bottom=max(max_component_bottom,bb.GetBottom())
    z["bottom"]=y

# compute exact per-zone depth counts after final placement
for z in zones:
    z["pins_by_depth"]={1:0,2:0,3:0}
    for it in z["items"]:
        pads=[p for p in it["fp"].Pads() if p.GetNetCode()!=0]
        assign,need=depth_assignment(pads)
        if need>3:
            raise RuntimeError(f'{it["fp"].GetReference()} needs {need} fanout layers after placement')
        it["depth_by_index"]=assign
        for idx,p in enumerate(pads):
            z["pins_by_depth"][assign[idx]] += 1
    # deepest layer nearest component; shallowest farthest, preventing endpoint barrel crossings
    cur=z["pin_start"]
    z["pin_base"]={}
    for d in (3,2,1):
        z["pin_base"][d]=cur
        cur += z["pins_by_depth"][d]*MM(PIN_PITCH)+MM(0.5)

# assign one bus row per electrically multi-node net
net_pads={}
for fp in b.GetFootprints():
    for pad in fp.Pads():
        if pad.GetNetCode()!=0:
            net_pads.setdefault(pad.GetNetname(),[]).append(pad)
routed_net_names=[n for n,pads in sorted(net_pads.items()) if len(pads)>=2]
bus_start=max_component_bottom+MM(5.0)
bus_y={name:bus_start+i*MM(BUS_PITCH) for i,name in enumerate(routed_net_names)}

# route each zone
route_records=[]
via_count=0
track_count=0
for z in zones:
    pin_index={1:0,2:0,3:0}
    for it in sorted(z["items"],key=lambda x:x["top"]):
        fp=it["fp"]
        pads=[p for p in fp.Pads() if p.GetNetCode()!=0]
        assign,need=depth_assignment(pads)
        if need>3:
            raise RuntimeError(f"{fp.GetReference()} exceeded 3 fanout depths")
        for idx,pad in enumerate(pads):
            d=assign[idx]
            net=pad.GetNet()
            pos=pad.GetPosition()
            if len(net_pads.get(pad.GetNetname(),[]))<2:
                continue
            xpin=z["pin_base"][d]+pin_index[d]*MM(PIN_PITCH)
            pin_index[d]+=1
            p1=pcbnew.VECTOR2I(xpin,pos.y)
            p2=pcbnew.VECTOR2I(xpin,bus_y[pad.GetNetname()])
            layer=fan_layers[d-1]
            # PTH pads already connect every copper layer; only SMD pads need via-in-pad.
            has_hole=False
            try: has_hole=pad.HasHole()
            except Exception:
                try: has_hole=pad.GetDrillSize().x>0
                except Exception: has_hole=False
            if not has_hole:
                add_via(pos,net,front_layer,layer); via_count+=1
            add_track(pos,p1,net,layer); track_count+=1
            # endpoint depth blocks are ordered deepest-nearest, so skip barrels are never crossed.
            add_via(p1,net,layer,vert_layer); via_count+=1
            add_track(p1,p2,net,vert_layer); track_count+=1
            add_via(p2,net,vert_layer,bus_layer); via_count+=1
            route_records.append((pad.GetNetname(),p2))
    z["used_pins"]=sum(pin_index.values())

# bottom-layer horizontal bus per net
for name,pts in {}.items():
    pass
by_net={}
for name,p in route_records:
    by_net.setdefault(name,[]).append(p)
for name,pts in by_net.items():
    if len(pts)<2: continue
    xs=sorted(p.x for p in pts)
    add_track(pcbnew.VECTOR2I(xs[0],bus_y[name]),pcbnew.VECTOR2I(xs[-1],bus_y[name]),name_to_net[name],bus_layer)
    track_count+=1

# board outline tightly around all routing
xmin=MM(1.0)
xmax=max(z["right"] for z in zones)+MM(MARGIN)
ymin=MM(1.0)
ymax=(bus_start+max(1,len(routed_net_names))*MM(BUS_PITCH)+MM(MARGIN))
for a,c in [((xmin,ymin),(xmax,ymin)),((xmax,ymin),(xmax,ymax)),((xmax,ymax),(xmin,ymax)),((xmin,ymax),(xmin,ymin))]:
    sh=pcbnew.PCB_SHAPE(b)
    sh.SetShape(pcbnew.SHAPE_T_SEGMENT); sh.SetLayer(pcbnew.Edge_Cuts)
    sh.SetStart(pcbnew.VECTOR2I(*a)); sh.SetEnd(pcbnew.VECTOR2I(*c)); sh.SetWidth(MM(0.05)); b.Add(sh)

# project rules matching JLC HDI capability
try:
    with open(pro_path) as f: pro=json.load(f)
    rules=pro["board"]["design_settings"]["rules"]
    rules["min_clearance"]=0.0762
    rules["min_hole_clearance"]=0.09
    rules["min_track_width"]=0.08
    rules["min_via_diameter"]=0.20
    rules["min_via_annular_width"]=0.05
    rules["min_microvia_diameter"]=0.20
    rules["min_microvia_drill"]=0.10
    default_nc=pro["net_settings"]["classes"][0]
    default_nc.update({"clearance":TRACK,"track_width":TRACK,"via_diameter":VIA_D,"via_drill":VIA_DRILL,
                       "microvia_diameter":0.20,"microvia_drill":0.10})
    with open(pro_path,"w") as f: json.dump(pro,f,indent=2)
except Exception as e:
    print("project rule update warning",repr(e),file=sys.stderr)

b.BuildConnectivity()
pcbnew.SaveBoard(board_path,b)

report={
 "assigned_pad_nodes":assigned,
 "source_nodes":len(nodes),
 "missing_nodes":missing,
 "footprints":len(fps),
 "nets":len(nets),
 "electrical_pad_objects":sum(len(v) for v in net_pads.values()),
 "multi_pad_nets":len(routed_net_names),
 "fanout_max_layers_needed":max_need,
 "zones":[{"height_mm":pcbnew.ToMM(z["h"]),"pins":z["pins"],"used_pins":z["used_pins"],
           "width_mm":pcbnew.ToMM(z["right"]-z["comp_left"]),"pins_by_depth":z["pins_by_depth"]} for z in zones],
 "tracks_added":track_count,
 "vias_added":via_count,
 "board_mm":[pcbnew.ToMM(xmax-xmin),pcbnew.ToMM(ymax-ymin)],
 "rules_mm":{"track":TRACK,"clearance":CLEAR,"via_diameter":VIA_D,"via_drill":VIA_DRILL,
             "pin_pitch":PIN_PITCH,"bus_pitch":BUS_PITCH},
}
open(os.path.join(root,"channel_report.json"),"w").write(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
if missing:
    raise RuntimeError(f"missing {len(missing)} schematic nodes")
