#!/usr/bin/env python3
import json, os, subprocess, sys, xml.etree.ElementTree as ET
import pcbnew

root=sys.argv[1]
board_path=os.path.join(root,"pcbgolf.kicad_pcb")
schematics=[
 "pcbgolf.kicad_sch","pcbgolf_2.kicad_sch","pcbgolf_3.kicad_sch","pcbgolf_4.kicad_sch","pcbgolf_5.kicad_sch"
]
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

# JLCPCB 6-layer aggressive-but-published capability.
b.BuildConnectivity()
ns=b.GetConnectivity().GetNetSettings()
nc=ns.GetDefaultNetclass()
nc.SetClearance(pcbnew.FromMM(0.09))
nc.SetTrackWidth(pcbnew.FromMM(0.09))
nc.SetViaDiameter(pcbnew.FromMM(0.25))
nc.SetViaDrill(pcbnew.FromMM(0.15))
nc.SetDiffPairWidth(pcbnew.FromMM(0.09))
nc.SetDiffPairGap(pcbnew.FromMM(0.09))
nc.SetDiffPairViaGap(pcbnew.FromMM(0.09))
ns.ClearAllCaches()
ns.RecomputeEffectiveNetclasses()

pro_path=os.path.join(root,"pcbgolf.kicad_pro")
with open(pro_path) as f:
    pro=json.load(f)
rules=pro["board"]["design_settings"]["rules"]
rules["min_clearance"]=0.0762
rules["min_hole_clearance"]=0.10
rules["min_track_width"]=0.08
rules["min_via_diameter"]=0.25
rules["min_through_hole_diameter"]=0.15
default_nc=pro["net_settings"]["classes"][0]
default_nc.update({
  "clearance":0.09, "track_width":0.09,
  "via_diameter":0.25, "via_drill":0.15,
  "diff_pair_width":0.09, "diff_pair_gap":0.09, "diff_pair_via_gap":0.09
})
with open(pro_path,"w") as f:
    json.dump(pro,f,indent=2)
# wipe routing if any
for t in list(b.GetTracks()): b.Remove(t)
# ensure nets
name_to_net={}
for name in sorted(nets):
    ni=b.FindNet(name)
    if not ni:
        ni=pcbnew.NETINFO_ITEM(b,name)
        b.Add(ni)
    name_to_net[name]=ni

fps={fp.GetReference():fp for fp in b.GetFootprints()}
assigned=0; missing=[]
for name, entries in nets.items():
    ni=name_to_net[name]
    for ref,pin in entries:
        fp=fps.get(ref)
        if not fp:
            missing.append((name,ref,pin,"ref")); continue
        pad=fp.FindPadByNumber(str(pin))
        if not pad:
            missing.append((name,ref,pin,"pad")); continue
        pad.SetNet(ni); assigned+=1

# outline around footprint bounding boxes, excluding M2 decorative bolts from extents
for d in list(b.Drawings()):
    if d.GetLayer()==pcbnew.Edge_Cuts: b.Remove(d)
boxes=[fp.GetBoundingBox() for fp in b.GetFootprints() if not fp.GetReference().startswith("BH")]
xmin=min(bb.GetLeft() for bb in boxes); xmax=max(bb.GetRight() for bb in boxes)
ymin=min(bb.GetTop() for bb in boxes); ymax=max(bb.GetBottom() for bb in boxes)
m=pcbnew.FromMM(2.0)
xmin-=m; xmax+=m; ymin-=m; ymax+=m
for a,c in [((xmin,ymin),(xmax,ymin)),((xmax,ymin),(xmax,ymax)),((xmax,ymax),(xmin,ymax)),((xmin,ymax),(xmin,ymin))]:
    sh=pcbnew.PCB_SHAPE(b); sh.SetShape(pcbnew.SHAPE_T_SEGMENT); sh.SetLayer(pcbnew.Edge_Cuts)
    sh.SetStart(pcbnew.VECTOR2I(*a)); sh.SetEnd(pcbnew.VECTOR2I(*c)); sh.SetWidth(pcbnew.FromMM(0.05)); b.Add(sh)
b.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(1.6))
pcbnew.SaveBoard(board_path,b)

report={
 "assigned_pad_nodes":assigned,
 "source_nodes":len(nodes),
 "missing_nodes":missing,
 "footprints":len(fps),
 "nets":len(nets),
 "outline_mm":[pcbnew.ToMM(xmax-xmin),pcbnew.ToMM(ymax-ymin)],
 "routing_rules_mm":{"clearance":0.09,"track":0.09,"via_diameter":0.25,"via_drill":0.15}
}
open(os.path.join(root,"forward_annotate.json"),"w").write(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
if missing:
    print("WARNING missing nodes",len(missing),file=sys.stderr)

# trigger
