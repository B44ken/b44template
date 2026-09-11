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
b.SetCopperLayerCount(4)

# spread footprints with a simple shelf packer; this is a guaranteed-connectivity fallback
fps=list(b.GetFootprints())
items=[]
for fp in fps:
    bb=fp.GetBoundingBox()
    items.append((bb.GetHeight(), bb.GetWidth(), fp))
items.sort(reverse=True, key=lambda t:(t[0],t[1]))
target_w=pcbnew.FromMM(140)
gap=pcbnew.FromMM(2.5)
x=pcbnew.FromMM(10); y=pcbnew.FromMM(10); row_h=0
for h,w,fp in items:
    bb=fp.GetBoundingBox()
    if x + w > target_w:
        x=pcbnew.FromMM(10)
        y += row_h + gap
        row_h=0
        bb=fp.GetBoundingBox()
    dx=x-bb.GetLeft(); dy=y-bb.GetTop()
    fp.Move(pcbnew.VECTOR2I(dx,dy))
    x += w + gap
    row_h=max(row_h,h)
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
m=pcbnew.FromMM(5.0)
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
 "outline_mm":[pcbnew.ToMM(xmax-xmin),pcbnew.ToMM(ymax-ymin)]
}
open(os.path.join(root,"forward_annotate.json"),"w").write(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
if missing:
    print("WARNING missing nodes",len(missing),file=sys.stderr)

# trigger
