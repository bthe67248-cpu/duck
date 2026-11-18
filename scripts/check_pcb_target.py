"""Kinematic PCB/shield sizing study; does not modify CAD, mass or mounting files.

Run from any directory: python scripts/check_pcb_target.py [--samples 6000]
The board's long dimension is vertical; shield is board size + existing 6.6 mm
allowances, 18.5 mm outer depth. Guarded cases expand every shield box half-size
by 3 mm. Only sampled shield-to-other-body contacts are counted; this does not
certify full joint limits, dynamic walking, get-up or the installed UNO/HAT stack.
"""
import argparse,collections,hashlib,json,pathlib,xml.etree.ElementTree as ET
import mujoco,numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--samples',type=int,default=6000)
ap.add_argument('--seed',type=int,default=1)
ap.add_argument('--output',type=pathlib.Path,default=ROOT/'results/hardware_readiness/pcb_target_check.json')
args=ap.parse_args()
if args.samples<1:ap.error('--samples must be positive')
base=mujoco.MjModel.from_xml_path(str(ROOT/'model/duck_v3_gc_scene.xml'))
d=mujoco.MjData(base);mujoco.mj_resetDataKeyframe(base,d,base.key('sim_start').id)
home=d.qpos.copy();rng=np.random.default_rng(args.seed)
head={'neck_pitch':.4,'head_pitch':.6,'head_yaw':1.2,'head_roll':.44}
joints=[base.actuator(i).name for i in range(14)]
poses=[]
for _ in range(args.samples):
 q=home.copy()
 for n in list(head)+[n for n in joints if n not in head]:
  j=base.joint(n).id; amount=head.get(n,.3)
  q[base.jnt_qposadr[j]]=np.clip(home[base.jnt_qposadr[j]]+rng.uniform(-amount,amount),*base.jnt_range[j])
 poses.append(q)

def evaluate(L=110,W=53.34,cx=-50,cz=20,guard_mm=0):
 root=ET.parse(ROOT/'model/duck_v3_gc.xml').getroot()
 root.find('compiler').set('meshdir',str(ROOT/'model/assets'))
 for g in root.iter('geom'):
  name=g.get('name','')
  if name=='shield_back_collision':
   g.set('size',f'{(W+6.6)/2000} {(L+6.6)/2000} .00125')
   g.set('pos',f'{cx/1000-.008} 0 {cz/1000}')
  elif name in ('shield_side_l_collision','shield_side_r_collision'):
   g.set('size',f'.00125 {(L+6.6)/2000} .00925')
   sy=(W/2+2.05)/1000*(1 if name.endswith('l_collision') else -1)
   g.set('pos',f'{cx/1000} {sy} {cz/1000}')
 for g in root.iter('geom'):
  if g.get('name','').startswith('shield_') and g.get('name','').endswith('_collision'):
   g.set('size',' '.join(str(float(v)+guard_mm/1000) for v in g.get('size').split()))
 # Attach original scene floor/keyframes to the edited robot. Asset names are disjoint.
 scene=ET.parse(ROOT/'model/duck_v3_gc_scene.xml').getroot()
 for section in ('asset','worldbody'):
  root.find(section).extend(list(scene.find(section)))
 root.append(scene.find('keyframe'))
 m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(m)
 shield={m.geom(n).id for n in ['shield_back_collision','shield_side_l_collision','shield_side_r_collision']}
 pairs=collections.Counter();poses_bad=0;worst=0
 for q in poses:
  data.qpos[:]=q;mujoco.mj_forward(m,data)
  found=set()
  for c in data.contact[:data.ncon]:
   if (c.geom1 in shield or c.geom2 in shield) and c.geom1!=m.geom('floor').id and c.geom2!=m.geom('floor').id:
    found.add(tuple(sorted((m.geom(c.geom1).name or m.body(m.geom_bodyid[c.geom1]).name,m.geom(c.geom2).name or m.body(m.geom_bodyid[c.geom2]).name))))
    worst=min(worst,c.dist)
  poses_bad+=bool(found);pairs.update(found)
 return dict(samples=args.samples,guard_inflation_mm=guard_mm,pcb_length_mm=L,pcb_width_mm=W,shield_center_mm=[cx,0,cz],bad_poses=poses_bad,worst_penetration_mm=round(-worst*1000,2),pairs=dict((str(k),v) for k,v in pairs.items()))
if __name__=='__main__':
 rows=[]
 for L,cx,cz,guard in [(110,-50,20,0),(70,-50,20,0),(70,-70,0,0),(70,-70,0,3)]:
  row=evaluate(L=L,cx=cx,cz=cz,guard_mm=guard);rows.append(row);print(json.dumps(row),flush=True)
 source_hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['model/duck_v3_gc.xml','model/duck_v3_gc_scene.xml','scripts/collision_proxies.json']}
 result=dict(seed=args.seed,operating_envelope_head_rad=head,operating_envelope_legs_rad=.3,
             method='Static random poses; shield collision proxies only resized/repositioned; guarded case expands box half-sizes. No CAD, PCB routing, mass or dynamics changed.',
             source_sha256=source_hashes,candidates=rows)
 args.output.parent.mkdir(parents=True,exist_ok=True)
 args.output.write_text(json.dumps(result,indent=2)+'\n')
