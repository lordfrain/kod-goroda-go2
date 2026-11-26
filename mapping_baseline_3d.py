"""Offline PointCloud2 + Odometry baseline; not a full SLAM implementation."""
import argparse, csv, json
from pathlib import Path
import numpy as np

def unit(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError("Invalid quaternion")
    return q / n

def rotation(q):
    x,y,z,w = unit(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def slerp(q0,q1,a):
    q0,q1 = unit(q0),unit(q1)
    d = float(q0 @ q1)
    if d < 0:
        q1,d = -q1,-d
    if d > 0.9995:
        return unit((1-a)*q0+a*q1)
    theta = np.arccos(np.clip(d,-1,1))
    return (np.sin((1-a)*theta)*q0+np.sin(a*theta)*q1)/np.sin(theta)

def pose_at(times, poses, t, gap):
    i = int(np.searchsorted(times,t))
    if i < len(times) and times[i] == t:
        return poses[i]
    if i == 0 or i == len(times) or times[i]-times[i-1] > gap:
        return None
    a = (t-times[i-1])/(times[i]-times[i-1])
    return np.r_[(1-a)*poses[i-1,:3]+a*poses[i,:3],slerp(poses[i-1,3:],poses[i,3:],a)]

def transform(points, pose):
    return points @ rotation(pose[3:]).T + pose[:3]

def voxel(points, size):
    if not len(points):
        return points
    _, i = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[i]

def stamp(m):
    return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec

def xyz(v): return [v.x,v.y,v.z]
def quat(v): return [v.x,v.y,v.z,v.w]

def bag_messages(args, topic, expected):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag),storage_id=args.storage),
                rosbag2_py.ConverterOptions("cdr","cdr"))
    types = {x.name:x.type for x in reader.get_all_topics_and_types()}
    if types.get(topic) != expected:
        raise ValueError(f"{topic}: expected {expected}, got {types.get(topic)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    cls = get_message(expected)
    while reader.has_next():
        _, data, _ = reader.read_next()
        yield deserialize_message(data,cls)

def write_ply(path, points):
    with path.open("wb") as f:
        f.write(f"ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n".encode("ascii"))
        f.write(np.asarray(points,dtype="<f4").tobytes())

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag",type=Path)
    ap.add_argument("--storage",required=True,choices=["mcap","sqlite3"])
    ap.add_argument("--odom",required=True)
    ap.add_argument("--cloud",required=True)
    ap.add_argument("--base-frame",required=True)
    ap.add_argument("--cloud-frame",required=True)
    ap.add_argument("--extrinsic",nargs=7,type=float,required=True,metavar=("X","Y","Z","QX","QY","QZ","QW"),
                    help="Verified transform from cloud frame to odometry child frame; meters, quaternion xyzw")
    ap.add_argument("--gap-ms",type=float,default=200)
    ap.add_argument("--voxel",type=float,default=.10)
    ap.add_argument("--output",type=Path,default=Path("baseline_output"))
    a=ap.parse_args()
    if a.voxel <= 0 or a.gap_ms <= 0: ap.error("Positive voxel/gap required")
    ext=np.array(a.extrinsic); unit(ext[3:])
    if not np.all(np.isfinite(ext)): ap.error("Non-finite extrinsic")
    rows={}; parent=None
    for m in bag_messages(a,a.odom,"nav_msgs/msg/Odometry"):
        if m.child_frame_id != a.base_frame: raise ValueError("Unexpected odometry child frame")
        if parent is None: parent=m.header.frame_id
        if parent != m.header.frame_id: raise ValueError("Odometry parent frame changed")
        p=np.array(xyz(m.pose.pose.position)+quat(m.pose.pose.orientation))
        if np.all(np.isfinite(p)) and np.linalg.norm(p[3:])>1e-12: rows[stamp(m)]=p
    if len(rows)<2: raise ValueError("Not enough valid odometry")
    times=np.array(sorted(rows),dtype=np.int64); poses=np.array([rows[t] for t in times])
    from sensor_msgs_py import point_cloud2
    counts=dict(clouds=0,used=0,missing_pose=0,empty=0,input_points=0,output_points=0)
    merged=np.empty((0,3)); batch=[]; scan_rows=[]
    for m in bag_messages(a,a.cloud,"sensor_msgs/msg/PointCloud2"):
        counts["clouds"]+=1
        if m.header.frame_id != a.cloud_frame: raise ValueError("Unexpected cloud frame; verify TF")
        t=stamp(m); pose=pose_at(times,poses,t,a.gap_ms*1e6)
        if pose is None:
            counts["missing_pose"]+=1; continue
        raw=point_cloud2.read_points(m,field_names=("x","y","z"),skip_nans=True)
        points=np.column_stack([raw[name] for name in ("x","y","z")]).astype(float)
        points=points[np.isfinite(points).all(axis=1)]
        counts["input_points"]+=len(points)
        if not len(points): counts["empty"]+=1; continue
        global_points=transform(transform(voxel(points,a.voxel),ext),pose)
        batch.append(global_points); counts["used"]+=1
        scan_rows.append([int(t),*pose])
        if len(batch)>=20:
            merged=voxel(np.concatenate([merged,*batch]),a.voxel); batch=[]
    if batch: merged=voxel(np.concatenate([merged,*batch]),a.voxel)
    if not len(merged): raise ValueError("No usable clouds")
    a.output.mkdir(parents=True,exist_ok=True)
    write_ply(a.output/"map.ply",merged)
    with (a.output/"trajectory.csv").open("w",newline="") as f:
        w=csv.writer(f); w.writerow(["stamp_ns","x","y","z","qx","qy","qz","qw"]); w.writerows(scan_rows)
    counts["output_points"]=len(merged)
    counts["odom_poses"]=len(times)
    counts["odom_gap_max_ms"]=float(np.diff(times).max()/1e6)
    counts["used_fraction"]=counts["used"]/max(1,counts["clouds"])
    counts["odom_path_length_m"]=float(np.linalg.norm(np.diff(poses[:,:3],axis=0),axis=1).sum())
    counts["odom_endpoint_distance_m"]=float(np.linalg.norm(poses[-1,:3]-poses[0,:3]))
    counts["map_frame"]=parent
    counts["note"]="Internal consistency only; no ground truth or loop-closure claim."
    (a.output/"quality.json").write_text(json.dumps(counts,indent=2),encoding="utf8")
    print(json.dumps(counts,indent=2))

if __name__=="__main__": main()

