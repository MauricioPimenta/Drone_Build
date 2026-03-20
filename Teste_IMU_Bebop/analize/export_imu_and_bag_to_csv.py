#!/usr/bin/env python3
import os
import csv
import struct
from pathlib import Path

import numpy as np
import pandas as pd

# =========================
# SD LOG (.BIN) READER
# =========================
HEADER_FMT = "<4sHH"        # magic[4], uint16 ver, uint16 struct_size
HEADER_SIZE = struct.calcsize(HEADER_FMT)

SAMPLE_FMT = "<Iffffff"     # uint32 t_us + 6 float32
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)

SD_COLUMNS = ["t_us", "ax", "ay", "az", "gx", "gy", "gz"]


def read_sd_log_bin(path: Path) -> pd.DataFrame:
    data = path.read_bytes()
    if len(data) < HEADER_SIZE:
        raise RuntimeError(f"{path}: arquivo pequeno demais")

    magic, ver, sz = struct.unpack_from(HEADER_FMT, data, 0)
    if magic != b"IMUF":
        raise RuntimeError(f"{path}: magic inválido {magic!r} (esperado b'IMUF')")
    if sz != SAMPLE_SIZE:
        raise RuntimeError(
            f"{path}: struct_size={sz}, esperado {SAMPLE_SIZE}. "
            f"Seu firmware mudou a struct?"
        )

    payload = data[HEADER_SIZE:]
    n = len(payload) // SAMPLE_SIZE
    payload = payload[: n * SAMPLE_SIZE]

    dtype = np.dtype([
        ("t_us", "<u4"),
        ("ax",   "<f4"), ("ay", "<f4"), ("az", "<f4"),
        ("gx",   "<f4"), ("gy", "<f4"), ("gz", "<f4"),
    ])

    arr = np.frombuffer(payload, dtype=dtype, count=n)

    df = pd.DataFrame({k: arr[k] for k in SD_COLUMNS})
    df["t_s"] = df["t_us"] * 1e-6
    df["dt_s"] = df["t_s"].diff()
    df["fs_hz_inst"] = 1.0 / df["dt_s"]
    return df


def export_sd_folder(sd_folder: Path, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    bins = sorted(sd_folder.glob("LOG*.BIN"))
    if not bins:
        print(f"[SD] Nenhum LOG*.BIN encontrado em {sd_folder}")
        return

    for p in bins:
        df = read_sd_log_bin(p)
        out_csv = out_dir / f"{p.stem}.csv"
        df.to_csv(out_csv, index=False)
        print(f"[SD] {p.name} -> {out_csv}  ({len(df)} samples)")


# =========================
# ROSBAG -> CSV
# =========================
def quat_to_euler_xyz(qx, qy, qz, qw):
    # roll (x), pitch (y), yaw (z) - convenção ROS (REP 103), intrínseca XYZ
    # Implementação padrão (sem depender de tf)
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(t3, t4)

    return float(roll), float(pitch), float(yaw)


def export_rosbag(bag_path: Path, out_dir: Path, add_euler=True):
    out_dir.mkdir(parents=True, exist_ok=True)

    # ROS1: precisa rodar com ambiente do ROS carregado (source /opt/ros/noetic/setup.bash)
    import rosbag

    # arquivos por tópico
    writers = {}
    files = {}

    def get_writer(topic, fieldnames):
        if topic in writers:
            return writers[topic]
        safe = topic.strip("/").replace("/", "__")
        f = open(out_dir / f"{safe}.csv", "w", newline="")
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        writers[topic] = w
        files[topic] = f
        return w

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, t in bag.read_messages():
            t_bag = t.to_sec()

            # /B1/cmd_vel : geometry_msgs/Twist (sem header)
            if topic == "/B1/cmd_vel":
                w = get_writer(topic, [
                    "t_bag",
                    "lin_x","lin_y","lin_z",
                    "ang_x","ang_y","ang_z",
                ])
                w.writerow({
                    "t_bag": t_bag,
                    "lin_x": msg.linear.x, "lin_y": msg.linear.y, "lin_z": msg.linear.z,
                    "ang_x": msg.angular.x, "ang_y": msg.angular.y, "ang_z": msg.angular.z,
                })

            # /B1/odom : nav_msgs/Odometry (tem header)
            elif topic == "/B1/odom":
                t_msg = msg.header.stamp.to_sec()
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                lv = msg.twist.twist.linear
                av = msg.twist.twist.angular

                base_fields = [
                    "t_msg","t_bag",
                    "px","py","pz",
                    "qx","qy","qz","qw",
                    "vx","vy","vz",
                    "wx","wy","wz",
                ]
                if add_euler:
                    base_fields += ["roll","pitch","yaw"]

                w = get_writer(topic, base_fields)

                row = {
                    "t_msg": t_msg, "t_bag": t_bag,
                    "px": p.x, "py": p.y, "pz": p.z,
                    "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
                    "vx": lv.x, "vy": lv.y, "vz": lv.z,
                    "wx": av.x, "wy": av.y, "wz": av.z,
                }
                if add_euler:
                    r, pit, y = quat_to_euler_xyz(q.x, q.y, q.z, q.w)
                    row.update({"roll": r, "pitch": pit, "yaw": y})

                w.writerow(row)

            # PoseStamped (Optitrack)
            elif topic in ("/natnet_ros/B1/pose"):
                t_msg = msg.header.stamp.to_sec()
                p = msg.pose.position
                q = msg.pose.orientation

                base_fields = [
                    "t_msg","t_bag",
                    "px","py","pz",
                    "qx","qy","qz","qw",
                ]
                if add_euler:
                    base_fields += ["roll","pitch","yaw"]

                w = get_writer(topic, base_fields)

                row = {
                    "t_msg": t_msg, "t_bag": t_bag,
                    "px": p.x, "py": p.y, "pz": p.z,
                    "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
                }
                if add_euler:
                    r, pit, y = quat_to_euler_xyz(q.x, q.y, q.z, q.w)
                    row.update({"roll": r, "pitch": pit, "yaw": y})

                w.writerow(row)

            # takeoff/land : std_msgs/Empty (sem header)
            elif topic in ("/B1/takeoff", "/B1/land"):
                w = get_writer(topic, ["t_bag", "event"])
                w.writerow({
                    "t_bag": t_bag,
                    "event": "takeoff" if topic.endswith("takeoff") else "land"
                })

    # fecha arquivos
    for f in files.values():
        f.close()

    print(f"[BAG] Export concluído para {out_dir}")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", type=str, required=True, help="caminho do .bag")
    ap.add_argument("--out", type=str, default="out_csv", help="pasta de saída")
    ap.add_argument("--sd_folder", type=str, default="", help="pasta com LOG*.BIN (opcional)")
    ap.add_argument("--no_euler", action="store_true", help="não salvar roll/pitch/yaw")
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    export_rosbag(Path(args.bag), out_dir / "bag", add_euler=not args.no_euler)

    if args.sd_folder:
        export_sd_folder(Path(args.sd_folder), out_dir / "sd")

    print("OK.")


if __name__ == "__main__":
    main()