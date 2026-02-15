# Compute and store a bag hash (ROS2 bag directory)
import hashlib, os, pathlib

def hash_bag(bag_dir):
    p = pathlib.Path(bag_dir)
    h = hashlib.sha256()
    for f in sorted(p.rglob("*")):
        if f.is_file():
            h.update(f.read_bytes())
    return h.hexdigest()

print(hash_bag("my_run_bag"))
      
