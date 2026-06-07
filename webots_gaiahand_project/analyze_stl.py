"""Analyze STL file complexity - count triangle faces for each mesh."""
import struct
import os
import glob

meshes_dir = os.path.join(os.path.dirname(__file__), "meshes")
stl_files = glob.glob(os.path.join(meshes_dir, "*.STL"))

print(f"{'File':<40s} {'Faces':>10s} {'Size (MB)':>10s}")
print("-" * 62)

total_faces = 0
total_size = 0

for f in sorted(stl_files):
    size = os.path.getsize(f)
    with open(f, 'rb') as fp:
        fp.read(80)  # skip header
        count = struct.unpack('<I', fp.read(4))[0]
    size_mb = size / (1024 * 1024)
    print(f"{os.path.basename(f):<40s} {count:>10,} {size_mb:>10.2f}")
    total_faces += count
    total_size += size

print("-" * 62)
print(f"{'TOTAL':<40s} {total_faces:>10,} {total_size/(1024*1024):>10.2f}")
