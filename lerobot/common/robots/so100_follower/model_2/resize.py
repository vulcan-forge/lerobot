#!/usr/bin/env python3
import sys
import xml.etree.ElementTree as ET

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} INPUT.urdf [OUTPUT.urdf]")
    sys.exit(1)

in_file  = sys.argv[1]
out_file = sys.argv[2] if len(sys.argv) > 2 else in_file

tree = ET.parse(in_file)
root = tree.getroot()

# add scale to every mesh tag
for mesh in root.findall('.//mesh'):
    mesh.set('scale', '0.001 0.001 0.001')

tree.write(out_file, encoding='utf-8', xml_declaration=True)
print(f"Patched meshes to mm→m scale in: {out_file}")
