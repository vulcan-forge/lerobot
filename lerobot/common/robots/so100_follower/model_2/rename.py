#!/usr/bin/env python3
import sys
import xml.etree.ElementTree as ET

def sanitize(s: str) -> str:
    # replace all hyphens with underscores
    return s.replace('-', '_')

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} INPUT.urdf [OUTPUT.urdf]")
        sys.exit(1)

    input_urdf  = sys.argv[1]
    output_urdf = sys.argv[2] if len(sys.argv) > 2 else input_urdf

    # parse
    tree = ET.parse(input_urdf)
    root = tree.getroot()

    # fix <link name="..."> and <joint name="...">
    for tag in ('link', 'joint'):
        for elem in root.findall(tag):
            name = elem.get('name')
            if name and '-' in name:
                elem.set('name', sanitize(name))

    # fix all parent/child link references in joints
    for joint in root.findall('joint'):
        for sub in ('parent', 'child'):
            node = joint.find(sub)
            if node is not None:
                link = node.get('link')
                if link and '-' in link:
                    node.set('link', sanitize(link))

    # write out
    tree.write(output_urdf, encoding='utf-8', xml_declaration=True)
    print(f"Sanitized URDF written to: {output_urdf}")

if __name__ == '__main__':
    main()
