#!/usr/bin/env python3
"""Convert MuJoCo MJCF XML to URDF."""

import xml.etree.ElementTree as ET
import math
import argparse
from pathlib import Path


def parse_vec(s, n=None):
    v = [float(x) for x in s.split()]
    if n is not None and len(v) != n:
        v = (v + [0.0] * n)[:n]
    return v


def quat_to_rpy(w, x, y, z):
    """MuJoCo quat (w x y z) → URDF rpy (roll pitch yaw, extrinsic XYZ)."""
    r00 = 1 - 2 * (y * y + z * z)
    r10 = 2 * (x * y + w * z)
    r20 = 2 * (x * z - w * y)
    r21 = 2 * (y * z + w * x)
    r22 = 1 - 2 * (x * x + y * y)
    r01 = 2 * (x * y - w * z)
    r02 = 2 * (x * z + w * y)
    r12 = 2 * (y * z - w * x)
    r11 = 1 - 2 * (x * x + z * z)

    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(-r12, r11)
        yaw = 0.0
    return roll, pitch, yaw


def fmt_v(v):
    return " ".join(f"{x:.8g}" for x in v)


def fmt_rpy(rpy):
    return fmt_v(rpy)


def fmt_xyz(pos):
    return fmt_v(pos)


class Converter:
    def __init__(self, mjcf_path, mesh_pkg="package://g1_description/meshes/", xacro=False):
        self.tree = ET.parse(mjcf_path)
        self.root = self.tree.getroot()
        self.mesh_pkg = mesh_pkg
        self.xacro = xacro
        # name -> filename
        self.mesh_files = {}
        # class_name -> tag -> {attr: val}
        self.defaults = {}
        self.links = []
        self.joints = []
        self._parse_assets()
        self._parse_defaults(self.root.find("default"), parent_class=None)

    # ── asset parsing ──────────────────────────────────────────────────────

    def _parse_assets(self):
        asset = self.root.find("asset")
        if asset is None:
            return
        for mesh in asset.findall("mesh"):
            name = mesh.get("name", "")
            file = mesh.get("file", name + ".STL")
            self.mesh_files[name] = file

    # ── default class parsing ──────────────────────────────────────────────

    def _parse_defaults(self, elem, parent_class):
        if elem is None:
            return
        cls = elem.get("class") or "__global__"
        merged = dict(self.defaults.get(parent_class or "__global__", {}) if parent_class else {})
        for child in elem:
            if child.tag == "default":
                self._parse_defaults(child, cls)
            else:
                tag_defaults = dict(merged.get(child.tag, {}))
                tag_defaults.update(child.attrib)
                merged[child.tag] = tag_defaults
        self.defaults[cls] = merged

    def _resolve(self, elem, inherited_class):
        """Return effective attribute dict for elem, merging class defaults."""
        cls = elem.get("class") or inherited_class
        base = {}
        if cls and cls in self.defaults:
            base.update(self.defaults[cls].get(elem.tag, {}))
        if "__global__" in self.defaults:
            for k, v in self.defaults["__global__"].get(elem.tag, {}).items():
                base.setdefault(k, v)
        base.update(elem.attrib)
        return base

    # ── body / link traversal ─────────────────────────────────────────────

    def convert(self):
        worldbody = self.root.find("worldbody")
        self.links.append("  <link name=\"world\"/>")
        for body in worldbody.findall("body"):
            self._walk_body(body, parent="world", parent_class=None)

    def _walk_body(self, body, parent, parent_class):
        name = body.get("name")
        if not name:
            return

        # Effective child class for this body's contents
        child_class = body.get("childclass") or body.get("class") or parent_class

        # Body pose relative to parent
        pos = parse_vec(body.get("pos", "0 0 0"), 3)
        quat_str = body.get("quat")
        if quat_str:
            w, x, y, z = parse_vec(quat_str, 4)
            rpy = quat_to_rpy(w, x, y, z)
        else:
            rpy = (0.0, 0.0, 0.0)

        # Emit link
        self.links.append(self._mk_link(body, child_class))

        # Emit joint(s)
        joints_in_body = body.findall("joint")
        if joints_in_body:
            j = joints_in_body[0]
            self.joints.append(self._mk_joint(j, parent, name, pos, rpy, child_class))
        else:
            self.joints.append(self._mk_fixed(name, parent, pos, rpy))

        # Recurse
        for child in body.findall("body"):
            self._walk_body(child, name, child_class)

    # ── link builder ──────────────────────────────────────────────────────

    def _mk_link(self, body, cls):
        name = body.get("name")
        parts = [f'  <link name="{name}">']

        inertial = body.find("inertial")
        if inertial is not None:
            parts.append(self._mk_inertial(inertial))

        for geom in body.findall("geom"):
            attrs = self._resolve(geom, cls)
            g_type = attrs.get("type", "sphere")
            g_class = attrs.get("class", "")
            contype = attrs.get("contype", "1")
            group = attrs.get("group", "0")

            # Skip tendon-visualization geoms
            if group == "4":
                continue

            is_visual = (contype == "0") or (g_class == "visual") or (g_type == "mesh" and contype == "0")
            is_mesh = g_type == "mesh"
            is_prim = g_type in ("box", "capsule", "cylinder", "sphere")

            if is_mesh:
                v = self._mk_visual_mesh(attrs)
                if v:
                    parts.append(v)
            elif is_prim and not is_visual:
                c = self._mk_collision_prim(attrs, g_type)
                if c:
                    parts.append(c)

        parts.append("  </link>")
        return "\n".join(parts)

    def _mk_inertial(self, elem):
        pos = parse_vec(elem.get("pos", "0 0 0"), 3)
        mass = float(elem.get("mass", "0"))
        quat_str = elem.get("quat")
        if quat_str:
            w, x, y, z = parse_vec(quat_str, 4)
            rpy = quat_to_rpy(w, x, y, z)
        else:
            rpy = (0.0, 0.0, 0.0)
        di = elem.get("diaginertia")
        if di:
            ixx, iyy, izz = parse_vec(di, 3)
            ixy = ixz = iyz = 0.0
        else:
            fi = elem.get("fullinertia")
            if fi:
                ixx, iyy, izz, ixy, ixz, iyz = parse_vec(fi, 6)
            else:
                ixx = iyy = izz = ixy = ixz = iyz = 1e-6

        return (
            f"    <inertial>\n"
            f"      <origin xyz=\"{fmt_xyz(pos)}\" rpy=\"{fmt_rpy(rpy)}\"/>\n"
            f"      <mass value=\"{mass:.8g}\"/>\n"
            f"      <inertia ixx=\"{ixx:.9g}\" iyy=\"{iyy:.9g}\" izz=\"{izz:.9g}\""
            f" ixy=\"{ixy:.9g}\" ixz=\"{ixz:.9g}\" iyz=\"{iyz:.9g}\"/>\n"
            f"    </inertial>"
        )

    def _geom_pose(self, attrs):
        pos = parse_vec(attrs.get("pos", "0 0 0"), 3)
        quat_str = attrs.get("quat")
        if quat_str:
            w, x, y, z = parse_vec(quat_str, 4)
            rpy = quat_to_rpy(w, x, y, z)
        else:
            rpy = (0.0, 0.0, 0.0)
        return pos, rpy

    def _mk_visual_mesh(self, attrs):
        mesh_name = attrs.get("mesh")
        if not mesh_name:
            return None
        pos, rpy = self._geom_pose(attrs)
        fname = self.mesh_files.get(mesh_name, mesh_name + ".STL")
        rgba_str = attrs.get("rgba", "0.7 0.7 0.7 1")
        rgba = parse_vec(rgba_str)
        r, g, b, a = (rgba + [1.0])[:4]
        prefix = "${mesh_pkg}" if self.xacro else self.mesh_pkg
        return (
            f"    <visual>\n"
            f"      <origin xyz=\"{fmt_xyz(pos)}\" rpy=\"{fmt_rpy(rpy)}\"/>\n"
            f"      <geometry>\n"
            f"        <mesh filename=\"{prefix}{fname}\"/>\n"
            f"      </geometry>\n"
            f"      <material name=\"mesh_mat\">\n"
            f"        <color rgba=\"{r:.3f} {g:.3f} {b:.3f} {a:.3f}\"/>\n"
            f"      </material>\n"
            f"    </visual>"
        )

    def _mk_collision_prim(self, attrs, g_type):
        pos, rpy = self._geom_pose(attrs)
        size_str = attrs.get("size", "0.01")
        size = parse_vec(size_str)

        if g_type == "box":
            # half-extents → full extents
            sx, sy, sz = (size + [size[0], size[0]])[:3]
            geo = f'        <box size="{2*sx:.8g} {2*sy:.8g} {2*sz:.8g}"/>'
        elif g_type in ("capsule", "cylinder"):
            r = size[0]
            half_l = size[1] if len(size) > 1 else size[0]
            geo = f'        <cylinder radius="{r:.8g}" length="{2*half_l:.8g}"/>'
        elif g_type == "sphere":
            r = size[0]
            geo = f'        <sphere radius="{r:.8g}"/>'
        else:
            return None

        return (
            f"    <collision>\n"
            f"      <origin xyz=\"{fmt_xyz(pos)}\" rpy=\"{fmt_rpy(rpy)}\"/>\n"
            f"      <geometry>\n"
            f"{geo}\n"
            f"      </geometry>\n"
            f"    </collision>"
        )

    # ── joint builder ─────────────────────────────────────────────────────

    def _mk_joint(self, j, parent, child, pos, rpy, cls):
        attrs = self._resolve(j, cls)
        jname = attrs.get("name", f"joint_{child}")
        mj_type = attrs.get("type", "hinge")

        type_map = {"hinge": "revolute", "slide": "prismatic",
                    "free": "floating", "ball": "fixed"}
        urdf_type = type_map.get(mj_type, "fixed")

        axis = parse_vec(attrs.get("axis", "0 0 1"), 3)
        range_str = attrs.get("range", "-3.14159 3.14159")
        lo, hi = parse_vec(range_str, 2)

        effort = 0.0
        frc = attrs.get("actuatorfrcrange")
        if frc:
            fv = parse_vec(frc, 2)
            effort = max(abs(fv[0]), abs(fv[1]))

        lines = [
            f'  <joint name="{jname}" type="{urdf_type}">',
            f'    <parent link="{parent}"/>',
            f'    <child link="{child}"/>',
            f'    <origin xyz="{fmt_xyz(pos)}" rpy="{fmt_rpy(rpy)}"/>',
        ]
        if urdf_type in ("revolute", "prismatic"):
            lines.append(f'    <axis xyz="{fmt_v(axis)}"/>')
            lines.append(
                f'    <limit lower="{lo:.8g}" upper="{hi:.8g}"'
                f' effort="{effort:.4g}" velocity="10.0"/>'
            )
        elif urdf_type == "floating":
            pass  # no axis/limit for floating

        lines.append("  </joint>")
        return "\n".join(lines)

    def _mk_fixed(self, child, parent, pos, rpy):
        return (
            f'  <joint name="fixed_{child}" type="fixed">\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{child}"/>\n'
            f'    <origin xyz="{fmt_xyz(pos)}" rpy="{fmt_rpy(rpy)}"/>\n'
            f'  </joint>'
        )

    # ── output ────────────────────────────────────────────────────────────

    def to_urdf(self, xacro=False):
        self.convert()
        robot_name = self.root.get("model", "g1_tether")
        if xacro:
            header = (
                '<?xml version="1.0"?>\n'
                '<robot name="' + robot_name + '" '
                'xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
                '\n'
                '  <!-- mesh path — override: xacro g1_tether.urdf.xacro '
                'mesh_pkg:=package://other/meshes/ -->\n'
                '  <xacro:arg name="mesh_pkg" '
                'default="package://g1_description/meshes/"/>\n'
                '  <xacro:property name="mesh_pkg" '
                'value="$(arg mesh_pkg)"/>\n'
            )
            footer = "</robot>"
        else:
            header = '<?xml version="1.0"?>\n<robot name="' + robot_name + '">\n'
            footer = "</robot>"

        parts = [header]
        parts.extend(self.links)
        parts.append("")
        parts.extend(self.joints)
        parts.append("")
        parts.append(footer)
        return "\n".join(parts)


def main():
    ap = argparse.ArgumentParser(description="MJCF → URDF/xacro converter")
    ap.add_argument("input", help="Path to .xml MJCF file")
    ap.add_argument("output", help="Path for output file (.urdf or .urdf.xacro)")
    ap.add_argument("--mesh-pkg", default="package://g1_description/meshes/",
                    help="ROS package:// prefix for mesh filenames")
    ap.add_argument("--xacro", action="store_true",
                    help="Emit xacro format with ${mesh_pkg} substitution")
    args = ap.parse_args()

    c = Converter(args.input, mesh_pkg=args.mesh_pkg, xacro=args.xacro)
    out = c.to_urdf(xacro=args.xacro)
    Path(args.output).write_text(out)
    print(f"Written {args.output}")


if __name__ == "__main__":
    main()
