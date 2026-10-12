# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Keyboard teleoperation of the Touch: driver + velocity bridge + control panel + 3D view.

    ros2 launch touch_teleop teleop.launch.py                 # everything
    ros2 launch touch_teleop teleop.launch.py rviz:=false     # no 3D view
    ros2 launch touch_teleop teleop.launch.py dashboard:=false

Optional keyboard node, in its own terminal (it reads the keys from it):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/touch/cmd_vel

The 3D model of the Touch comes from the package omni_description (Deliverable 1 driver,
third_party/geomagic_touch_ros2). If it is not installed, the 3D view shows only the
target and the workspace.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Colours of the 3D model (the omni_description meshes have none): RGBA per link.
LINK_COLORS = {
    'base': '0.22 0.22 0.25 1',
    'torso': '0.40 0.40 0.44 1',
    'upper_arm': '0.15 0.45 0.85 1',
    'lower_arm': '0.15 0.45 0.85 1',
    'wrist': '0.70 0.70 0.72 1',
    'tip': '0.70 0.70 0.72 1',
    'stylus': '0.12 0.12 0.12 1',
}
DEFAULT_COLOR = '0.6 0.6 0.6 1'


MESH_CACHE = os.path.expanduser('~/.touch_raw_driver/meshes')


def _read_stl(path):
    """Return the triangles of an STL file (binary or ASCII) as a list of 3 vertices."""
    import struct
    data = open(path, 'rb').read()
    tris = []
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * n == len(data):                       # binary STL
            for i in range(n):
                v = struct.unpack('<12f', data[84 + 50 * i: 84 + 50 * i + 48])
                tris.append((v[3:6], v[6:9], v[9:12]))
            return tris
    verts = []                                             # ASCII STL
    for line in data.decode('ascii', 'ignore').splitlines():
        parts = line.split()
        if parts[:1] == ['vertex']:
            verts.append(tuple(float(x) for x in parts[1:4]))
            if len(verts) == 3:
                tris.append(tuple(verts))
                verts = []
    return tris


def _write_fixed_stl(tris, path):
    """Write a binary STL with outward normals recomputed from the geometry.

    The original meshes have missing or inconsistent normals, which RViz renders black.
    The winding is made consistent with a positive enclosed volume (outward normals).
    """
    import struct

    def sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    volume = sum(v0[0] * (v1[1] * v2[2] - v1[2] * v2[1]) - v0[1] * (v1[0] * v2[2] - v1[2] * v2[0])
                 + v0[2] * (v1[0] * v2[1] - v1[1] * v2[0]) for v0, v1, v2 in tris)
    if volume < 0:
        tris = [(v0, v2, v1) for v0, v1, v2 in tris]
    out = [b'touch_teleop fixed mesh'.ljust(80, b' '), struct.pack('<I', len(tris))]
    for v0, v1, v2 in tris:
        nx, ny, nz = cross(sub(v1, v0), sub(v2, v0))
        norm = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
        out.append(struct.pack('<12fH', nx / norm, ny / norm, nz / norm,
                               *v0, *v1, *v2, 0))
    with open(path, 'wb') as f:
        f.write(b''.join(out))


def _fixed_mesh_uri(uri):
    """package://omni_description/meshes/x.stl -> file:// URI of a copy with good normals."""
    prefix = 'package://'
    if not uri.startswith(prefix) or not uri.lower().endswith('.stl'):
        return uri
    pkg, rel = uri[len(prefix):].split('/', 1)
    src = os.path.join(get_package_share_directory(pkg), rel)
    os.makedirs(MESH_CACHE, exist_ok=True)
    dst = os.path.join(MESH_CACHE, pkg + '_' + os.path.basename(rel))
    _write_fixed_stl(_read_stl(src), dst)
    return 'file://' + dst


def colored_robot_description(xacro_file):
    """Process the third-party xacro: colour every visual and fix the mesh normals."""
    import xacro
    doc = xacro.process_file(xacro_file)
    for mesh in doc.getElementsByTagName('mesh'):
        mesh.setAttribute('filename', _fixed_mesh_uri(mesh.getAttribute('filename')))
    for link in doc.getElementsByTagName('link'):
        rgba = LINK_COLORS.get(link.getAttribute('name'), DEFAULT_COLOR)
        for visual in link.getElementsByTagName('visual'):
            for old in visual.getElementsByTagName('material'):
                visual.removeChild(old)
            material = doc.createElement('material')
            material.setAttribute('name', 'touch_' + link.getAttribute('name'))
            color = doc.createElement('color')
            color.setAttribute('rgba', rgba)
            material.appendChild(color)
            visual.appendChild(material)
    return doc.toxml()


def generate_launch_description():
    driver_params = os.path.join(get_package_share_directory('touch_raw_driver'),
                                 'config', 'touch_raw_driver.yaml')
    teleop_share = get_package_share_directory('touch_teleop')
    teleop_params = os.path.join(teleop_share, 'config', 'teleop.yaml')
    rviz_config = os.path.join(teleop_share, 'rviz', 'touch_teleop.rviz')
    rviz_on = IfCondition(LaunchConfiguration('rviz'))

    actions = [
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='Start the control panel (buttons, arrow keys)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start the 3D view (RViz)'),

        Node(package='touch_raw_driver', executable='touch_raw_driver',
             name='touch_raw_driver', output='screen', parameters=[driver_params]),

        Node(package='touch_teleop', executable='twist_to_target',
             name='twist_to_target', output='screen', parameters=[teleop_params]),

        Node(package='touch_teleop', executable='touch_dashboard', name='touch_dashboard',
             output='screen', parameters=[teleop_params],
             condition=IfCondition(LaunchConfiguration('dashboard'))),

        # Frames: world (the Franka will be added here later) -> touch_base (frame of
        # /touch/pose) -> touch/base (base of the 3D model, rotated by 180 deg about z).
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='world_to_touch_base',
             arguments=['--frame-id', 'world', '--child-frame-id', 'touch_base']),
    ]

    try:
        urdf = os.path.join(get_package_share_directory('omni_description'),
                            'urdf', 'omni.urdf.xacro')
        robot_description = colored_robot_description(urdf)
        actions += [
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='touch_base_to_model',
                 arguments=['--x', '0.0', '--y', '0.1684', '--z', '-0.1028',
                            '--yaw', '3.14159265', '--frame-id', 'touch_base',
                            '--child-frame-id', 'touch/base'],
                 condition=rviz_on),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace='touch', name='robot_state_publisher',
                 parameters=[{'robot_description': robot_description,
                              'frame_prefix': 'touch/'}],
                 remappings=[('joint_states', '/touch/model_joint_states')],
                 condition=rviz_on),
            Node(package='touch_teleop', executable='touch_model_bridge',
                 name='touch_model_bridge', condition=rviz_on),
        ]
    except PackageNotFoundError:
        actions.append(LogInfo(msg='omni_description not found: 3D model of the Touch '
                                   'disabled (see third_party/geomagic_touch_ros2).'))

    actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                        arguments=['-d', rviz_config], condition=rviz_on))
    return LaunchDescription(actions)
