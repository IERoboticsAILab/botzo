'''
Build the Gazebo (gz sim Harmonic) URDF from botzo_description's URDF.

The description URDF stays the single source of truth for links, joints and
meshes; this module only adds what the simulator needs:

  * servo-like joint limits (effort / velocity), so joints cannot move
    infinitely fast or strong,
  * cheap collision shapes: a box for the body and a small sphere at each toe
    (mesh-vs-mesh contacts are slow and jittery),
  * foot friction,
  * the gz_ros2_control hardware + plugin and an odometry publisher.
'''

import copy
import xml.etree.ElementTree as ET

LEGS = ('FL', 'FR', 'BL', 'BR')
JOINTS = [f'{leg}_{part}_joint' for leg in LEGS for part in ('shoulder', 'femur', 'tibia')]

# DS3225 servo: ~25 kg*cm stall torque (2.45 N*m), 0.13 s / 60 deg (~8 rad/s).
SERVO_EFFORT = 2.45
SERVO_VELOCITY = 8.0

# Joint ranges (rad, URDF convention). The gait uses roughly shoulder +-0.4,
# femur -1.36..-0.49, tibia -0.08..1.04; these add margin but stop a fallen
# robot from folding its legs through the chassis.
JOINT_LIMITS = {'shoulder': (-0.6, 0.6), 'femur': (-1.7, 0.1), 'tibia': (-0.4, 1.4)}

# Collision shapes, measured from the STL meshes (metres, link frames).
BODY_BOX_SIZE = (0.24, 0.156, 0.124)
BODY_BOX_CENTER = (0.0, 0.0, 0.062)
TOE_RADIUS = 0.008
TOE_CENTER = (-0.0016, -0.0066, -0.1024 + TOE_RADIUS)  # left legs (y > 0); mirrored for right legs
FOOT_FRICTION = 1.0


def _xyz(values):
    return ' '.join(f'{v:g}' for v in values)


def _remove_children(root, tag, keep=lambda el: False):
    for el in list(root.findall(tag)):
        if not keep(el):
            root.remove(el)


def _set_collision(link, geometry, center):
    for col in link.findall('collision'):
        link.remove(col)
    col = ET.SubElement(link, 'collision')
    ET.SubElement(col, 'origin', xyz=_xyz(center), rpy='0 0 0')
    geom = ET.SubElement(col, 'geometry')
    geom.append(geometry)


def _add_servo_limits(root):
    for joint in root.findall('joint'):
        for old in joint.findall('limit'):
            joint.remove(old)
        lower, upper = JOINT_LIMITS[joint.get('name').split('_')[1]]
        joint.set('type', 'revolute')
        ET.SubElement(joint, 'limit', lower=f'{lower:g}', upper=f'{upper:g}',
                      effort=f'{SERVO_EFFORT:g}', velocity=f'{SERVO_VELOCITY:g}')
        ET.SubElement(joint, 'dynamics', damping='0.02', friction='0.01')


def _simplify_collisions(root):
    for link in root.findall('link'):
        name = link.get('name')
        if name == 'base':
            _set_collision(link, ET.Element('box', size=_xyz(BODY_BOX_SIZE)), BODY_BOX_CENTER)
        elif name.endswith('_tibia'):
            x, y, z = TOE_CENTER
            y = y if name[1] == 'L' else -y
            _set_collision(link, ET.Element('sphere', radius=f'{TOE_RADIUS:g}'), (x, y, z))
        else:
            for col in link.findall('collision'):
                link.remove(col)


def _set_foot_friction(root):
    for leg in LEGS:
        ref = f'{leg}_tibia'
        gz = next((g for g in root.findall('gazebo') if g.get('reference') == ref), None)
        if gz is None:
            gz = ET.SubElement(root, 'gazebo', reference=ref)
        for tag in ('mu1', 'mu2'):
            el = gz.find(tag)
            if el is None:
                el = ET.SubElement(gz, tag)
            el.text = f'{FOOT_FRICTION:g}'


def _add_ros2_control(root, controllers_yaml, initial_positions):
    rc = ET.SubElement(root, 'ros2_control', name='BotzoGazebo', type='system')
    hw = ET.SubElement(rc, 'hardware')
    ET.SubElement(hw, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for joint in JOINTS:
        j = ET.SubElement(rc, 'joint', name=joint)
        ET.SubElement(j, 'command_interface', name='position')
        pos = ET.SubElement(j, 'state_interface', name='position')
        if joint in initial_positions:  # spawn in this pose instead of all zeros
            ET.SubElement(pos, 'param', name='initial_value').text = f'{initial_positions[joint]:.4f}'
        ET.SubElement(j, 'state_interface', name='velocity')

    gz = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gz, 'plugin', filename='gz_ros2_control-system',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    # The yaml also holds position_proportional_gain (see config/controllers.yaml).
    ET.SubElement(plugin, 'parameters').text = controllers_yaml


def _add_odometry(root):
    gz = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gz, 'plugin', filename='gz-sim-odometry-publisher-system',
                           name='gz::sim::systems::OdometryPublisher')
    for tag, text in (('odom_frame', 'odom'), ('robot_base_frame', 'base'),
                      ('dimensions', '3'), ('odom_publish_frequency', '50')):
        ET.SubElement(plugin, tag).text = text


def make_sim_urdf(description_urdf: str, controllers_yaml: str, initial_positions=None) -> str:
    '''
    Return the simulation URDF (as a string) for the given description URDF.
    initial_positions: optional {joint name: rad} pose to spawn in.
    '''
    root = copy.deepcopy(ET.fromstring(description_urdf))

    # Drop simulator/ROS 1 leftovers from the description: transmissions, any
    # existing ros2_control block and plugin <gazebo> tags. Per-link
    # <gazebo reference=...> tags are kept.
    _remove_children(root, 'transmission')
    _remove_children(root, 'ros2_control')
    _remove_children(root, 'gazebo', keep=lambda el: el.get('reference') is not None)

    _add_servo_limits(root)
    _simplify_collisions(root)
    _set_foot_friction(root)
    _add_ros2_control(root, controllers_yaml, initial_positions or {})
    _add_odometry(root)
    return ET.tostring(root, encoding='unicode')
