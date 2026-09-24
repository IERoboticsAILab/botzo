import os
import xml.etree.ElementTree as ET

from botzo_gazebo.sim_description import JOINTS, make_sim_urdf

HERE = os.path.dirname(__file__)
DESCRIPTION = os.path.join(HERE, '..', '..', 'botzo_description', 'urdf', 'botzo.urdf')


def sim_root():
    with open(DESCRIPTION) as f:
        return ET.fromstring(make_sim_urdf(f.read(), '/tmp/controllers.yaml'))


def test_every_joint_has_servo_limits_and_a_position_interface():
    root = sim_root()
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        assert joint.get('type') == 'revolute'
        assert float(limit.get('lower')) < 0.0 < float(limit.get('upper'))
        assert float(limit.get('effort')) > 0 and float(limit.get('velocity')) > 0
    rc_joints = [j.get('name') for j in root.find('ros2_control').findall('joint')]
    assert sorted(rc_joints) == sorted(JOINTS)
    assert sorted(JOINTS) == sorted(j.get('name') for j in root.findall('joint'))


def test_single_ros2_control_block_and_no_classic_plugins():
    root = sim_root()
    assert len(root.findall('ros2_control')) == 1
    assert root.find('ros2_control/hardware/plugin').text == 'gz_ros2_control/GazeboSimSystem'
    filenames = [p.get('filename') for p in root.iter('plugin') if p.get('filename')]
    assert 'gz_ros2_control-system' in filenames
    assert not any('libgazebo' in f for f in filenames)
    assert not root.findall('transmission')


def test_collisions_are_a_body_box_and_toe_spheres():
    root = sim_root()
    for link in root.findall('link'):
        shapes = [c.find('geometry')[0].tag for c in link.findall('collision')]
        name = link.get('name')
        if name == 'base':
            assert shapes == ['box']
        elif name.endswith('_tibia'):
            assert shapes == ['sphere']
        else:
            assert shapes == []


def test_feet_have_one_friction_block_each():
    root = sim_root()
    for leg in ('FL', 'FR', 'BL', 'BR'):
        blocks = [g for g in root.findall('gazebo') if g.get('reference') == f'{leg}_tibia']
        assert len(blocks) == 1
        assert float(blocks[0].find('mu1').text) >= 0.8


def test_left_legs_are_on_the_left():
    '''REP-103: x forward, y left. FL/BL hips must have y > 0, toes tucked toward the body.'''
    root = sim_root()
    joints = {j.get('name'): j for j in root.findall('joint')}
    for leg in ('FL', 'FR', 'BL', 'BR'):
        y = float(joints[f'{leg}_shoulder_joint'].find('origin').get('xyz').split()[1])
        assert (y > 0) == (leg[1] == 'L')
        toe_y = float(root.find(f"link[@name='{leg}_tibia']/collision/origin").get('xyz').split()[1])
        assert (toe_y < 0) == (leg[1] == 'L')


def test_initial_positions_become_initial_values():
    with open(DESCRIPTION) as f:
        root = ET.fromstring(make_sim_urdf(f.read(), '/tmp/c.yaml', {'FL_femur_joint': -0.9}))
    j = root.find("ros2_control/joint[@name='FL_femur_joint']")
    assert float(j.find("state_interface[@name='position']/param").text) == -0.9
