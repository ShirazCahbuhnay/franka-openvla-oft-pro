import os
import tempfile
from string import Template

import yaml


def get_package_share_path():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_yaml_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def _require_vector(name, values, length):
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f'{name} must contain exactly {length} values')
    return [float(value) for value in values]


def _require_positive_float(name, value):
    numeric_value = float(value)
    if numeric_value <= 0.0:
        raise ValueError(f'{name} must be greater than zero')
    return numeric_value


def _require_positive_int(name, value):
    numeric_value = int(value)
    if numeric_value <= 0:
        raise ValueError(f'{name} must be greater than zero')
    return numeric_value


def _format_vector(values):
    return ' '.join(f'{float(value):g}' for value in values)


def _load_scene_object(name, object_config, *, default_shape, default_static):
    if not isinstance(object_config, dict):
        raise ValueError(f'{name} must be a mapping')

    scene_object = dict(object_config)
    scene_object['frame_id'] = str(scene_object.get('frame_id', 'world'))
    scene_object['object_id'] = str(scene_object.get('object_id', name))
    scene_object['shape'] = str(scene_object.get('shape', default_shape)).lower()
    if scene_object['shape'] not in {'box', 'cylinder'}:
        raise ValueError(f"{name}.shape must be either 'box' or 'cylinder'")

    ignored_collision_links = scene_object.get('ignored_collision_links', [])
    if not isinstance(ignored_collision_links, list):
        raise ValueError(f'{name}.ignored_collision_links must be a list')
    scene_object['ignored_collision_links'] = [
        str(link_name) for link_name in ignored_collision_links if str(link_name)
    ]

    dimensions_length = 3 if scene_object['shape'] == 'box' else 2
    scene_object['dimensions'] = _require_vector(
        f'{name}.dimensions',
        scene_object.get('dimensions'),
        dimensions_length,
    )
    scene_object['position'] = _require_vector(
        f'{name}.position',
        scene_object.get('position'),
        3,
    )
    scene_object['orientation_rpy'] = _require_vector(
        f'{name}.orientation_rpy',
        scene_object.get('orientation_rpy', [0.0, 0.0, 0.0]),
        3,
    )

    scene_object['static'] = bool(scene_object.get('static', default_static))
    scene_object['mass'] = _require_positive_float(
        f'{name}.mass',
        scene_object.get('mass', 0.05),
    )

    material = scene_object.get('material', {})
    if not isinstance(material, dict):
        raise ValueError(f'{name}.material must be a mapping')
    material['ambient'] = _require_vector(
        f'{name}.material.ambient',
        material.get('ambient', [0.45, 0.32, 0.22, 1.0]),
        4,
    )
    material['diffuse'] = _require_vector(
        f'{name}.material.diffuse',
        material.get('diffuse', [0.55, 0.39, 0.27, 1.0]),
        4,
    )
    material['specular'] = _require_vector(
        f'{name}.material.specular',
        material.get('specular', [0.1, 0.1, 0.1, 1.0]),
        4,
    )
    scene_object['material'] = material
    return scene_object


def _load_scene_camera(name, camera_config, *, default_enabled):
    if not isinstance(camera_config, dict):
        raise ValueError(f'{name} must be a mapping')

    camera = dict(camera_config)
    camera['enabled'] = bool(camera.get('enabled', default_enabled))
    camera['model_name'] = str(camera.get('model_name', name))
    camera['frame_id'] = str(camera.get('frame_id', f'{camera["model_name"]}_frame'))
    camera['optical_frame_id'] = str(
        camera.get('optical_frame_id', f'{camera["model_name"]}_optical_frame')
    )
    camera['gz_topic'] = str(camera.get('gz_topic', f'/{camera["model_name"]}/image'))
    camera['gz_camera_info_topic'] = str(
        camera.get('gz_camera_info_topic', f'/{camera["model_name"]}/camera_info')
    )
    camera['ros_image_topic'] = str(
        camera.get('ros_image_topic', f'/{camera["model_name"]}/image_raw')
    )
    camera['ros_camera_info_topic'] = str(
        camera.get('ros_camera_info_topic', f'/{camera["model_name"]}/camera_info')
    )
    camera['position'] = _require_vector(
        f'{name}.position',
        camera.get('position', [1.05, -0.85, 1.10]),
        3,
    )
    camera['orientation_rpy'] = _require_vector(
        f'{name}.orientation_rpy',
        camera.get('orientation_rpy', [0.0, 0.70, 2.15]),
        3,
    )

    image_size = camera.get('image_size', [640, 480])
    if not isinstance(image_size, list) or len(image_size) != 2:
        raise ValueError(f'{name}.image_size must contain exactly 2 values')
    camera['image_width'] = _require_positive_int(f'{name}.image_size[0]', image_size[0])
    camera['image_height'] = _require_positive_int(f'{name}.image_size[1]', image_size[1])

    camera['horizontal_fov'] = _require_positive_float(
        f'{name}.horizontal_fov',
        camera.get('horizontal_fov', 1.05),
    )
    camera['near_clip'] = _require_positive_float(
        f'{name}.near_clip',
        camera.get('near_clip', 0.05),
    )
    camera['far_clip'] = _require_positive_float(
        f'{name}.far_clip',
        camera.get('far_clip', 4.0),
    )
    if camera['far_clip'] <= camera['near_clip']:
        raise ValueError(f'{name}.far_clip must be greater than {name}.near_clip')
    camera['update_rate'] = _require_positive_float(
        f'{name}.update_rate',
        camera.get('update_rate', 15.0),
    )
    camera['visualize'] = bool(camera.get('visualize', False))
    return camera


def get_table_scene_config_file():
    return os.path.join(
        get_package_share_path(),
        'config',
        'table_scene.yaml',
    )


def load_table_scene_config():
    config = _load_yaml_file(get_table_scene_config_file())
    if not isinstance(config, dict):
        raise ValueError('table_scene.yaml must contain a mapping at the top level')

    if 'table' not in config:
        raise ValueError("table_scene.yaml must define a 'table' mapping")
    config['world_name'] = str(config.get('world_name', 'panda_table'))
    config['table'] = _load_scene_object(
        'table',
        config['table'],
        default_shape='box',
        default_static=True,
    )
    config['scene_camera'] = _load_scene_camera(
        'scene_camera',
        config.get('scene_camera', {}),
        default_enabled=False,
    )
    if 'pick_object' in config:
        config['pick_object'] = _load_scene_object(
            'pick_object',
            config['pick_object'],
            default_shape='cylinder',
            default_static=False,
        )
    return config


def get_scene_object_collision_parameters(config, object_key):
    scene_object = config[object_key]
    return {
        'frame_id': scene_object['frame_id'],
        'object_id': scene_object['object_id'],
        'shape': scene_object['shape'],
        'ignored_collision_link_names': ','.join(scene_object['ignored_collision_links']),
        'dimensions': scene_object['dimensions'],
        'position': scene_object['position'],
        'orientation_rpy': scene_object['orientation_rpy'],
    }


def get_scene_object_collision_parameter_list(config=None):
    table_scene = config or load_table_scene_config()
    object_keys = ['table']
    if 'pick_object' in table_scene:
        object_keys.append('pick_object')
    return [
        get_scene_object_collision_parameters(table_scene, object_key)
        for object_key in object_keys
    ]


def _render_geometry(scene_object):
    if scene_object['shape'] == 'box':
        return f'<box><size>{_format_vector(scene_object["dimensions"])}</size></box>'

    height, radius = scene_object['dimensions']
    return f'<cylinder><radius>{radius:g}</radius><length>{height:g}</length></cylinder>'


def _render_inertial(scene_object):
    if scene_object['static']:
        return ''

    mass = scene_object['mass']
    if scene_object['shape'] == 'box':
        size_x, size_y, size_z = scene_object['dimensions']
        ixx = mass * (size_y * size_y + size_z * size_z) / 12.0
        iyy = mass * (size_x * size_x + size_z * size_z) / 12.0
        izz = mass * (size_x * size_x + size_y * size_y) / 12.0
    else:
        height, radius = scene_object['dimensions']
        ixx = mass * (3.0 * radius * radius + height * height) / 12.0
        iyy = ixx
        izz = 0.5 * mass * radius * radius

    return (
        '        <inertial>\n'
        '          <pose>0 0 0 0 0 0</pose>\n'
        f'          <mass>{mass:g}</mass>\n'
        '          <inertia>\n'
        f'            <ixx>{ixx:g}</ixx>\n'
        '            <ixy>0</ixy>\n'
        '            <ixz>0</ixz>\n'
        f'            <iyy>{iyy:g}</iyy>\n'
        '            <iyz>0</iyz>\n'
        f'            <izz>{izz:g}</izz>\n'
        '          </inertia>\n'
        '        </inertial>\n'
    )


def _render_scene_model(scene_object):
    geometry = _render_geometry(scene_object)
    pose = _format_vector(scene_object['position'] + scene_object['orientation_rpy'])
    ambient = _format_vector(scene_object['material']['ambient'])
    diffuse = _format_vector(scene_object['material']['diffuse'])
    specular = _format_vector(scene_object['material']['specular'])
    inertial = _render_inertial(scene_object)
    static_text = 'true' if scene_object['static'] else 'false'

    return (
        f'    <model name="{scene_object["object_id"]}">\n'
        f'      <static>{static_text}</static>\n'
        f'      <pose>{pose}</pose>\n'
        '      <link name="link">\n'
        f'{inertial}'
        '        <collision name="collision">\n'
        '          <geometry>\n'
        f'            {geometry}\n'
        '          </geometry>\n'
        '          <surface>\n'
        '            <friction>\n'
        '              <ode>\n'
        '                <mu>1.0</mu>\n'
        '                <mu2>1.0</mu2>\n'
        '              </ode>\n'
        '            </friction>\n'
        '          </surface>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry>\n'
        f'            {geometry}\n'
        '          </geometry>\n'
        '          <material>\n'
        f'            <ambient>{ambient}</ambient>\n'
        f'            <diffuse>{diffuse}</diffuse>\n'
        f'            <specular>{specular}</specular>\n'
        '          </material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    )


def _render_scene_camera_model(scene_camera):
    if not scene_camera['enabled']:
        return ''

    pose = _format_vector(scene_camera['position'] + scene_camera['orientation_rpy'])
    visualize = 'true' if scene_camera['visualize'] else 'false'

    return (
        f'    <model name="{scene_camera["model_name"]}">\n'
        '      <static>true</static>\n'
        f'      <pose>{pose}</pose>\n'
        '      <link name="link">\n'
        '        <collision name="body_collision">\n'
        '          <geometry>\n'
        '            <box><size>0.08 0.08 0.08</size></box>\n'
        '          </geometry>\n'
        '        </collision>\n'
        '        <visual name="body_visual">\n'
        '          <geometry>\n'
        '            <box><size>0.08 0.08 0.08</size></box>\n'
        '          </geometry>\n'
        '          <material>\n'
        '            <ambient>0.12 0.12 0.16 1.0</ambient>\n'
        '            <diffuse>0.16 0.16 0.22 1.0</diffuse>\n'
        '            <specular>0.05 0.05 0.05 1.0</specular>\n'
        '          </material>\n'
        '        </visual>\n'
        f'        <sensor name="{scene_camera["model_name"]}_sensor" type="camera">\n'
        '          <always_on>1</always_on>\n'
        f'          <update_rate>{scene_camera["update_rate"]:g}</update_rate>\n'
        f'          <visualize>{visualize}</visualize>\n'
        f'          <topic>{scene_camera["gz_topic"]}</topic>\n'
        '          <camera>\n'
        f'            <camera_info_topic>{scene_camera["gz_camera_info_topic"]}</camera_info_topic>\n'
        f'            <optical_frame_id>{scene_camera["optical_frame_id"]}</optical_frame_id>\n'
        f'            <horizontal_fov>{scene_camera["horizontal_fov"]:g}</horizontal_fov>\n'
        '            <image>\n'
        f'              <width>{scene_camera["image_width"]}</width>\n'
        f'              <height>{scene_camera["image_height"]}</height>\n'
        '              <format>R8G8B8</format>\n'
        '            </image>\n'
        '            <clip>\n'
        f'              <near>{scene_camera["near_clip"]:g}</near>\n'
        f'              <far>{scene_camera["far_clip"]:g}</far>\n'
        '            </clip>\n'
        '          </camera>\n'
        '        </sensor>\n'
        '      </link>\n'
        '    </model>\n'
    )


def generate_table_world_file(config=None):
    table_scene = config or load_table_scene_config()
    template_path = os.path.join(get_package_share_path(), 'worlds', 'panda_table.sdf')
    template_text = Template(_load_yaml_or_text(template_path))

    scene_objects = [table_scene['table']]
    if 'pick_object' in table_scene:
        scene_objects.append(table_scene['pick_object'])

    substitutions = {
        'world_name': table_scene['world_name'],
        'scene_models': ''.join(_render_scene_model(scene_object) for scene_object in scene_objects),
        'scene_camera_model': _render_scene_camera_model(table_scene['scene_camera']),
    }
    rendered_world = template_text.substitute(substitutions)

    output_dir = os.path.join(tempfile.gettempdir(), 'panda_gazebo_bringup')
    os.makedirs(output_dir, exist_ok=True)
    world_file = os.path.join(output_dir, 'panda_table.generated.sdf')
    with open(world_file, 'w', encoding='utf-8') as file:
        file.write(rendered_world)
    return world_file


def _load_yaml_or_text(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        return file.read()
