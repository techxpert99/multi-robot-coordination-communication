from os import makedirs
import numpy as np
from numpy.lib.npyio import save
from numpy.random.mtrand import randint, seed
from robotgen import build_robot
from obstaclegen import spawn_obstacles
from configuration_parser import read_conf

def syn_model(model_name):
    return f'<?xml version="1.0"?><sdf version="1.7"><model name="{model_name}"><static>1</static>'

def pose(pose_tuple):
    return f'<pose>{pose_tuple[0]} {pose_tuple[1]} {pose_tuple[2]} {pose_tuple[3]} {pose_tuple[4]} {pose_tuple[5]}</pose>'

def geometry(geometry_type,dim1,dim2=None,dim3=None):
    if geometry_type == 'box':
        return f'<geometry><box><size>{dim1} {dim2} {dim3}</size></box></geometry>'
    elif geometry_type == 'cylinder':
        return f'<geometry><cylinder><radius>{dim1}</radius><length>{dim2}</length></cylinder></geometry>'
    elif geometry_type == 'sphere':
        return f'<geometry><sphere><radius>{dim1}</radius></sphere></geometry>'

def material(material_name):
	return f'<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/{material_name}</name></script></material>'

def inertial(inertial_name,pose,mass,inertia):
    return f'<inertial>{pose}<mass>{mass}</mass><inertia><ixx>{inertia[0,0]}</ixx><ixy>{inertia[0,1]}</ixy><ixz>{inertia[0,2]}</ixz><iyx>{inertia[1,0]}</iyx><iyy>{inertia[1,1]}</iyy><iyz>{inertia[1,2]}</iyz><izx>{inertia[2,0]}</izx><izy>{inertia[2,1]}</izy><izz>{inertia[2,2]}</izz></inertia></inertial>'

def visual(visual_name,pose,geometry,material=''):
    return f'<visual name="{visual_name}">{pose}{geometry}{material}</visual>'

def collision(collision_name,pose,geometry):
    return f'<collision name="{collision_name}">{pose}{geometry}</collision>'

def link(link_name,pose,inertial,visual,collision):
    return f'<link name="{link_name}">{pose}{inertial}{visual}{collision}</link>'

def joint(joint_name,type,parent,child,axis=None):
    if axis is None:
        return f'<joint name="{joint_name}" type="{type}"><parent>{parent}</parent><child>{child}</child></joint>'
    return f'<joint name="{joint_name}" type="{type}"><parent>{parent}</parent><child>{child}</child><axis><xyz>{axis}</xyz></joint>'

def fin_model():
    return '</model></sdf>'

def cuboid_link(link_name,pose_tuple,size_tuple,mass_value,material_name=''):
    _inertia = np.array([[(size_tuple[1]**2+size_tuple[2]**2)/12,0,0],[0,(size_tuple[0]**2+size_tuple[2]**2)/12,0],[0,0,(size_tuple[1]**2+size_tuple[0]**2)/12]])*mass_value
    _pose = pose(pose_tuple)
    _inertial = inertial(f'{link_name:}::inertial',pose((0,0,0,0,0,0)),mass_value,_inertia)
    _visual = visual(f'{link_name}::visual',pose((0,0,0,0,0,0)),geometry('box',size_tuple[0],size_tuple[1],size_tuple[2]),material(material_name))
    _collision = collision(f'{link_name}::collision',pose((0,0,0,0,0,0)),geometry('box',size_tuple[0],size_tuple[1],size_tuple[2]))
    return link(link_name,_pose,_inertial,_visual,_collision)

def cylinder_link(link_name,pose_tuple,size_tuple,mass_value,material_name=''):
    height_inertia = mass_value*size_tuple[1]*size_tuple[1]/12
    radius_inertia = mass_value*size_tuple[0]*size_tuple[0]/4
    _inertia = np.array([[height_inertia+radius_inertia,0,0],[0,height_inertia+radius_inertia,0],[0,0,2*radius_inertia]])
    _pose = pose(pose_tuple)
    _inertial = inertial(f'{link_name:}::inertial',pose((0,0,0,0,0,0)),mass_value,_inertia)
    _visual = visual(f'{link_name}::visual',pose((0,0,0,0,0,0)),geometry('cylinder',size_tuple[0],size_tuple[1]),material(material_name))
    _collision = collision(f'{link_name}::collision',pose((0,0,0,0,0,0)),geometry('cylinder',size_tuple[0],size_tuple[1]))
    return link(link_name,_pose,_inertial,_visual,_collision)

def spherical_link(link_name,pose_tuple,size_tuple,mass_value,material_name=''):
    radius_inertia = 2*mass_value*size_tuple[0]*size_tuple[0]/5
    _inertia = np.array([[1,0,0],[0,1,0],[0,0,1]])*radius_inertia
    _pose = pose(pose_tuple)
    _inertial = inertial(f'{link_name:}::inertial',pose((0,0,0,0,0,0)),mass_value,_inertia)
    _visual = visual(f'{link_name}::visual',pose((0,0,0,0,0,0)),geometry('sphere',size_tuple[0],size_tuple[1]),material(material_name))
    _collision = collision(f'{link_name}::collision',pose((0,0,0,0,0,0)),geometry('sphere',size_tuple[0],size_tuple[1]))
    return link(link_name,_pose,_inertial,_visual,_collision)

def init_model(model_name): return model_name,[syn_model(model_name)]

def add(model,component):
    model[1].append(component)

def get_model(model):
    model[1].append(fin_model())
    _model = ''
    for component in model[1]:
        _model += component
    return _model 

def gravity(gravity):
    return f'<gravity>{gravity}</gravity>'

def model_config(model_name,author_name,author_email):
    return f'''
    <?xml version="1.0"?>
        <model>
            <name>{model_name}</name>
            <version>1.0</version>
            <sdf version="1.7">model.sdf</sdf>
            <author>
                <name>{author_name}</name>
                <email>{author_email}</email>
            </author>
            <description>
                Randomly generated world.
            </description>
        </model>
    '''

def write_file(file_name,content):
    with open(file_name,'w') as f:
        f.write(content)

def save_model(model,save_path,author_name,author_email):
    save_path += '/'+model[0]
    try:
        makedirs(save_path)
    except:
        pass
    write_file(save_path+'/model.config',model_config(model[0],author_name,author_email))
    write_file(save_path+'/model.sdf',get_model(model))

def main():
    c = read_conf()

    arena_stretch_x,arena_stretch_y = c['arena']
    ground_thickness = c['ground_thickness']
    wall_thickness = c['wall_thickness']
    wall_height = c['wall_height']
    heavy_mass = c['heavy_mass']
    ground_material,wall_material = c['ground_material'],c['wall_material']
    obstacle_materials = c['obstacle_materials']
    num_obstacles = c['num_obstacles']
    side_bounds = c['side_bounds']
    radius_bounds = c['radius_bounds']
    num_retries = c['num_retries']
    author_name = c['author_name'][0] + ' ' + c['author_name'][1]
    author_email = c['author_email']

    ground_link = cuboid_link('arena_frame::ground_link',(0,0,ground_thickness/2,0,0,0),(arena_stretch_x+2*wall_thickness,arena_stretch_y+2*wall_thickness,ground_thickness),heavy_mass,ground_material)

    wall_yz_front_link = cuboid_link('arena_frame::wall_yz_front_link',((arena_stretch_x+wall_thickness)/2,0,wall_height/2+ground_thickness,0,0,0),(wall_thickness,arena_stretch_y,wall_height),heavy_mass,wall_material)
    wall_yz_back_link = cuboid_link('arena_frame::wall_yz_back_link',((-arena_stretch_x-wall_thickness)/2,0,wall_height/2+ground_thickness,0,0,0),(wall_thickness,arena_stretch_y,wall_height),heavy_mass,wall_material)
    wall_xz_front_link = cuboid_link('arena_frame::wall_xz_front_link',(0,(arena_stretch_y+wall_thickness)/2,wall_height/2+ground_thickness,0,0,0),(arena_stretch_x,wall_thickness,wall_height),heavy_mass,wall_material)
    wall_xz_back_link = cuboid_link('arena_frame::wall_xz_back_link',(0,(-arena_stretch_y-wall_thickness)/2,wall_height/2+ground_thickness,0,0,0),(arena_stretch_x,wall_thickness,wall_height),heavy_mass,wall_material)

    wall_yz_front_joint = joint('arena_frame::wall_yz_front_ground_joint','fixed','arena_frame::ground_link','arena_frame::wall_yz_front_link')
    wall_yz_back_joint = joint('arena_frame::wall_yz_back_ground_joint','fixed','arena_frame::ground_link','arena_frame::wall_yz_back_link')
    wall_xz_front_joint = joint('arena_frame::wall_xz_front_ground_joint','fixed','arena_frame::ground_link','arena_frame::wall_xz_front_link')
    wall_xz_back_joint = joint('arena_frame::wall_xz_back_ground_joint','fixed','arena_frame::ground_link','arena_frame::wall_xz_back_link')

    model = init_model(c['map_name'])
    add(model,ground_link)
    add(model,wall_yz_front_link)
    add(model,wall_yz_back_link)
    add(model,wall_xz_front_link)
    add(model,wall_xz_back_link)
    add(model,wall_yz_front_joint)
    add(model,wall_yz_back_joint)
    add(model,wall_xz_front_joint)
    add(model,wall_xz_back_joint)

    seed(c['seed']%(2**32))
    obstacles = spawn_obstacles(num_obstacles,side_bounds,radius_bounds,(-arena_stretch_x/2,-arena_stretch_y/2,arena_stretch_x/2,arena_stretch_y/2),num_retries)

    def choose_obstacle_material():
        return obstacle_materials[randint(0,len(obstacle_materials)-1)]

    for i,obstacle in enumerate(obstacles):
        _type,(cx,cy),dimension = obstacle
        if _type == 'cube':
            ln = cuboid_link(f'arena_obstacles::obstacle-{i+1}',(cx,cy,ground_thickness+dimension/2,0,0,0),(dimension,dimension,dimension),heavy_mass,choose_obstacle_material())
        elif _type == 'cylinder':
            ln = cylinder_link(f'arena_obstacles::obstacle-{i+1}',(cx,cy,ground_thickness+dimension,0,0,0),(dimension,2*dimension,None),heavy_mass,choose_obstacle_material())    
        elif _type == 'sphere':
            ln = spherical_link(f'arena_obstacles::obstacle-{i+1}',(cx,cy,ground_thickness+dimension,0,0,0),(dimension,None,None),heavy_mass,choose_obstacle_material())    
        add(model,ln)
    
    save_model(model,c['map_path'],author_name,author_email)

if __name__ == '__main__':
    main()