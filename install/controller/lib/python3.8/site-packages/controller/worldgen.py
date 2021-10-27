from os import makedirs

def world_init(world_name):
    return \
    world_name,[f'''
    <?xml version="1.0" ?>
    <sdf version="1.5">
    <world name="{world_name}">
    <include><uri>model://ground_plane</uri></include><include><uri>model://sun</uri></include>
    ''']

def add_model(world,model_name,model_path,model_pose=(0,0,0,0,0,0)):
    world[1].append(
    f'''
    <include>
        <uri>{model_path}</uri>
        <name>{model_name}</name>
        <pose>{model_pose[0]} {model_pose[1]} {model_pose[2]} {model_pose[3]} {model_pose[4]} {model_pose[5]}</pose>
    </include>
    ''')

def write_file(file_name,content):
    with open(file_name,'w') as f:
        f.write(content)

def get_world(world):
    world[1].append('</world></sdf>')
    _world = ''
    for component in world[1]:
        _world += component
    return _world

def save_world(world,save_path):
    try:
        makedirs(save_path)
    except:
        pass
    write_file(save_path+f'/{world[0]}.world',get_world(world))

def main():
    
    from mapgen import main as generate_map
    generate_map()

    from configuration_parser import read_conf
    c = read_conf()
    world = world_init(c['world_name'])
    add_model(world,c['map_name'],c['map_path']+'/'+c['map_name'])
    
    from robotgen import build_and_save_robot

    if type(c['spawn_robots']) == str:
        build_and_save_robot(c['spawn_robots'])
        add_model(world,c['spawn_robots'],c['robot_path']+'/'+c['spawn_robots'],c[f'spawn_pose_{c["spawn_robots"]}'])
    else:
        for robot in c['spawn_robots']:
            build_and_save_robot(robot)
            add_model(world,robot,c['robot_path']+'/'+robot,c[f'spawn_pose_{robot}'])

    save_world(world,c['world_path'])


if __name__ == '__main__':
    main()