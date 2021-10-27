from configuration_parser import read_conf
from os import makedirs

def robot_sdf_header():
    return \
    '''
    <?xml version="1.0" ?>
    <sdf version="1.5">
    '''
def robot_sdf(namespace,diff_drive,cmd_vel,odom,laser_out,pose):
    return \
    f'''
    <model name="customized_pioneer2dx">
        <pose> {pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]} </pose>
        <link name="chassis">
        <pose>0 0 0.16 0 0 0</pose>
        <inertial>
            <mass>5.67</mass>
            <inertia>
            <ixx>0.07</ixx>
            <iyy>0.08</iyy>
            <izz>0.10</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
            </inertia>
        </inertial>
        <collision name="collision">
            <geometry>
            <box>
                <size>0.445 0.277 0.17</size>
            </box>
            </geometry>
        </collision>
        <collision name="castor_collision">
            <pose>-0.200 0 -0.12 0 0 0</pose>
            <geometry>
            <sphere>
                <radius>0.04</radius>
            </sphere>
            </geometry>
            <surface>
            <friction>
                <ode>
                <mu>0</mu>
                <mu2>0</mu2>
                <slip1>1.0</slip1>
                <slip2>1.0</slip2>
                </ode>
            </friction>
            </surface>
        </collision>
        <visual name="visual">
            <pose>0 0 0.04 0 0 0</pose>
            <geometry>
            <mesh>
                <uri>model://pioneer2dx/meshes/chassis.dae</uri>
            </mesh>
            </geometry>
        </visual>
        <visual name="castor_visual">
            <pose>-0.200 0 -0.12 0 0 0</pose>
            <geometry>
            <sphere>
                <radius>0.04</radius>
            </sphere>
            </geometry>
            <material>
            <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/FlatBlack</name>
            </script>
            </material>
        </visual>
        </link>
        <link name="right_wheel">
        <pose>0.1 -.17 0.11 0 1.5707 1.5707</pose>
        <inertial>
            <mass>1.5</mass>
            <inertia>
            <ixx>0.0051</ixx>
            <iyy>0.0051</iyy>
            <izz>0.0090</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
            </inertia>
        </inertial>
        <collision name="collision">
            <geometry>
            <cylinder>
                <radius>0.11</radius>
                <length>0.05</length>
            </cylinder>
            </geometry>
            <surface>
            <friction>
                <ode>
                <mu>100000.0</mu>
                <mu2>100000.0</mu2>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
                </ode>
            </friction>
            </surface>
        </collision>
        <visual name="visual">
            <geometry>
            <cylinder>
                <radius>0.11</radius>
                <length>0.05</length>
            </cylinder>
            </geometry>
            <material>
            <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/FlatBlack</name>
            </script>
            </material>
        </visual>
        </link>
        <link name="left_wheel">
        <pose>0.1 .17 0.11 0 1.5707 1.5707</pose>
        <inertial>
            <mass>1.5</mass>
            <inertia>
            <ixx>0.0051</ixx>
            <iyy>0.0051</iyy>
            <izz>0.0090</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
            </inertia>
        </inertial>
        <collision name="collision">
            <geometry>
            <cylinder>
                <radius>0.11</radius>
                <length>0.05</length>
            </cylinder>
            </geometry>
            <surface>
            <friction>
                <ode>
                <mu>100000.0</mu>
                <mu2>100000.0</mu2>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
                </ode>
            </friction>
            </surface>
        </collision>
        <visual name="visual">
            <geometry>
            <cylinder>
                <radius>0.11</radius>
                <length>0.05</length>
            </cylinder>
            </geometry>
            <material>
            <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/FlatBlack</name>
            </script>
            </material>
        </visual>
        </link>
        <joint type="revolute" name="left_wheel_hinge">
        <pose>0 0 -0.03 0 0 0</pose>
        <child>left_wheel</child>
        <parent>chassis</parent>
        <axis>
            <xyz>0 1 0</xyz>
            <use_parent_model_frame>true</use_parent_model_frame>
        </axis>
        </joint>
        <joint type="revolute" name="right_wheel_hinge">
        <pose>0 0 0.03 0 0 0</pose>
        <child>right_wheel</child>
        <parent>chassis</parent>
        <axis>
            <xyz>0 1 0</xyz>
            <use_parent_model_frame>true</use_parent_model_frame>
        </axis>
        </joint>
        <plugin name='{diff_drive}' filename='libgazebo_ros_diff_drive.so'>
            <ros>
            <namespace>/{namespace}</namespace>
            <remapping>cmd_vel:={cmd_vel}</remapping>
            <remapping>odom:={odom}</remapping>
            </ros>
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <left_joint>left_wheel_hinge</left_joint>
        <right_joint>right_wheel_hinge</right_joint>
        <wheel_separation>0.39</wheel_separation>
        <wheel_diameter>0.15</wheel_diameter>
        <torque>5</torque>
        <command_topic>cmd_vel</command_topic>
        <odometry_topic>odom</odometry_topic>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>chassis</robot_base_frame>
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>
        </plugin>
        <!-- Add the laser range finder -->
        <link name="laser_link">
            <inertial>
            <!-- Mass of the laser range finder in kg -->
            <mass>0.1</mass>
            </inertial>
            <!-- Position is towards the front of the robot -->
            <!-- Laser finder is mounted on top -->
            <pose>0.15 0 0.30 0 0 0</pose>
            
            <!-- Add a mesh to make it more visually appealing -->
            <visual name="visual">
            <geometry>
                <mesh>
                <uri>model://hokuyo/meshes/hokuyo.dae</uri>
                </mesh>
            </geometry>
            </visual>
            
            <!-- Collision properties of the base of the laser range finder-->
            <collision name="collision-base">
            <pose>0 0 -0.0145 0 0 0</pose>
            <geometry>
                <box>
                <size>0.05 0.05 0.041</size>
                </box>
            </geometry>
            </collision>
            <!-- Collision properties of the top of the laser range finder-->
            <collision name="collision-top">
            <pose>0 0 0.0205 0 0 0</pose>
            <geometry>
                <cylinder>
                <radius>0.021</radius>
                <length>0.029</length>
                </cylinder>
            </geometry>
            </collision>
            
            <!-- Describes the type and properties of the sensor -->
        <sensor name="laser" type="ray">
        <pose>0.01 0 0.0175 0 -0 0</pose>
            <ray>
                <scan>
                <horizontal>
                    <samples>181</samples>
                    <resolution>1</resolution>
                    <min_angle>-1.57080</min_angle>
                    <max_angle>1.57080</max_angle>
                </horizontal>
                </scan>
                <range>
                <min>0.08</min>
                <max>10</max>
                <resolution>0.05</resolution>
                </range>
            </ray>
            <always_on>1</always_on>
            <update_rate>10</update_rate>
            <visualize>true</visualize>

            <plugin name='laser' filename='libgazebo_ros_ray_sensor.so'>
                <ros>
                <namespace>/{namespace}</namespace>
                <argument>--ros-args --remap ~/out:={laser_out}</argument>
                </ros>
                <output_type>sensor_msgs/LaserScan</output_type>
            </plugin>
        </sensor>
        </link>
        <!-- Connect laser range finder to the robot's body -->
        <joint type="fixed" name="laser_joint">
            <child>laser_link</child>
            <parent>chassis</parent>
        </joint>
    </model>
    '''

def robot_sdf_footer():
    return '</sdf>'

def robot_config(author_name,author_email):
    return f'''
    <?xml version="1.0"?>
    <model>
    <name>Customized Pioneer 2DX</name>
    <version>1.0</version>
    <sdf version="1.5">model.sdf</sdf>
    <author>
        <name>Nate Koenig, {author_name}</name>
        <email>nate@osrfoundation.org, {author_email}</email>
    </author>
    <description>
        A customized model of the pioneer 2 dx robot.
    </description>
    <depend>
    </depend>
    </model>
    '''


def write_file(file_name,content):
    with open(file_name,'w') as f:
        f.write(content)

def save_robot(robot_name,robot_config,robot_sdf,save_path):
    save_path += '/'+robot_name
    try:
        makedirs(save_path)
    except:
        pass
    write_file(save_path+'/model.config',robot_config)
    write_file(save_path+'/model.sdf',robot_sdf)

def build_robot(namespace,pose):
    c = read_conf()
    cmd_vel = c['robot_velocity']
    odom = c['robot_odometery']
    diff_drive = c['robot_diff_drive']
    laser_out = c['robot_laser_out']
    return robot_sdf(namespace,diff_drive,cmd_vel,odom,laser_out,pose)

def build_and_save_robot(namespace,pose=(0,0,0,0,0,0)):
    c = read_conf()
    author_name = c['author_name'][0] + ' ' + c['author_name'][1]
    author_email = c['author_email']
    model_path = c['robot_path']
    save_robot(namespace,robot_config(author_name,author_email),robot_sdf_header()+build_robot(namespace,pose)+robot_sdf_footer(),model_path)

if __name__ == '__main__':
    
    c = read_conf()
    namespace = c['robot_namespace']
    build_and_save_robot(namespace)