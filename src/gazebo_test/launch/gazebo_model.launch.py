import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


import launch_ros
from launch_ros.actions import Node
import xacro


# 

def generate_launch_description():
    

    

    # this name has to match the robot name in the Xacro file
    robotXacroName='Aruco_Marker'
    #testfile = 'simple_robot'
    
    # this is the name of our package, at the same time this is the name of the 
    # folder that will be used to define the paths
    namePackage = 'gazebo_test'
    package_name = 'opencv_tools'


    
    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'model/robot.xacro'
    #model2FileRelativePath = 'model/robot2.xacro'
    # this is a relative path to the Gazebo world file
    worldFileRelativePath = 'model/empty_world.world'
    
    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
   # pathModelFile2 = os.path.join(get_package_share_directory(namePackage),model2FileRelativePath)

    # this is the absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()
    #robotDescription2 = xacro.process_file(pathModelFile2).toxml()


    
    # this is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                                                       'launch','gazebo.launch.py'))
    
    # this is the launch description   
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'world': pathWorldFile}.items())
   
    
    # here, we create a gazebo_ros Node 
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py',
                          arguments=['-topic','robot_description','-entity', robotXacroName],output='screen')
    
    # spawnModelNode2 = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                       arguments=['-topic','robot_description','-entity', testfile],output='screen')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    
    #Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}] ,
        arguments = pathModelFile,
    )

    # nodeRobotStatePublisher2 = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robotDescription2,
    #     'use_sim_time': True}] ,
    #     arguments = pathModelFile2,
    #     )





    nodeJointStatePublisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen'
    # )
    
    PublishTransform =Node(
            package='opencv_tools',
            executable='tf_subscriber', 
            name='PublishTransform',
            output='screen',
        )
    


    # here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()
    
     
    # we add gazeboLaunch 
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    #launchDescriptionObject.add_action(spawnModelNode2)

    

    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    #launchDescriptionObject.add_action(nodeRobotStatePublisher2)
    launchDescriptionObject.add_action(nodeJointStatePublisher)

    launchDescriptionObject.add_action(PublishTransform)
    





    
    return launchDescriptionObject