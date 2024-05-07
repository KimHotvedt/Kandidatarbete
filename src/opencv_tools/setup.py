from setuptools import find_packages, setup

package_name = 'opencv_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimhotvedt',
    maintainer_email='kimhot@chalmers.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = opencv_tools.basic_image_publisher:main',
            'img_subscriber = opencv_tools.basic_image_subscriber:main',
            'aruco_marker_pose_estimation_tf = opencv_tools.aruco_marker_pose_estimation_tf:main',
            'tf_subscriber = opencv_tools.tf_subscriber:main'

        ],
    },
)
