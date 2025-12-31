from setuptools import find_packages, setup

package_name = 'chinorobo_ros2_opencv_03'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'image_proc_node = chinorobo_ros2_opencv_03.image_proc_node:main',
        'color2grayscale_node = chinorobo_ros2_opencv_03.color2grayscale:main',
        'face_detection_node = chinorobo_ros2_opencv_03.face_detection:main',
        'face_mosaic_node = chinorobo_ros2_opencv_03.face_mosaic:main',
        ],
    },
)
