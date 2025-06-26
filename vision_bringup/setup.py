from setuptools import find_packages, setup

package_name = 'vision_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # keep ament registration
        ('share/ament_index/resource_index/packages',
        ['resource/vision_bringup']),
        # install package.xml
        ('share/vision_bringup', ['package.xml']),
        # install all launch files
        ('share/vision_bringup/launch', ['launch/obstacle_launch.py']),
        ('share/vision_bringup/launch', ['launch/grad_final_vis.py']),
        # ✅ 모델 2개를 한꺼번에 등록
        ('share/' + package_name + '/model', [
            'model/obstacle_detector.pt',
            'model/best_model.pth'
        ]),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loe',
    maintainer_email='ldm92834334@gmail.com',
    description='This package is about vision module that process vision data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lanenet_process = vision_bringup.lanenet_process:main',
            'pcd_process = vision_bringup.pcd_processs:main',
            'yolo_process = vision_bringup.yolo_process:main',
            'obstacle_detector = vision_bringup.obstacle_detector:main',
            'obstacle_detector_crosswalk = vision_bringup.obstacle_detector_crosswalk:main',
            'slic_process = vision_bringup.slic_process:main',
        ],
    },
)
