from setuptools import find_packages, setup

package_name = 'application'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=[
	    'stt_node',
	    'tts_node',
	    'intent_node',
	    'vito_stt_client_pb2',
	    'vito_stt_client_pb2_grpc',
        'button_node',
        'keyboard_node',
        'hmi_planning_node'
	],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/application_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='junyeong4321@gmail.com',
    description='Voice control system with STT and TTS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
	    'console_scripts': [
		'stt_node = stt_node:main',
		'tts_node = tts_node:main',
		'intent_node = intent_node:main',
        'button_node = button_node:main',
        'keyboard_node = keyboard_node:main',
        'hmi_planning_node = hmi_planning_node:main'
        ],
    },
)

