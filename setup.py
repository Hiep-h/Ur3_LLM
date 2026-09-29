import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3_llm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools', 'openai'],
    zip_safe=True,
    maintainer='hiep',
    maintainer_email='vuvanhiep0510@gmail.com',
    description='LLM-based skill planning control for UR3/UR3e via MoveIt2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_publisher = ur3_llm_control.scene_publisher:main',
            'skill_executor = ur3_llm_control.skill_executor:main',
        ],
    },
)
