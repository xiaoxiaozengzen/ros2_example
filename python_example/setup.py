import os
from glob import glob

from setuptools import setup

package_name = 'python_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # 获取launch.py文件
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
        (os.path.join("share", package_name), glob("launch/*.launch")),
        # 添加launch文件
        (os.path.join("share", package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='yanduochao@megvii.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # 节点的二进制 和 对应的main的映射
    entry_points={
        'console_scripts': [
            "python_example_test1_exe = python_example.test1:main",
            "python_example_test2_exe = python_example.test2:main",
        ],
    },
)
