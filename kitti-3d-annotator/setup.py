from setuptools import setup, find_packages

setup(
    name='kitti_3da',
    version='0.1.0',
    packages=find_packages(include=['annotator', 'annotator.*']),
    entry_points={
        'console_scripts': ['view_kitti=view_kitti:main', 'view_kitti1=view_kittibin:main']
    }
)
