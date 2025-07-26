from setuptools import setup, find_packages

setup(
    name='vision',
    version='0.0.0',
    packages=find_packages(include=['vision', 'vision.*']),
    install_requires=[],
    entry_points={
        'console_scripts': [
            'detector = vision.detector:main', #custom model
            #'detector = vision.obj_det:main', #preoptimized zed sample model
        ],
    },
)