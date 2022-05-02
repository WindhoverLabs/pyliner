from setuptools import setup, find_packages

setup(
    name='pyliner',
    version='1.0.1',
    author='Erik VanderWerf',
    author_email='evanderwerf@windhoverlabs.com',
    description='Interface to control drones through Airliner',
    url='windhoverlabs.com',
    license='3BSD-3-Clause',
    packages=find_packages(),
    install_requires=[
        'enum34',
        'future',
        'geographiclib',
        'orderedset',
        'protobuf',
        'sortedcontainers'
    ],
    entry_points={
        'console_scripts': [
            'pyliner = pyliner.__main__:main'
        ]
    }
)
