from setuptools import setup

package_name = 'place_markers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='comp3431',
    maintainer_email='z5310070@ad.unsw.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'place_markers = place_markers.webcam_pub:main',
            'webcam = place_markers.webcam_sub:main',
        ],
    },
)
