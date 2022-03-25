from setuptools import setup

import os
from glob import glob

package_name = 'pfh_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # During installation, we need to copy the launch files
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('lib/python3.8/site-packages', package_name, "savedModel"), glob('savedModel/new/*.h5')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Beslin Tony Mathews',
    maintainer_email='beslin-tony.mathews@informatik.hs-fulda.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pfh_sub_1 = pfh_subscriber.pfh_subscribers_fn_node_1:main',
                'pfh_sub_2 = pfh_subscriber.pfh_subscribers_fn_node_2:main',
                'pfh_sub_3 = pfh_subscriber.pfh_subscribers_fn_node_3:main',
                'pfh_sub_4 = pfh_subscriber.pfh_subscribers_fn_node_4:main',
                'pfh_sub_5 = pfh_subscriber.pfh_subscribers_fn_node_5:main',
                'pfh_sub_6 = pfh_subscriber.pfh_subscribers_fn_node_6:main',
                'pfh_sub_7 = pfh_subscriber.pfh_subscribers_fn_node_7:main',
                'pfh_sub_8 = pfh_subscriber.pfh_subscribers_fn_node_8:main',
                'pfh_all_lstm_sub = pfh_subscriber.pfh_lstm_aggregator:main',
        ],
    },
)
