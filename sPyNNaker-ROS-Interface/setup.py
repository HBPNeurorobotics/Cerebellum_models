from setuptools import setup

setup(name='ros_spinnaker_interface',
      version='0.1',
      description='Connect SpiNNaker with ROS and use transfer functions to convert between live spikes and ROS values.',
      url='https://github.com/reiths/ros_spinnaker_interface',
      author='Stephan Reith',
      author_email='stephan.reith@tum.de',
      license='MIT',
      packages=['ros_spinnaker_interface'],
      zip_safe=False)
