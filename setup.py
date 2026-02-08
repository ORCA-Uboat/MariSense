"""
MariSense: A Multimodal Dataset for Environmental Perception in Maritime Autonomy

Setup script for installing the MariSense package and tools.
"""

from setuptools import setup, find_packages
import os

# Read the contents of README file
this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# Read requirements
with open(os.path.join(this_directory, 'requirements.txt'), encoding='utf-8') as f:
    requirements = [line.strip() for line in f if line.strip() and not line.startswith('#')]

setup(
    name='marisense',
    version='1.0.0',
    author='[Author Name]',
    author_email='[author@email.com]',
    description='A Multimodal Dataset for Environmental Perception in Maritime Autonomy',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/[username]/MariSense',
    project_urls={
        'Bug Reports': 'https://github.com/[username]/MariSense/issues',
        'Source': 'https://github.com/[username]/MariSense',
        'Documentation': 'https://[username].github.io/MariSense',
    },
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Image Recognition',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
    python_requires='>=3.8',
    install_requires=requirements,
    extras_require={
        'dev': [
            'pytest>=6.2.0',
            'black>=21.0',
            'flake8>=3.9.0',
            'mypy>=0.910',
        ],
        'docs': [
            'sphinx>=4.0.0',
            'sphinx-rtd-theme>=0.5.0',
        ],
        'ros': [
            'rospy',
            'sensor_msgs',
            'cv_bridge',
        ],
        'all': [
            'ultralytics>=8.0.0',
            'open3d>=0.13.0',
            'motmetrics>=1.2.0',
        ],
    },
    entry_points={
        'console_scripts': [
            'marisense=marisense.cli:main',
        ],
    },
    include_package_data=True,
    package_data={
        'marisense': ['configs/*.yaml', 'configs/*.json'],
    },
    keywords='maritime autonomous-systems computer-vision dataset object-detection object-tracking odometry',
    zip_safe=False,
)
