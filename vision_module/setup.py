from pathlib import Path
from setuptools import setup, find_packages

this_dir = Path(__file__).parent
requirements = []
req_file = this_dir / 'requirements.txt'
if req_file.exists():
    requirements = [r.strip() for r in req_file.read_text().splitlines() if r.strip() and not r.startswith('#')]

# list package data to install into share/<package>
data_files = [
    ('share/vision_module', ['package.xml']),
]

# add launch/config/resource files if present
launch_file = this_dir / 'launch' / 'vision_launch.py'
if launch_file.exists():
    data_files.append(('share/vision_module/launch', [str(launch_file)]))
config_file = this_dir / 'config' / 'yolo.yaml'
if config_file.exists():
    data_files.append(('share/vision_module/config', [str(config_file)]))
# include entire resource dir if exists
resource_dir = this_dir / 'resource'
if resource_dir.exists():
    # add each file under resource into share/vision_module/resource
    resource_files = [str(p) for p in resource_dir.rglob('*') if p.is_file()]
    if resource_files:
        data_files.append(('share/vision_module/resource', resource_files))

setup(
    name='vision_module',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    include_package_data=True,
    install_requires=requirements,
    data_files=data_files,
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'vision_node = vision_module.vision_node:main'
        ],
    },
)