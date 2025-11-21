from pathlib import Path
from setuptools import setup, find_packages

# read long_description / requirements if you want
this_dir = Path(__file__).parent
requirements = []
req_file = this_dir / 'requirements.txt'
if req_file.exists():
    requirements = [r.strip() for r in req_file.read_text().splitlines() if r.strip() and not r.startswith('#')]

setup(
    name='vision_module',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    include_package_data=True,
    install_requires=requirements,
    zip_safe=False,
    # Provide a console entrypoint if you want ros2 run to work.
    # Replace vision_module.vision_node:main with the actual callable in your vision_node.py
    entry_points={
        'console_scripts': [
            'vision_node = vision_module.vision_node:main'
        ],
    },
)