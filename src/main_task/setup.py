from setuptools import setup, find_packages

setup(
    name='main_task',
    version='0.0.0',
    packages=find_packages(where='scripts'),
    package_dir={'': 'scripts'},
    install_requires=[
        'python-statemachine'
    ],
)
