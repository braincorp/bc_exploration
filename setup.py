from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from setuptools import setup, find_packages

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name='bc_exploration',
    version='0.0.1',
    description='Brain Corp Exploration Methods',
    long_description=long_description,
    author='Alexander Khoury',
    author_email='akhoury727@gmail.com',
    url='https://github.com/braincorp/bc_exploration',
    download_url='',
    license='Braincorp',
    install_requires=['numpy>=1.11.0',
		      'matplotlib==2.2.3',
                      'opencv-python',
                      'pyyaml==5.1'],
    package_data={'': ['input']},
    include_package_data=True,
        extras_require={
        'tests': ['pytest==4.3.0',
                  'pytest-pep8==1.0.6',
                  'pytest-xdist==1.26.1',
                  'pylint==1.9.2',
                  'astroid==1.6.5'
                  ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Topic :: Software Development :: Libraries',
        'Topic :: Software Development :: Libraries :: Python Modules'
    ],
    packages=find_packages(),
)
