# python setup.py bdist_wheel
# sudo pip install ./dist/*.whl

import setuptools

with open("README.md", "r") as fh:
    README = fh.read()

requirements = [
    'trimesh',
    "transforms3d",
    "numpy",
    "pandas",
    "shapely",
    "scikit-learn"
]

setup_requirements = [
    # TODO: put setup requirements (distutils extensions, etc.) here
]

test_requirements = [
    'pytest'
]

setuptools.setup(
    name="part2cad",
    version="0.0.1",

    author="Zeyu Zhang",
    author_email="zeyuz@outlook.com",
    
    description="Part-level object CAD replacement",
    long_description=README,
    long_description_content_type="text/markdown",
    
    url="https://github.com/TooSchoolForCool/CIESSL",
    
    packages=setuptools.find_packages(include=['CIESSL']),
    include_package_data=True,

    license="Apache-2.0",

    test_suite='tests',

    install_requires=requirements,
    
    tests_require=test_requirements,

    setup_requires=setup_requirements,

    classifiers=(
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: Apache-2.0 License",
        "Operating System :: OS Independent",
    ),
)
