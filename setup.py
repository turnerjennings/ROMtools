from setuptools import setup,find_packages


setup(
    name='ROMtools',
    version='0.2.6',
    description='Tools for reduced order modelling of dynamical systems',
    author='Turner Jennings',
    author_email='turner.jennings@outlook.com',
    url='https://github.com/turnerjennings/ROMtools',
    license='MIT',
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy",
        "matplotlib"],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Natural Language :: English",
        "Topic :: Scientific/Engineering"
    ]
)