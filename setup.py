from setuptools import setup, find_packages

setup(
    name="SyntheticToolkit",
    version="0.1",
    description="Tools for isaac sim",
    packages=find_packages(),
    install_requires=[
        # Add your dependencies here
    ],
    include_package_data=True,
    zip_safe=False,
)
