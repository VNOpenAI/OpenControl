import setuptools

ver = {}
with open('OpenControl/_version.py') as fd:
        exec(fd.read(), ver)
version = ver.get('__version__')

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="OpenControl",
    version=version,
    author="VNOpenAI",
    author_email="phi9b2@gmail.com",
    description="A python control systems package",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/VNOpenAI/OpenControl",
    project_urls={
        "Bug Tracker": "https://github.com/VNOpenAI/OpenControl/issues",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    packages=setuptools.find_packages(),
    python_requires=">=3.8",
)