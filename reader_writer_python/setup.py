from setuptools import setup

package_name = "reader_writer_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "reader = reader_writer_python.reader:main",
            "writer = reader_writer_python.writer:main",
            "remapping = reader_writer_python.remapping:main",
        ]
    },
)
