import os
import shutil
import subprocess
import re
from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.env import VirtualRunEnv, VirtualBuildEnv
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.scm import Version

required_conan_version = ">=1.53.0"


class DevastatorConan(ConanFile):
    name = "devastator"
    version = "dev"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "ros": ["system", "foxy", "humble"],
        "cuda": ["system", "ANY", None],
        "with_test": [True, False],
        "platform": ["orin", "a1000", "x86"],
    }
    default_options = {
        "ros": "system",
        "cuda": "system",
        "with_test": False,
        "platform": "x86",
    }

    def requirements(self):
        # self.requires("abseil/20220623.0")
        # self.requires("adolc/2.7.2")
        # self.requires("adrss/1.1.0")
        # self.requires("alc/1.0.1")
        # self.requires("ald/0.1.7")
        # self.requires("ceres-solver/2.1.0")
        # self.requires("glog/0.6.0@transformer/stable", force=True)
        # self.requires("grpc/1.50.1")
        # self.requires("gtsam/4.1.1")
        # self.requires("zeromq/4.3.4")
        # self.requires("hiredis/1.0.2")
        # self.requires("hslidar_sdk/4.3.0")
        # self.requires("inno_lidar/2.5.0")
        # self.requires("jsoncpp/1.8.3")
        # self.requires("mimalloc/2.0.9")
        # self.requires("nanoflann/1.4.3")
        # self.requires("nlohmann_json/[>3.10.5]")
        # self.requires("opencv/4.5.5@transformer/stable")
        # self.requires("osqp/0.6.2")
        # self.requires("pcl/1.11.1")
        # self.requires("proj/7.2.1")
        # self.requires("libkml/1.3.0")
        # if self.options.ros == "foxy":
        #     self.requires("ros2/foxy", transitive_headers=True, transitive_libs=True)
        # self.requires("rslidar_sdk/1.3.2")
        # if self.options.cuda and self.cuda_version is not None:
        #     if Version(self.cuda_version) >= "11.4":
        #         self.requires("tensorrt/8.4.1.5")
        #     else:
        #         self.requires("tensorrt/8.2.5.1")
        # self.requires("tinyxml2/9.0.0")
        self.requires("toml11/3.7.0")
        self.requires("yaml-cpp/0.7.0")
        # # try to resolve conflicts
        # self.requires("boost/1.82.0", force=True)
        # self.requires("eigen/3.4.0", force=True)
        # self.requires("libjpeg-turbo/3.0.0", force=True)
        # self.requires("openssl/[>=1.1.1s]", force=True)
        # self.requires("protobuf/3.21.12", force=True)
        # self.requires("pybind11/[>=2.10.0]", force=True)
        # self.requires("sqlite3/[>=3.39.4]", force=True)
        # self.requires("zlib/[>=1.2.12]", force=True)
        # self.requires("fast-cdr/1.0.26", force=True)
        # if self.options.with_test:
        #     self.requires("gtest/1.11.0")
        # if self.options.platform == "a1000":
        #     self.requires("bsnn/3.10.1")
        #     self.requires("rcall/0.0.2")
        # if self.options.platform == "orin":
        #     self.requires("jetson_multimedia_api/35.1.0")

    # def build_requirements(self):
        # self.build_requires("grpc/1.50.1")
        # self.build_requires("protobuf/3.21.12")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["CMAKE_POLICY_DEFAULT_CMP0077"] = "NEW"
        # if self.options.cuda and self.cuda_version is not None:
        #     tc.variables["USE_CUDA"] = True
        if self.options.with_test:
            tc.variables["BUILD_TESTING"] = True
        else:
            tc.variables["BUILD_TESTING"] = False
        tc.generate()
        tc = CMakeDeps(self)
        tc.generate()
        tc = VirtualRunEnv(self)
        tc.generate()
        tc = VirtualBuildEnv(self)
        tc.generate(scope="build")
