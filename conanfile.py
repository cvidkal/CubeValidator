from conans import ConanFile
from conans.tools import load
import re
import os


class Template(ConanFile):
    python_requires = "cmake-build-helper/[>=0.1]@navi/develop", "qmake-build-helper/[>=0.1]@navi/develop"
    name = "template"
    license = "None"
    author = "YOGO Robot"
    url = "www.yogorobot.com"
    description = "template"
    topics = ("basic")
    settings = "os", "compiler", "build_type", "arch"
    options = {
    }
    default_options = {}
    generators = "qmake", "cmake", "vscode"
    scm = {
        "type": "git",
        "url": "auto",
        "revision": "auto",
        "password": os.environ.get("SECRET", None)
    }

    requires = (
        "apriltag/1.0.0@navi/thirdparty",
        "opencv/3.4.12@navi/thirdparty",
        "eigen/3.3.7@navi/thirdparty",
        "boost/1.77.0@navi/thirdparty",
    )

    def set_version(self):
        content = load(os.path.join(self.recipe_folder, "version.txt"))
        self.version = content.strip()

    def build_with_cmake(self):
        cmake = self.python_requires["cmake-build-helper"].module.CMakeBuildEnvironment(
            self)

        args = []
        cmake.configure(self.source_folder, args)
        cmake.build()

    def build(self):
        self.build_with_cmake()

    def package(self):
        self.copy("*.h*", src="include", dst="include", keep_path=True)
        self.copy("*.so*", dst="lib", keep_path=False)

        # for executable

        # deps = self.deps_cpp_info.deps
        # for dep in deps:
        #     dep_cpp_info = self.deps_cpp_info[dep]
        #     for lib in dep_cpp_info.lib_paths:
        #         self.copy("*.so*", dst="lib", src=lib, keep_path=False)
        #         self.copy("*.a", dst="lib", src=lib, keep_path=False)
