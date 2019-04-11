from conans import ConanFile, AutoToolsBuildEnvironment
import re

class MainProject(ConanFile):
    name = "module_ethercat"
    license = "GPLv3"
    url = f"https://rmc-github.robotic.dlr.de/robotkernel/{name}"
    description = "robotkernel-5 is a modular, easy configurable hardware abstraction framework"
    settings = "os", "compiler", "build_type", "arch"
    scm = {
        "type": "git",
        "url": "auto",
        "revision": "auto",
        "submodule": "recursive",
    }

    generators = "pkg_config"
    requires = (
            "robotkernel/[~=5.0]@robotkernel/unstable",
            "service_provider_memory_inspection/[~=5.0]@robotkernel/unstable",
            "service_provider_canopen_protocol/[~=5.0]@robotkernel/unstable",
            "service_provider_key_value/[~=5.0]@robotkernel/unstable",
            "service_provider_sercos_protocol/[~=5.0]@robotkernel/unstable",
            "service_provider_file_protocol/[~=5.0]@robotkernel/unstable",
            "service_provider_process_data_inspection/[~=5.0]@robotkernel/unstable",
            "libethercat/[~=0.2]@common/stable" )

    def source(self):
        filedata = None
        filename = "project.properties"
        with open(filename, 'r') as f:
            filedata = f.read()
        with open(filename, 'w') as f:
            f.write(re.sub("VERSION *=.*[^\n]", f"VERSION = {self.version}", filedata))

    def build(self):
        self.run("autoreconf -if")
        autotools = AutoToolsBuildEnvironment(self)
        autotools.libs=[]
        autotools.include_paths=[]
        autotools.library_paths=[]
        if self.settings.build_type == "Debug":
            autotools.flags = ["-O0", "-g"]
        else:
            autotools.flags = ["-O3"]
        autotools.configure(configure_dir=".")
        autotools.make()

    def package(self):
        autotools = AutoToolsBuildEnvironment(self)
        autotools.install()

    def package_info(self):
        self.cpp_info.includedirs = ['include']
        self.cpp_info.bindirs = ['bin']
        self.cpp_info.resdirs = ['share']
