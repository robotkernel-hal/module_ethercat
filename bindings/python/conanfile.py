import os
from conan import ConanFile, conan_version
from conan.tools.files import copy
from conan.tools.scm import Version

IS_CONAN1 = Version(conan_version) < "2.0.0"

class lnrk_interface_python(ConanFile):
    name = "module_ethercat_rkgui"
    description = "python binding to module_ethercat."
    author = "Robert Burger <robert.burgert@dlr.de>"
    license = "GPLv3"

    url = "https://rmc-github.robotic.dlr.de/robotkernel/module_ethercat"
    settings = "os"
    pure_python_folder = "."
    exports_sources = os.path.join(pure_python_folder, "*")

    def requirements(self):
        self.requires("service_provider_memory_inspection_rkgui/[~6]@robotkernel/unstable")
        self.requires("service_provider_canopen_protocol_rkgui/[~6]@robotkernel/unstable")
        self.requires("service_provider_key_value_rkgui/[~6]@robotkernel/unstable")
        self.requires("service_provider_sercos_protocol_rkgui/[~6]@robotkernel/unstable")
        self.requires("service_provider_file_protocol_rkgui/[~6]@robotkernel/unstable")
        self.requires("service_provider_process_data_inspection_rkgui/[~6]@robotkernel/unstable")

    def package(self):
        copy(self, os.path.join(self.pure_python_folder, "*"), self.source_folder, self.package_folder)

    def package_info(self):
        if IS_CONAN1:
            self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, self.pure_python_folder))
            self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, os.path.dirname(self.pure_python_folder)))
        self.runenv_info.append_path("PYTHONPATH", os.path.join(self.package_folder, self.pure_python_folder))
        self.runenv_info.append_path("PYTHONPATH", os.path.join(self.package_folder, os.path.dirname(self.pure_python_folder)))
