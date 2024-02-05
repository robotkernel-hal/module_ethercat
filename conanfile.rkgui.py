import os
from conan import ConanFile, conan_version
from conan.tools.scm import Version
from conan.tools.files import copy

class lnrk_interface_python(ConanFile):
    name = "module_ethercat_rkgui"
    description = "python binding to module_ethercat."
    author = "Robert Burger <robert.burgert@dlr.de>"
    license = "GPLv3"
    
    url = f"https://rmc-github.robotic.dlr.de/robotkernel/module_ethercat"
    settings = "os"
    pure_python_folder = os.path.join("bindings","python")
    exports_sources = os.path.join(pure_python_folder, "*")
    
    def requirements(self):
        self.requires("service_provider_memory_inspection_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_canopen_protocol_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_key_value_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_sercos_protocol_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_file_protocol_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_process_data_inspection_rkgui/[>=5.1]@robotkernel/stable")

    def package(self):
        copy(self, os.path.join(self.pure_python_folder, "*"), self.source_folder, self.package_folder)

    def package_info(self):
        if Version(conan_version) < "2.0.0":
            self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, os.path.dirname(self.pure_python_folder)))
        self.runenv_info.append_path("PYTHONPATH", os.path.join(self.package_folder, os.path.dirname(self.pure_python_folder)))
    
