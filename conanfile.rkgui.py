import os

from conan import ConanFile
from conan.tools.files import copy


class lnrk_interface_python(ConanFile):
    python_requires = "cissy_conantools/[~1]@tools/stable"

    @property
    def cissy_conantools(self):
        return self.python_requires["cissy_conantools"].module

    name = "module_ethercat_rkgui"
    description = "python binding to module_ethercat."
    author = "Robert Burger <robert.burgert@dlr.de>"
    license = "GPLv3"

    url = "https://rmc-github.robotic.dlr.de/robotkernel/module_ethercat"
    settings = "os"
    pure_python_folder = os.path.join("bindings", "python")
    exports_sources = os.path.join(pure_python_folder, "*")

    def requirements(self):
        self.requires("service_provider_memory_inspection_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_canopen_protocol_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_key_value_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_sercos_protocol_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_file_protocol_rkgui/[>=5.1]@robotkernel/stable")
        self.requires("service_provider_process_data_inspection_rkgui/[>=5.1]@robotkernel/stable")

    def package(self):
        copy(
            self,
            os.path.join(self.pure_python_folder, "*"),
            self.source_folder,
            self.package_folder,
        )

    def package_info(self):
        self.cissy_conantools.autoset_package_info(self,
            pythondirs=[
                os.path.join(self.package_folder, self.pure_python_folder),
                os.path.join(self.package_folder, os.path.dirname(self.pure_python_folder))
            ],
        )

