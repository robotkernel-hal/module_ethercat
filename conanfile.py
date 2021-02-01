from conans import tools, python_requires
import os

base = python_requires("conan_template/[~=6]@robotkernel/stable")

class MainProject(base.RobotkernelConanFile):
    name = "module_ethercat"
    description = "robotkernel EtherCAT master module based on libethercat."
    exports_sources = ["*", "!.gitignore"] + ["!%s" % x for x in tools.Git().excluded_files()]
    requires = (
            "robotkernel/[~=6]@robotkernel/unstable",
            "service_provider_memory_inspection/[~=6]@robotkernel/unstable",
            "service_provider_canopen_protocol/[~=6]@robotkernel/unstable",
            "service_provider_key_value/[~=6]@robotkernel/unstable",
            "service_provider_sercos_protocol/[~=6]@robotkernel/unstable",
            "service_provider_file_protocol/[~=6]@robotkernel/unstable",
            "service_provider_process_data_inspection/[~=6]@robotkernel/unstable",
            "libethercat/[~=0.2]@common/stable" )

    def package_info(self):
        super(base.RobotkernelConanFile, self).package_info()
        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "bindings/python/rk_gui_plugin"))

