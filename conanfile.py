from conans import ConanFile, tools
import os

class MainProject(ConanFile):
    python_requires = "conan_template/[~=5]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_ethercat"
    description = "robotkernel EtherCAT master module based on libethercat."
    url = "https://rmc-github.robotic.dlr.de/robotkernel/module_ethercat"
    exports_sources = ["*", "!.gitignore"] + ["!%s" % x for x in tools.Git().excluded_files()]
    requires = [
            "robotkernel/[~=5]@robotkernel/stable",
            "service_provider_memory_inspection/py3-support@robotkernel/snapshot",
            "service_provider_canopen_protocol/py3-support@robotkernel/snapshot",
            "service_provider_key_value/py3-support@robotkernel/snapshot",
            "service_provider_sercos_protocol/py3-support@robotkernel/snapshot",
            "service_provider_file_protocol/py3-support@robotkernel/snapshot",
            "service_provider_process_data_inspection/py3-support@robotkernel/snapshot",
            "libethercat/[~=0.3]@common/stable" ]

    def package_info(self):
        base = self.python_requires["conan_template"].module.RobotkernelConanFile
        base.package_info(self)

        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "bindings/python"))

