from conans import ConanFile, tools
import os

class MainProject(ConanFile):
    python_requires = "conan_template_ln_generator/[~=5 >=5.0.7]@robotkernel/stable"
    python_requires_extend = "conan_template_ln_generator.RobotkernelLNGeneratorConanFile"

    name = "module_ethercat"
    description = "robotkernel EtherCAT master module based on libethercat."
    exports_sources = ["*", "!.gitignore"] + ["!%s" % x for x in tools.Git().excluded_files()]
    requires = [
            "robotkernel/[~=5]@robotkernel/stable",
            "service_provider_memory_inspection/[~=5]@robotkernel/stable",
            "service_provider_canopen_protocol/[~=5]@robotkernel/stable",
            "service_provider_key_value/[~=5]@robotkernel/stable",
            "service_provider_sercos_protocol/[~=5]@robotkernel/stable",
            "service_provider_file_protocol/[~=5]@robotkernel/stable",
            "service_provider_process_data_inspection/[~=5]@robotkernel/stable",
            "libethercat/misra-2012-libosal-no-alloc@common/snapshot" ]

    def configure(self):
        self.options["libethercat"].max_slaves          = 256
        self.options["libethercat"].max_groups          = 8
        self.options["libethercat"].max_pdlen           = 3036
        self.options["libethercat"].max_mbx_entries     = 16
        self.options["libethercat"].max_init_cmd_data   = 2048
        self.options["libethercat"].max_slave_fmmu      = 8
        self.options["libethercat"].max_slave_sm        = 8

        base = self.python_requires["conan_template_ln_generator"].module.RobotkernelLNGeneratorConanFile
        base.configure(self)

    def package_info(self):
        base = self.python_requires["conan_template_ln_generator"].module.RobotkernelLNGeneratorConanFile
        base.package_info(self)

        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "bindings/python"))

