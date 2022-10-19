from conans import ConanFile, tools
import os

class MainProject(ConanFile):
    python_requires = "conan_template/[~=5]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

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
            "libethercat/[~0.4]@common/unstable" ]

    options = { "ecat_device": "ANY", }
    default_options = { "ecat_device": "sock_raw", }

    def configure(self):
        self.options["libethercat"].max_slaves                  = 256
        self.options["libethercat"].max_groups                  = 8
        self.options["libethercat"].max_pdlen                   = 3036
        self.options["libethercat"].max_mbx_entries             = 16
        self.options["libethercat"].max_init_cmd_data           = 2048
        self.options["libethercat"].max_slave_fmmu              = 8
        self.options["libethercat"].max_slave_sm                = 8
        self.options["libethercat"].max_datagrams               = 100
        self.options["libethercat"].max_eeprom_cat_sm           = 8
        self.options["libethercat"].max_eeprom_cat_fmmu         = 8
        self.options["libethercat"].max_eeprom_cat_pdo          = 128
        self.options["libethercat"].max_eeprom_cat_pdo_entries  = 32
        self.options["libethercat"].max_eeprom_cat_strings      = 128
        self.options["libethercat"].max_eeprom_cat_dc           = 8
        self.options["libethercat"].max_string_len              = 128
        self.options["libethercat"].max_data                    = 4096
        self.options["libethercat"].max_ds402_subdevs           = 4
        self.options["libethercat"].max_coe_emergencies         = 10
        self.options["libethercat"].max_coe_emergency_msg_len   = 32
        self.options["libethercat"].ecat_device                 = self.options.ecat_device

        base = self.python_requires["conan_template"].module.RobotkernelConanFile
        base.configure(self)

    def package_info(self):
        base = self.python_requires["conan_template"].module.RobotkernelConanFile
        base.package_info(self)

        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "bindings/python"))

