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
            "libethercat/0.4.0-dc-test@common/snapshot" ]

    options = {
            "max_slaves"                 : "ANY",
            "max_groups"                 : "ANY",
            "max_pdlen"                  : "ANY",
            "max_mbx_entries"            : "ANY",
            "max_init_cmd_data"          : "ANY",
            "max_slave_fmmu"             : "ANY",
            "max_slave_sm"               : "ANY",
            "max_datagrams"              : "ANY",
            "max_eeprom_cat_sm"          : "ANY",
            "max_eeprom_cat_fmmu"        : "ANY",
            "max_eeprom_cat_pdo"         : "ANY",
            "max_eeprom_cat_pdo_entries" : "ANY",
            "max_eeprom_cat_strings"     : "ANY",
            "max_eeprom_cat_dc"          : "ANY",
            "max_string_len"             : "ANY",
            "max_data"                   : "ANY",
            "max_ds402_subdevs"          : "ANY",
            "max_coe_emergencies"        : "ANY",
            "max_coe_emergency_msg_len"  : "ANY",
            "ecat_device"                : "ANY",
            }
    default_options = {
            "max_slaves"                 : 256,
            "max_groups"                 : 8,
            "max_pdlen"                  : 3036,
            "max_mbx_entries"            : 16,
            "max_init_cmd_data"          : 2048,
            "max_slave_fmmu"             : 8,
            "max_slave_sm"               : 8,
            "max_datagrams"              : 100,
            "max_eeprom_cat_sm"          : 8,
            "max_eeprom_cat_fmmu"        : 8,
            "max_eeprom_cat_pdo"         : 128,
            "max_eeprom_cat_pdo_entries" : 32,
            "max_eeprom_cat_strings"     : 128,
            "max_eeprom_cat_dc"          : 8,
            "max_string_len"             : 128,
            "max_data"                   : 4096,
            "max_ds402_subdevs"          : 4,
            "max_coe_emergencies"        : 10,
            "max_coe_emergency_msg_len"  : 32,
            "ecat_device"                : "sock_raw",
            }

    def configure(self):
        self.options["libethercat"].max_slaves                  = self.options.max_slaves
        self.options["libethercat"].max_groups                  = self.options.max_groups
        self.options["libethercat"].max_pdlen                   = self.options.max_pdlen
        self.options["libethercat"].max_mbx_entries             = self.options.max_mbx_entries
        self.options["libethercat"].max_init_cmd_data           = self.options.max_init_cmd_data
        self.options["libethercat"].max_slave_fmmu              = self.options.max_slave_fmmu
        self.options["libethercat"].max_slave_sm                = self.options.max_slave_sm
        self.options["libethercat"].max_datagrams               = self.options.max_datagrams
        self.options["libethercat"].max_eeprom_cat_sm           = self.options.max_eeprom_cat_sm
        self.options["libethercat"].max_eeprom_cat_fmmu         = self.options.max_eeprom_cat_fmmu
        self.options["libethercat"].max_eeprom_cat_pdo          = self.options.max_eeprom_cat_pdo
        self.options["libethercat"].max_eeprom_cat_pdo_entries  = self.options.max_eeprom_cat_pdo_entries
        self.options["libethercat"].max_eeprom_cat_strings      = self.options.max_eeprom_cat_strings
        self.options["libethercat"].max_eeprom_cat_dc           = self.options.max_eeprom_cat_dc
        self.options["libethercat"].max_string_len              = self.options.max_string_len
        self.options["libethercat"].max_data                    = self.options.max_data
        self.options["libethercat"].max_ds402_subdevs           = self.options.max_ds402_subdevs
        self.options["libethercat"].max_coe_emergencies         = self.options.max_coe_emergencies
        self.options["libethercat"].max_coe_emergency_msg_len   = self.options.max_coe_emergency_msg_len
        self.options["libethercat"].ecat_device                 = self.options.ecat_device

        base = self.python_requires["conan_template"].module.RobotkernelConanFile
        base.configure(self)

    def package_info(self):
        base = self.python_requires["conan_template"].module.RobotkernelConanFile
        base.package_info(self)

        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "bindings/python"))

