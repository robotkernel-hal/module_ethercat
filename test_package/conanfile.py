import os
import shutil
from six import StringIO

from conans import ConanFile, CMake, tools
from conans.errors import ConanException
from conans.client.run_environment import RunEnvironment

class module_ethercat_test(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "mod_test.rkc"

    def test(self):
        rkc_path = os.path.join(self.source_folder, "mod_test.rkc")
        shutil.copy(rkc_path, 'mod_test.rkc')

        if not tools.cross_building(self.settings):
            re = RunEnvironment(self)
            with tools.environment_append(re.vars):
                outbuf = StringIO()
                try:
                    self.run("robotkernel --test-run --config .%smod_test.rkc" % os.sep, output=outbuf)
                except ConanException as ce:
                    if "ec_open failed" in outbuf.getvalue():
                        # everything is fine here, ignore
                        print(outbuf.getvalue())
                        pass
                    else:
                        raise
        else:
            self.output.warn("Skipping run cross built package")

