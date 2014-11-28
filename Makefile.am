ACLOCAL_AMFLAGS = -I m4 

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_ethercat.la

libmodule_ethercat_la_SOURCES  = module_ethercat.cpp module_ethercat_firmware.cpp
libmodule_ethercat_la_CXXFLAGS = -Wall -I$(top_srcdir)/kernel/source -I$(top_srcdir)/src -I$(top_builddir) @ROBOTKERNEL_CFLAGS@
libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

install-exec-hook:
	mkdir -p $(prefix)/share/ln
	-cp -rf $(srcdir)/share/ln/* $(prefix)/share/ln/

