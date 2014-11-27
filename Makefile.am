ACLOCAL_AMFLAGS = -I m4 

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_jr3.la

libmodule_jr3_la_SOURCES  = module_jr3.cpp module_jr3_firmware.cpp
libmodule_jr3_la_CXXFLAGS = -Wall -I$(top_srcdir)/kernel/source -I$(top_srcdir)/src -I$(top_builddir) @ROBOTKERNEL_CFLAGS@
libmodule_jr3_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

install-exec-hook:
	mkdir -p $(prefix)/share/ln
	-cp -rf $(srcdir)/share/ln/* $(prefix)/share/ln/

