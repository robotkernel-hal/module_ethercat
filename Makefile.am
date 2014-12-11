ACLOCAL_AMFLAGS = -I m4 

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_ethercat.la

libmodule_ethercat_la_SOURCES  = module_ethercat.cpp \
								 slave.cpp \
								 libethercat/coe.c \
								 libethercat/datagram.c \
								 libethercat/datagram_pool.c \
								 libethercat/ec.c \
								 libethercat/hw.c \
								 libethercat/mbx.c \
								 libethercat/eeprom.c

libmodule_ethercat_la_CXXFLAGS = -Wall -I$(top_srcdir)/kernel/source -I$(top_srcdir)/src -I$(top_srcdir)/libethercat -I$(top_builddir) @ROBOTKERNEL_CFLAGS@
libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

install-exec-hook:
	mkdir -p $(prefix)/share/ln
	-cp -rf $(srcdir)/share/ln/* $(prefix)/share/ln/

