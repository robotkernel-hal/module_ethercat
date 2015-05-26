ACLOCAL_AMFLAGS = -I m4 -I scripts/m4

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_ethercat.la

libmodule_ethercat_la_SOURCES  = module_ethercat.cpp \
								 master.cpp \
								 slave.cpp \
								 libethercat/coe.c \
								 libethercat/slave.c \
								 libethercat/datagram.c \
								 libethercat/datagram_pool.c \
								 libethercat/message_pool.c \
								 libethercat/ec.c \
								 libethercat/hw.c \
								 libethercat/mbx.c \
								 libethercat/eeprom.c \
								 libethercat/dc.c \
								 libethercat/timer.c

libmodule_ethercat_la_CXXFLAGS = -Wall \
								 -I$(top_srcdir)/kernel/source \
								 -I$(top_srcdir)/src \
								 -I$(top_srcdir)/libethercat/include \
								 -I$(top_builddir)\
								 @ROBOTKERNEL_CFLAGS@ \
								 @LN_CFLAGS@ \
								 @INTERFACE_MEMORY_INSPECTION_CFLAGS@ \
								 @INTERFACE_CANOPEN_PROTOCOL_CFLAGS@

libmodule_ethercat_la_CFLAGS   = -std=gnu99 \
								 -Wall \
								 -I$(top_srcdir)/kernel/source \
								 -I$(top_srcdir)/src \
								 -I$(top_srcdir)/libethercat/include \
								 -I$(top_builddir) \
								 @ROBOTKERNEL_CFLAGS@ \
								 @LN_CFLAGS@ \
								 @INTERFACE_MEMORY_INSPECTION_CFLAGS@ \
								 @INTERFACE_CANOPEN_PROTOCOL_CFLAGS@

libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

install-exec-hook:
	mkdir -p $(prefix)/share/ln
	-cp -rf $(srcdir)/share/ln/* $(prefix)/share/ln/

