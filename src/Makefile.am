ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libmodule_ethercat.la

BUILT_SOURCES = $(top_builddir)/src/gen_definitions.h
CLEANFILES = $(top_builddir)/src/gen_definitions.h

libmodule_ethercat_la_SOURCES  = master.cpp group.cpp slave.cpp slave_file_protocol.cpp slave_memory_inspection.cpp \
								 slave_canopen_protocol.cpp hw_stream.c

libmodule_ethercat_la_CXXFLAGS = -Wall -Wno-address-of-packed-member -Wno-deprecated-declarations -I$(srcdir) -I$(builddir) \
                                 @ROBOTKERNEL_CFLAGS@ @LIBETHERCAT_CFLAGS@  \
                                 @SERVICE_PROVIDER_MEMORY_INSPECTION_CFLAGS@ \
                                 @SERVICE_PROVIDER_CANOPEN_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_KEY_VALUE_CFLAGS@ \
                                 @SERVICE_PROVIDER_SERCOS_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_FILE_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@

libmodule_ethercat_la_CFLAGS = -Wall -Wno-address-of-packed-member -Wno-deprecated-declarations -I$(srcdir) -I$(builddir) \
                                 @LIBETHERCAT_CFLAGS@ 

libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION) @LIBETHERCAT_LIBS@

VPATH = $(srcdir):$(top_srcdir)/share

GEN_DEFINITIONS = pd_definitions/module_ethercat/master

$(top_builddir)/src/gen_definitions.h: Makefile $(GEN_DEFINITIONS)
	@echo creating $@
	rk_generate --indir $(top_srcdir)/share --out $@ $(GEN_DEFINITIONS)
	touch $@
