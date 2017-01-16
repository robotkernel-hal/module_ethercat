ACLOCAL_AMFLAGS = -I m4 -I scripts/m4

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_ethercat.la
BUILT_SOURCES = ln_messages.h
CLEANFILES = ln_messages.h

libmodule_ethercat_la_SOURCES  = master.cpp slave.cpp

libmodule_ethercat_la_CXXFLAGS = -pthread -Wall -I$(srcdir) -I$(builddir) -I$(srcdir)/libethercat/include \
                                 @ROBOTKERNEL_CFLAGS@ @YAML_CPP_CFLAGS@ @LIBSTRING_UTIL_CFLAGS@ \
                                 @LIBETHERCAT_PIC_CFLAGS@ @LINKS_AND_NODES_CFLAGS@ \
                                 @INTERFACE_MEMORY_INSPECTION_CFLAGS@ \
                                 @INTERFACE_CANOPEN_PROTOCOL_CFLAGS@ \
                                 @INTERFACE_SERCOS_PROTOCOL_CFLAGS@ \
                                 @INTERFACE_FILE_PROTOCOL_CFLAGS@

libmodule_ethercat_la_CFLAGS   = -pthread -std=gnu99 -Wall -I$(srcdir) -I$(builddir) -I$(srcdir)/libethercat/include \
                                 @ROBOTKERNEL_CFLAGS@ @YAML_CPP_CFLAGS@ @LIBSTRING_UTIL_CFLAGS@ \
                                 @LIBETHERCAT_PIC_CFLAGS@ @LINKS_AND_NODES_CFLAGS@ \
                                 @INTERFACE_MEMORY_INSPECTION_CFLAGS@ \
                                 @INTERFACE_CANOPEN_PROTOCOL_CFLAGS@ \
                                 @INTERFACE_SERCOS_PROTOCOL_CFLAGS@ \
                                 @INTERFACE_FILE_PROTOCOL_CFLAGS@

libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION) @LIBETHERCAT_PIC_LIBS@ -lpthread

VPATH = $(srcdir):$(top_srcdir)/share/ln/message_definitions 

LN_MESSAGES =  module_ethercat/set_ec_state \
               module_ethercat/get_ec_state 

ln_messages.h: Makefile $(LN_MESSAGES)
	@echo creating ln_messages.h
	@LN_GENERATE@ -o $@ $(LN_MESSAGES)
	touch $@

install-exec-hook:
	mkdir -p $(prefix)/share
	cp -ru $(top_srcdir)/share/* $(prefix)/share

