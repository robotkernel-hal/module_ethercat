ACLOCAL_AMFLAGS = -I m4 -I scripts/m4

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_ethercat.la
BUILT_SOURCES = ln_messages.h
CLEANFILES = ln_messages.h

libmodule_ethercat_la_SOURCES  = master.cpp slave.cpp

libmodule_ethercat_la_CXXFLAGS = --std=c++11 -pthread -Wall -I$(srcdir) -I$(builddir) -I$(srcdir)/libethercat/include \
                                 @ROBOTKERNEL_CFLAGS@ @YAML_CPP_RMC_CFLAGS@ @LIBSTRING_UTIL_CFLAGS@ \
                                 @LIBETHERCAT_PIC_CFLAGS@ @PTHREAD_CFLAGS@\
                                 @SERVICE_PROVIDER_MEMORY_INSPECTION_CFLAGS@ \
                                 @SERVICE_PROVIDER_CANOPEN_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_SERCOS_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_FILE_PROTOCOL_CFLAGS@

libmodule_ethercat_la_CFLAGS   = --std=c++11 -pthread -Wall -I$(srcdir) -I$(builddir) -I$(srcdir)/libethercat/include \
                                 @ROBOTKERNEL_CFLAGS@ @YAML_CPP_RMC_CFLAGS@ @LIBSTRING_UTIL_CFLAGS@ \
                                 @LIBETHERCAT_PIC_CFLAGS@ @PTHREAD_CFLAGS@\
                                 @SERVICE_PROVIDER_MEMORY_INSPECTION_CFLAGS@ \
                                 @SERVICE_PROVIDER_CANOPEN_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_SERCOS_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_FILE_PROTOCOL_CFLAGS@

libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION) @LIBETHERCAT_PIC_LIBS@ @PTHREAD_LIBS@ @LIBSTRING_UTIL_LIBS@

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

