ACLOCAL_AMFLAGS = -I m4

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_ethercat.la

libmodule_ethercat_la_SOURCES  = master.cpp slave.cpp

libmodule_ethercat_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) -I$(srcdir)/libethercat/include \
                                 @ROBOTKERNEL_CFLAGS@ @LIBETHERCAT_CFLAGS@ \
                                 @SERVICE_PROVIDER_MEMORY_INSPECTION_CFLAGS@ \
                                 @SERVICE_PROVIDER_CANOPEN_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_SERCOS_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_FILE_PROTOCOL_CFLAGS@

libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION) @LIBETHERCAT_PIC_LIBS@ @PTHREAD_LIBS@ @LIBSTRING_UTIL_LIBS@


