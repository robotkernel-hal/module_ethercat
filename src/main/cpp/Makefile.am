ACLOCAL_AMFLAGS = -I m4

libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_ethercat.la

libmodule_ethercat_la_SOURCES  = master.cpp slave.cpp slave_file_protocol.cpp slave_memory_inspection.cpp \
								 slave_canopen_protocol.cpp

libmodule_ethercat_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) -I$(srcdir)/libethercat/include \
                                 @ROBOTKERNEL_CFLAGS@ @LIBETHERCAT_CFLAGS@ \
                                 @SERVICE_PROVIDER_MEMORY_INSPECTION_CFLAGS@ \
                                 @SERVICE_PROVIDER_CANOPEN_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_SERCOS_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_FILE_PROTOCOL_CFLAGS@ \
                                 @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@

libmodule_ethercat_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION) @LIBETHERCAT_LIBS@ @LIBSTRING_UTIL_LIBS@


