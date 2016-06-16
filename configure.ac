#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.63])
AC_INIT([module_ethercat], [4.0.11], [robert.burger@dlr.de])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

RMPM_ENV
RMPM_ARCH

m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

# Checks for programs.
AC_PROG_CXX
AC_PROG_AWK
AC_PROG_INSTALL
AC_PROG_LIBTOOL
AC_PROG_LN_S

PKG_CHECK_MODULES([ROBOTKERNEL], [robotkernel], [], [
                    RMPM_CHECK_MODULES([ROBOTKERNEL], [robotkernel.robotkernel])
                   ])
PKG_CHECK_MODULES([INTERFACE_MEMORY_INSPECTION], [interface_memory_inspection], [], [
                    RMPM_CHECK_MODULES([INTERFACE_MEMORY_INSPECTION], [robotkernel.interface_memory_inspection])
                   ])
PKG_CHECK_MODULES([INTERFACE_CANOPEN_PROTOCOL], [interface_canopen_protocol], [], [
                    RMPM_CHECK_MODULES([INTERFACE_CANOPEN_PROTOCOL], [robotkernel.interface_canopen_protocol])
                   ])
PKG_CHECK_MODULES([INTERFACE_SERCOS_PROTOCOL], [interface_sercos_protocol], [], [
                    RMPM_CHECK_MODULES([INTERFACE_SERCOS_PROTOCOL], [robotkernel.interface_sercos_protocol])
                   ])
PKG_CHECK_MODULES([INTERFACE_PROCESS_DATA_INSPECTION], [interface_process_data_inspection], [], [
                    RMPM_CHECK_MODULES([INTERFACE_PROCESS_DATA_INSPECTION], [robotkernel.interface_process_data_inspection])
                   ])
PKG_CHECK_MODULES([INTERFACE_FILE_PROTOCOL], [interface_file_protocol], [], [
                    RMPM_CHECK_MODULES([INTERFACE_FILE_PROTOCOL], [robotkernel.interface_file_protocol])
                   ])
PKG_CHECK_MODULES([LIBSTRING_UTIL], [libstring_util], [], [
                    RMPM_CHECK_MODULES([LIBSTRING_UTIL], [software.libstring_util])
                   ])

RM_FIND_LN_FROM_ENV

AC_DEFINE_UNQUOTED(BUILD_DATE,"`date -u`",[Date (usually date of extraction from SVN).])
AC_DEFINE_UNQUOTED(BUILD_USER,"`whoami`",[User (usually user logged in on shell).])
AC_DEFINE_UNQUOTED(BUILD_HOST,"`hostname`",[Hostname (build host).])

# Checks for header files.
AC_CHECK_HEADERS([stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T

# Checks for library functions.
AC_CHECK_FUNCS([strdup])

# Checks for library functions.
AC_CHECK_LIB(dl, dlopen, DL_LIBS="-ldl", DL_LIBS=)
AC_SUBST(DL_LIBS)

AC_CONFIG_FILES([Makefile])
AC_CONFIG_FILES([module_ethercat.pt], [cp module_ethercat.pt ${ac_abs_top_srcdir}])

AC_OUTPUT

