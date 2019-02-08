#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT(
	[module_ethercat], 
	m4_esyscmd_s([cat project.properties | grep VERSION    | cut -d'=' -f2 | xargs printf "%s"]),
	m4_esyscmd_s([cat project.properties | grep MAINTAINER | cut -d'=' -f2 | xargs printf "%s"]))
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects silent-rules])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

# Checks for programs.
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])
AX_CHECK_COMPILE_FLAG([-std=gnu11], [CFLAGS+=" -std=gnu11"], [AC_PROG_CC_C99])
AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11([ext], [mandatory])
AC_PROG_AWK
AC_PROG_INSTALL
AC_PROG_LIBTOOL
AC_PROG_LN_S
AC_PROG_MAKE_SET
AC_PROG_INSTALL
m4_ifdef([PKG_PROG_PKG_CONFIG], [PKG_PROG_PKG_CONFIG])
m4_ifdef([PKG_INSTALLDIR], [PKG_INSTALLDIR], [pkgconfigdir='${libdir}/pkgconfig' AC_SUBST([pkgconfigdir])])

# Check dependencies
PKG_CHECK_MODULES([ROBOTKERNEL], [robotkernel])

AC_DEFUN([GET_LIBRARY_PATHES],
         [
            LIBSI=""
            for path in $2; do
                ANS=$([ [[ $path == -L* ]] || echo 'fail' ])
                if test x$ANS != xfail; then
                    STRIPPED=$([ echo ${path#-L} ])
                    if test "$LIBSI" = ""; then
                        LIBSI="$STRIPPED"
                    else
                        LIBSI="$LIBSI:$STRIPPED"
                    fi
                fi
            done

            $1[]=$LIBSI
        ])

GET_LIBRARY_PATHES(LIBETHERCAT_LIBPATHES, $LIBETHERCAT_LIBS)
AC_SUBST(LIBETHERCAT_LIBPATHES)
AC_SUBST(DLRRM_HOST_PLATFORM)

# Checks for header files.
AC_CHECK_HEADERS([stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T

# Checks for library functions.
AC_CHECK_FUNCS([strdup])

# Checks for library functions.
AC_CHECK_LIB(dl, dlopen)

# pthread support 
AX_CHECK_COMPILE_FLAG([-pthread], [AX_APPEND_FLAG([-pthread])] )
AC_CHECK_LIB([pthread], [pthread_create])

AC_CONFIG_FILES([Makefile src/main/cpp/Makefile module_ethercat.pc])
AC_CONFIG_FILES([share/ln/module_ethercat.inc.lnc], [cp share/ln/module_ethercat.inc.lnc ${ac_abs_top_srcdir}/share/ln/])

AC_OUTPUT
