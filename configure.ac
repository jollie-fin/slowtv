# based on https://github.com/ngageoint/hootenanny/blob/master/configure.ac

AC_PREREQ([2.65])
AC_INIT([amhello], [1.0], [bug-automake@gnu.org])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_SRCDIR([src])

AC_LANG([C++])
AX_CXX_COMPILE_STDCXX_20([], [mandatory])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_RANLIB
LT_INIT

# GDAL
m4_include([m4/ax_lib_gdal.m4])
AX_LIB_GDAL([1.4.4])
LIBS="$LIBS $GDAL_RPATH $GDAL_LDFLAGS"
CPPFLAGS="$CPPFLAGS $GDAL_CFLAGS"
if test "${GDAL_OGR_ENABLED}" = "yes" ; then
  ogrinfo --formats > /dev/null
  OGR_FORMAT_EXIT_STATUS=`echo $?`
  if test "$OGR_FORMAT_EXIT_STATUS" != "0"; then
    AC_MSG_ERROR(["There is an error with one or more GDAL file formats."]);
  fi
fi
if test "${GDAL_VERSION}" = "1.9.1" ; then
  AC_MSG_WARN(["GDAL v1.9.1 is known to have a bug in FileGDB export where the file can't be opened by ArcMap 10.2. GDAL v1.10.1 is known to work."]);
fi
if test "${GDAL_VERSION}" != "2.1.4" ; then
  AC_MSG_WARN([Most development occurs with GDAL v2.1.4, using v$GDAL_VERSION. Please run all tests after build (make test).]);
fi

# Boost
m4_include([m4/boost.m4])
AX_LIB_BOOST()

# OpenCV - the library names change from version to version so we have to search for the right name.
AC_LANG_PUSH([C++])
AC_SEARCH_LIBS([cvSmooth], [opencv_imgproc opencv_core opencv_highgui cxcore cv], [], [AC_MSG_ERROR([A necessary OpenCV library was not found])])
AC_SEARCH_LIBS([cvCreateImage], [opencv_imgproc opencv_core opencv_highgui cxcore cv], [], [AC_MSG_ERROR([A necessary OpenCV library was not found])])
AC_CHECK_HEADERS([opencv2/opencv.hpp opencv/cv.h], [hootFoundCvHeaders=yes; break;])
AS_IF([test "x$hootFoundCvHeaders" != "xyes"], [AC_MSG_ERROR([Unable to find the opencv headers])])

AC_COMPILE_IFELSE(
    [
        AC_LANG_PROGRAM([[
#ifdef HAVE_OPENCV2_OPENCV_HPP
# include <opencv2/opencv.hpp>
#endif
using namespace cv;
]], 
        [[
Mat m1, m2, m3; 
calcCovarMatrix(m1, m2, m3, COVAR_NORMAL | COVAR_ROWS | COVAR_SCALE, CV_64F);
]])
    ],
    [],
    AC_MSG_FAILURE("OpenCV version is incompatible. Version 4.2.0 is known to work."))

AC_LANG_POP

