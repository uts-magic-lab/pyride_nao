dnl Copyright (C) 1999-2005 Open Source Telecom Corporation.
dnl Copyright (C) 2006-2010 David Sugar, Tycho Softworks.
dnl
dnl This program is free software; you can redistribute it and/or modify
dnl it under the terms of the GNU General Public License as published by
dnl the Free Software Foundation; either version 2 of the License, or
dnl (at your option) any later version.
dnl
dnl This program is distributed in the hope that it will be useful,
dnl but WITHOUT ANY WARRANTY; without even the implied warranty of
dnl MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
dnl GNU General Public License for more details.
dnl
dnl You should have received a copy of the GNU General Public License
dnl along with this program; if not, write to the Free Software
dnl Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
dnl
dnl As a special exception to the GNU General Public License, if you
dnl distribute this file as part of a program that contains a configuration
dnl script generated by Autoconf, you may include it under the same
dnl distribution terms that you use for the rest of that program.


AC_DEFUN([OST_CXX_PROGRAMMING],[
    AC_REQUIRE([OST_PROG_CC_POSIX])
    AC_PROG_CPP
    AC_PROG_CXX
    AC_PROG_CXXCPP

    dnl
    dnl Check for common C++ portability problems
    dnl

    AC_LANG_SAVE
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""
    AC_LANG_CPLUSPLUS

    dnl Check whether we have bool
    AC_CACHE_CHECK(whether ${CXX} has built-in bool type,
        ac_cv_cxx_bool_type,
        AC_TRY_COMPILE(,
            [bool b1=true; bool b2=false;],
            ac_cv_cxx_bool_type=yes,
            ac_cv_cxx_bool_type=no
        )
    )

    if test $ac_cv_cxx_bool_type = yes ; then
        AC_DEFINE(HAVE_BOOL_TYPE, [1], [have bool type])
    fi

    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
    AH_BOTTOM([
#ifndef HAVE_BOOL_TYPE
typedef enum { true=1, false=0 } bool;
#endif


    ])
])

AC_DEFUN([OST_CXX_NEW_INIT],[
AC_REQUIRE([OST_PROG_CC_POSIX])
# AC_PROG_CPP
# AC_PROG_CXX
# AC_PROG_CXXCPP

    dnl
    dnl Check for common C++ portability problems
    dnl

    AC_LANG_SAVE
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""
    AC_LANG_CPLUSPLUS

    dnl Check whether we have bool
    AC_CACHE_CHECK([whether ${CXX} has new(size_t,void*)],
        ac_cv_cxx_new_init,
        AC_TRY_COMPILE([#include <new>
#include <iostream>
using namespace std;],
        [int* p1 = new int();
int* p2 = new (p1) int();
return 0;],
            ac_cv_cxx_new_init=yes,
            ac_cv_cxx_new_init=no
        )
    )

    if test $ac_cv_cxx_new_init = yes ; then
        AC_DEFINE(CCXX_HAVE_NEW_INIT, [1], [have new with init])
    fi

    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
    AH_BOTTOM([
#ifdef CCXX_NAMESPACES
#define USING(x) using namespace x;
#else
#define USING(x)
#endif

#ifdef  __KCC
#define KAI_NONSTD_IOSTREAM 1
#endif
    ])
])

AC_DEFUN([OST_CXX_ARRAYS],[
    AC_REQUIRE([OST_CXX_PROGRAMMING])
    dnl
    dnl Determine C++ support for dynamic sized arrays in stack frame.
    dnl

    AC_LANG_SAVE
    AC_LANG_CPLUSPLUS
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""

    AC_CACHE_CHECK(wheather dynamic arrays in stack frame,
        ost_cv_cxx_array,
        AC_TRY_COMPILE(,[
            int x;
            int y\[x\];
            return 0;
        ], ost_cv_cxx_array=no, ost_cv_array=yes)
    )
    if test "$ost_cv_cxx_array" = yes ; then
        AC_DEFINE(HAVE_DYN_ARRAY, [1], [c++ dynamic arrays in stack frame])
    fi
    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
])

AC_DEFUN([OST_CXX_IOSTREAM],[
    AC_REQUIRE([OST_CXX_PROGRAMMING])
    dnl
    dnl Determine kind of C++ iostream support.
    dnl

    AC_LANG_SAVE
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""
    AC_LANG_CPLUSPLUS
    AC_CACHE_CHECK(wheather old style iostreams,
        ost_cv_cxx_iostream,
        AC_TRY_COMPILE([
#include <iostream>
using namespace std;

class mystr : public streambuf, public iostream
{
    mystr();
};

mystr::mystr() : streambuf(), iostream((streambuf *)this)
{
}
        ],[return 0;],
            ost_cv_cxx_iostream=no,
            ost_cv_cxx_iostream=yes
        )
    )
    if test $ost_cv_cxx_iostream = yes ; then
        AC_DEFINE(HAVE_OLD_IOSTREAM, [1], [old style iostreams])
    fi
    AC_CHECK_HEADERS(sstream)

    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
])

AC_DEFUN([OST_CXX_NAMESPACE],[
    AC_REQUIRE([OST_CXX_PROGRAMMING])

    dnl
    dnl Determine if C++ supports namespaces.
    dnl

    AC_LANG_SAVE
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""
    AC_LANG_CPLUSPLUS
    AC_CACHE_CHECK(whether ${CXX} supports namespace,
    ost_cv_cxx_namespace,
        AC_TRY_COMPILE([
#include <iostream>
namespace Test { using namespace std; }],[return 0;],
            ost_cv_cxx_namespace=yes,
            ost_cv_cxx_namespace=no
        )
    )

    if test "$ost_cv_cxx_namespace" = yes ; then
        AC_DEFINE(CCXX_NAMESPACES, [1], [has c++ namespaces])
    fi

    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
])

AC_DEFUN([OST_CXX_MUTABLE],[
    AC_REQUIRE([OST_CXX_PROGRAMMING])

    dnl
    dnl Determine if C++ supports mutable members.
    dnl

    AC_LANG_SAVE
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""
    AC_LANG_CPLUSPLUS

    AC_CACHE_CHECK(whether ${CXX} supports mutable,
        ost_cv_cxx_mutable,
        AC_TRY_COMPILE([
class t {mutable int i;};], [return 0;],
            ost_cv_cxx_mutable=yes,
            ost_cv_cxx_mutable=no
        )
    )

    if test $ost_cv_cxx_mutable = no ; then
        COMMON_FLAGS="$COMMON_FLAGS -Dmutable"
    fi

    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
])

AC_DEFUN([OST_CXX_NOEXCEPTIONS],[
    AC_REQUIRE([OST_CXX_PROGRAMMING])

    dnl
    dnl Disable C++ exception handling whenever possible.
    dnl

    AC_LANG_SAVE
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""
    AC_LANG_CPLUSPLUS

    dnl strip -fexceptions flag if used
    optflags=$CXXFLAGS
    if test ! -z "$optflags" ; then
        CXXFLAGS=""
        for opt in $optflags ; do
            case $opt in
            *rtti*)
                ;;
            *exceptions*)
                ;;
            *)
                CXXFLAGS="$CXXFLAGS $opt"
            ;;
            esac
        done
    fi
    AC_CACHE_CHECK(whether ${CXX} supports -fno-exceptions,
        ac_cv_cxx_noexception_flag,[
        echo 'void f(){}' >conftest.cpp
        if test -z "`${CXX} -fno-exceptions -c conftest.cpp 2>&1`"; then
            COMMON_FLAGS="$COMMON_FLAGS -fno-exceptions"
            ac_cv_cxx_noexception_flag=yes
        else
            ac_cv_cxx_noexception_flag=no
        fi
        rm -f conftest*
    ])

    AC_CACHE_CHECK(whether ${CXX} supports -fno-rtti,
        ac_cv_cxx_no_rtti_flag,[
        echo '#include <sstream>' >conftest.cpp
        echo 'void f(){}' >>conftest.cpp
        if test -z "`${CXX} -fno-rtti -c conftest.cpp 2>&1`"; then
            COMMON_FLAGS="$COMMON_FLAGS -fno-rtti"
            ac_cv_cxx_no_rtti_flag=yes
        else
            ac_cv_cxx_no_rtti_flag=no
        fi
        rm -f conftest*
    ])

    AC_CACHE_CHECK(whether ${CXX} supports -fno-check-new,
        ac_cv_cxx_no_check_new_flag,[
        echo 'void f(){}' >conftest.cpp
        if test -z "`${CXX} -fno-check-new -c conftest.cpp 2>&1`"; then
            COMMON_FLAGS="$COMMON_FLAGS -fno-check-new"
            ac_cv_cxx_no_check_new_flag=yes
        else
            ac_cv_cxx_no_check_new_flag=no
        fi
        rm -f conftest*
    ])

    AC_CACHE_CHECK(whether ${CXX} supports -finline,
        ac_cv_cxx_inline_flag,[
        echo 'void f(){}' >conftest.cpp
        if test -z "`${CXX} -finline -c conftest.cpp 2>&1`"; then
            COMMON_FLAGS="$COMMON_FLAGS -finline"
            ac_cv_cxx_inline_flag=yes
        else
            ac_cv_cxx_inline_flag=no
        fi
        rm -f conftest*
    ])


    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
])

AC_DEFUN([OST_CXX_EXCEPTIONS],[
    AC_REQUIRE([OST_CXX_PROGRAMMING])

    dnl
    dnl Enable C++ exception handling whenever possible.
    dnl

    AC_LANG_SAVE
    ac_save_CXXFLAGS="$CXXFLAGS"
    CXXFLAGS=""
    AC_LANG_CPLUSPLUS

    dnl strip -fno-exceptions flag if used
    optflags=$CXXFLAGS
    if test ! -z "$optflags" ; then
        CXXFLAGS=""
        for opt in $optflags ; do
            case $opt in
            *no-rtti*)
                ;;
            *omit-frame-pointer*)
                ;;
            *no-exceptions*)
                ;;
            *)
                CXXFLAGS="$CXXFLAGS $opt"
            ;;
            esac
        done
    fi

    dnl Check for exception handling
    AC_CACHE_CHECK(whether ${CXX} supports -fhandle-exceptions,
        ac_cv_cxx_exception_flag,[
        echo 'void f(){}' >conftest.cpp
        if test -z "`${CXX} -fhandle-exceptions -c conftest.cpp 2>&1`"; then
            ac_cv_cxx_exception_flag=yes
            COMMON_FLAGS="$COMMON_FLAGS -fhandle-exceptions"
        else
            ac_cv_cxx_exception_flag=no
        fi
        rm -f conftest*
    ])

    if test $ac_cv_cxx_exception_flag = "yes" ; then
        ac_cv_cxx_exception_handling=yes
    else
        AC_CACHE_CHECK(whether ${CXX} supports exception handling,
        ac_cv_cxx_exception_handling,
        AC_TRY_COMPILE(
            [void f(void)
            {
                throw "abc";
            }
            void g(void)
            {
                try {
                    f();
                }
                catch(char*){}
            }
            ],,
            ac_cv_cxx_exception_handling=yes,
            ac_cv_cxx_exception_handling=no
        )
    )
    fi

    if test $ac_cv_cxx_exception_handling = yes ; then
        AC_DEFINE(CCXX_EXCEPTIONS, [1], [has c++ exception handling])
        AC_CHECK_HEADERS(exception)
    fi

    AC_LANG_RESTORE
    CXXFLAGS="$ac_save_CXXFLAGS"
    AH_BOTTOM([
#ifndef CCXX_EXCEPTIONS
/* disable HAVE_EXCEPTION */
#ifdef  HAVE_EXCEPTION
#undef  HAVE_EXCEPTION
#endif
/* throw - replacement to throw an exception */
#define THROW(x) abort()
/* throw - replacement to declare an exception */
#define THROWS(x)
/* throw - for empty list */
#define NEW_THROWS
#define THROWS_EMPTY
/*
 * work around dangeling if/else combinations:
 */
#else
#define THROW(x) throw x
#define THROWS(x) throw(x)
#define NEW_THROWS throw()
#define THROWS_EMPTY throw()
#endif

    ])
])

