dnl Copyright (C) 2001-2005 Open Source Telecom Corporation.
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

AC_DEFUN([OST_SYS_SOCKET],[
    AC_REQUIRE([OST_CC_SYSTIME])
    AC_REQUIRE([OST_CC_TYPES])
    ost_cv_inet_sockets=no
    ost_cv_unix_sockets=no
    ost_cv_lib_socket="c"
    ost_cv_ipv6=yes
    ost_cv_nat=yes
    ost_cv_nat_detected=no
    SOCKET_LIBS=""
    AC_CHECK_HEADERS(net/if.h)
    AC_CHECK_HEADERS(sys/socket.h,[
        AC_CHECK_HEADERS(select.h sys/select.h netinet/in_systm.h netinet/ip.h)
        AC_CHECK_HEADERS(netinet/inet.h netinet/in.h)
        AC_CHECK_HEADERS(arpa/inet.h, ost_cv_inet_sockets=yes)
        AC_CHECK_HEADERS(sys/sockio.h)
        AC_CHECK_HEADERS(sys/un.h, ost_cv_unix_sockets=yes)
    ],[
        AC_CHECK_HEADERS(winsock2.h winsock.h,[
            ost_cv_lib_socket="wsock32"
            SOCKET_LIBS="-lwsock32 -liberty -lws2_32"
            ost_cv_inet_sockets=yes
        ])
    ])
    AC_CHECK_LIB(socket, socket, [
        ost_cv_lib_socket="socket"
        SOCKET_LIBS="-lsocket"
    ])

    if test $ost_cv_inet_sockets = yes ; then
        AC_CHECK_LIB(${ost_cv_lib_socket}, getaddrinfo,[
            AC_DEFINE(HAVE_GETADDRINFO, [1], [getaddrinfo interface support])
        ])
        AC_ARG_WITH(ipv6, [  --without-ipv6          Disable ipv6],
            ost_cv_ipv6=no,[
            AC_CHECK_HEADERS(netinet6/in6.h linux/in6.h)
            AC_CHECK_LIB(${ost_cv_lib_socket}, inet_pton,[
                AC_DEFINE(HAVE_INET_PTON, [1], [ipv6 support])])
            AC_CHECK_LIB(${ost_cv_lib_socket}, gethostbyname2,[
                AC_DEFINE(HAVE_GETHOSTBYNAME2, [1], [ipv6 host lookup])])
        ])
        AC_CHECK_HEADERS(sys/libcsys.h)
        AC_DEFINE(HAVE_INET_SOCKETS, [1], [inet sockets])
        AC_CHECK_LIB(${ost_cv_lib_socket}, inet_aton,[
            AC_DEFINE(HAVE_INET_ATON, [1], [has inet_aton])])
        AC_CACHE_CHECK([for socklen_t defined], ost_cv_socklen_t, [
            ost_cv_socklen_t='no'
            AC_EGREP_HEADER(socklen_t, sys/socket.h,
                ost_cv_socklen_t='yes',[
                AC_EGREP_HEADER(socklen_t, cygwin/socket.h,
                    ost_cv_socklen_t='yes')])
        ])
        if test $ost_cv_socklen_t = yes ; then
            AC_DEFINE(HAVE_SOCKLEN_T, [1], [has socklen_t type])
        fi

        AC_ARG_WITH([nat],
            [AC_HELP_STRING([--without-nat],[ Disable NAT class interface (pf, ipf or netfilter), default is enabled.])],
            [ost_cv_nat=no]
        )

        if test "$ost_cv_nat" = "yes" ; then

            AC_CHECK_HEADERS([errno.h limits.h sys/types.h sys/socket.h sys/ioctl.h unistd.h])
            AC_CHECK_HEADERS([net/if.h],,,
                [#ifdef HAVE_SYS_SOCKET_H
                #include <sys/socket.h>
                #endif])

            if test "$ost_cv_nat_detected" = "no" ; then
                AC_CHECK_HEADERS([linux/netfilter_ipv4.h linux/netfilter_ipv6.h],,,
                    [#ifdef HAVE_LIMITS_H
                    #include <limits.h>
                    #endif])
                if test "$ac_cv_header_linux_netfilter_ipv4_h" = "yes" &&
                   [ test "$ac_cv_header_linux_netfilter_ipv6_h" = "yes" &&
                     test "$ost_cv_ipv6" = "yes" ||
                     test "$ost_cv_ipv6" = "no" ] ; then
                    AC_DEFINE(HAVE_NAT_NETFILTER, [1], [NetFilter NAT support])
                    ost_cv_nat_detected="yes"
                fi
            fi

            if test "$ost_cv_nat_detected" = "no" ; then
                ip_filter_compat="no"
                AC_CHECK_HEADERS([netinet/ip_compat.h ip_compat.h netinet/ip_fil_compat.h ip_fil_compat.h],
                    [ip_filter_compat=$ac_header
                    break],,
                    [#ifdef HAVE_SYS_TYPES_H
                    #include <sys/types.h>
                    #endif
                    #ifdef HAVE_SYS_SOCKET_H
                    #include <sys/socket.h>
                    #endif
                    #ifdef HAVE_NETINET_IN_H
                    #include <netinet/in.h>
                    #endif])
                if test "$ip_filter_compat" != "no" ; then
                    ip_filter_fil="no"
                    AC_CHECK_HEADERS([netinet/ip_fil.h ip_fil.h],
                        [ip_filter_fil=$ac_header
                        break],,
                                            [#ifdef HAVE_SYS_TYPES_H
                        #include <sys/types.h>
                        #endif
                        #ifdef HAVE_SYS_SOCKET_H
                        #include <sys/socket.h>
                        #endif
                        #ifdef HAVE_NETINET_IN_H
                        #include <netinet/in.h>
                        #endif
                        #include <$ip_filter_compat>])
                    ip_filter_nat="no"
                    AC_CHECK_HEADERS([netinet/ip_nat.h ip_nat.h],
                        [ip_filter_nat=$ac_header
                        break],,
                                            [#ifdef HAVE_SYS_TYPES_H
                        #include <sys/types.h>
                        #endif
                        #ifdef HAVE_SYS_SOCKET_H
                        #include <sys/socket.h>
                        #endif
                        #ifdef HAVE_NETINET_IN_H
                        #include <netinet/in.h>
                        #endif
                        #include <$ip_filter_compat>
                        #include <$ip_filter_fil>
                        #ifdef HAVE_NET_IF_H
                        #include <net/if.h>
                        #endif])
                    if test "$ip_filter_fil" != "no" &&
                       test "$ip_filter_nat" != "no" ; then
                        AC_DEFINE(HAVE_NAT_IPF, [1], [IPF NAT support])
                        ost_cv_nat_detected="yes"
                    fi
                fi
            fi

            if test "$ost_cv_nat_detected" = "no" ; then
                AC_CHECK_HEADERS([net/pfvar.h],,,
                [#ifdef HAVE_SYS_TYPES_H
                #include <sys/types.h>
                #endif
                #ifdef HAVE_SYS_SOCKET_H
                #include <sys/socket.h>
                #endif
                #ifdef HAVE_NET_IF_H
                #include <net/if.h>
                #endif
                #ifdef HAVE_NETINET_IN_H
                #include <netinet/in.h>
                #endif])
                if test "$ac_cv_header_net_pfvar.h" = "yes" ; then
                    AC_DEFINE(HAVE_NAT_PF, [1], [PF NAT support])
                    ost_cv_nat_detected="yes"
                fi
            fi

            if test "$ost_cv_nat_detected" = "yes"; then
                AC_DEFINE(CCXX_NAT, [1], [NAT support])
            else
                AC_MSG_WARN([-------------------------------------------------------])
                AC_MSG_WARN([Could not autodetect NAT engine - pf, ipf or netfilter.])
                AC_MSG_WARN([NAT engine c++ class interface support was disabled.   ])
                AC_MSG_WARN([This is not necessarilly a problem. If you do not plan ])
                AC_MSG_WARN([to use or don't know what this is, than it is safe to  ])
                AC_MSG_WARN([assume that you don't need it.             ])
                AC_MSG_WARN([-------------------------------------------------------])
                ost_cv_nat="no"
            fi
        fi
    fi

    if test $ost_cv_unix_sockets = yes ; then
        AC_DEFINE(HAVE_UNIX_SOCKETS, [1], [has unix domain sockets])
    fi
    THREAD_LIBS="$SOCKET_LIBS $THREAD_LIBS"
    AC_SUBST(SOCKET_LIBS)
    AH_BOTTOM([

#ifdef HAVE_SYS_LIBCSYS_H
#include <sys/libcsys.h>
#endif

#ifdef HAVE_WINSOCK2_H
#include <winsock2.h>
#else
#ifdef HAVE_WINSOCK_H
#include <winsock.h>
#else
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#ifdef HAVE_SELECT_H
#include <select.h>
#else
#ifdef HAVE_SYS_SELECT_H
#include <sys/select.h>
#endif
#endif

#ifdef HAVE_NETINET_IN_H
#if defined(__hpux) && defined(_XOPEN_SOURCE_EXTENDED)
#undef _XOPEN_SOURCE_EXTENDED
#endif
#include <netinet/in.h>
#ifdef  __hpux
#define _XOPEN_SOURCE_EXTENDED
#endif
#endif
#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#include <netdb.h>
#endif

#ifdef  HAVE_NETINET6_IN6_H
#include <netinet6/in6.h>
#endif

#ifdef  HAVE_LINIX_IN6_H
#include <linux/in6.h>
#endif

#ifdef HAVE_NETINET_IN_SYSTM_H
#include <netinet/in_systm.h>
#endif
#ifdef HAVE_NETINET_IP_H
#include <netinet/ip.h>
#endif
#ifdef HAVE_SYS_UN_H
#include <sys/un.h>
#endif
#endif
#endif
#endif

#ifndef HAVE_INET_ATON
#define inet_aton(cp, addr) (((*(unsigned long int *)(addr)) = inet_addr(cp)) != -1)
#endif

#ifndef SUN_LEN
#ifdef SCM_RIGHTS
#define HAVE_UN_LEN
#endif
#ifdef __linux__
#define HAVE_UN_LEN
#endif
#ifdef HAVE_UN_LEN
#define SUN_LEN(ptr) sizeof(sockaddr_un.sun_len) + sizeof(sockaddr_un.sun_family) + sizeof(sockaddr_un.sun_path) + 1
#else
#define SUN_LEN(ptr) ((size_t)((struct sockaddr_un *)0)->sun_path) + strlen((ptr)->sun_path))
#endif
#endif

#ifndef _OSF_SOURCE
#ifndef HAVE_SOCKLEN_T
#if defined(i386) && defined(__svr4__)
#define HAVE_SOCKLEN_U
#else
#if defined(__CYGWIN32__)
#define socklen_t int
#else
typedef int socklen_t;
#endif
#endif

#ifdef HAVE_SOCKLEN_U
#if !defined(__CYGWIN32__) && !defined(__MINGW32__)
typedef unsigned socklen_t;
#else
typedef int socklen_t;
#endif
#endif
#endif
#endif

#ifdef  __hpux
#ifdef  mutable
#undef  mutable
#endif
#endif

#if defined(AF_INET6) && defined(HAVE_INET_PTON)
#define CCXX_IPV6
#endif

#define CCXX_MULTIFAMILY_IP

    ])
])

