cmake_minimum_required(VERSION 2.8)
project(ccrtp)
find_package(qibuild)

include_directories( "." "src" )
add_definitions( -DHAVE_CONFIG_H -D_GNU_SOURCE )

qi_create_lib( ccrtp
  SRC src/rtppkt.cpp
      src/rtcppkt.cpp
      src/source.cpp
      src/data.cpp
      src/incqueue.cpp
      src/outqueue.cpp
      src/queue.cpp
      src/control.cpp
      src/members.cpp
      src/socket.cpp
      src/duplex.cpp
      src/pool.cpp
      src/CryptoContext.cxx
#      src/ccrtp/crypto/gcrypt/gcrypthmac.cxx
#      src/ccrtp/crypto/gcrypt/gcryptAesSrtp.cxx
#      src/ccrtp/crypto/gcrypt/InitializeGcrypt.cxx
#      src/ccrtp/crypto/openssl/hmac.cxx
#      src/ccrtp/crypto/openssl/AesSrtp.cxx
#      src/ccrtp/crypto/openssl/InitializeOpenSSL.cxx
  SHARED
)

qi_install_header( src/ccrtp/*.h SUBFOLDER ccrtp )
                                #cc++/buffer.h
                                #cc++/cmdoptns.h
                                #cc++/common.h
                                #cc++/config.h
                                #cc++/counter.h
                                #cc++/digest.h
                                #cc++/exception.h
                                #cc++/export.h
                                #cc++/file.h
                                #cc++/functions.h
                                #cc++/mime.h
                                #cc++/misc.h
                                #cc++/missing.h
                                #cc++/network.h
                                #cc++/numbers.h
                                #cc++/objcount.h
                                #cc++/object.h
                                #cc++/objlink.h
                                #cc++/objmap.h
                                #cc++/objsync.h
                                #cc++/oststring.h
                                #cc++/persist.h
                                #cc++/pointer.h
                                #cc++/process.h
                                #cc++/serial.h
                                #cc++/slog.h
                                #cc++/socket.h
                                #cc++/socketport.h
                                #cc++/strchar.h
                                #cc++/string.h
                                #cc++/template.h
                                #cc++/thread.h
                                #cc++/tokenizer.h
                                #cc++/unix.h
                                #cc++/url.h
                                #cc++/xml.h
                                #cc++/zstream.h
#)

qi_use_lib(ccrtp rt pthread dl ccgnu2 crypto)
qi_stage_lib(ccrtp)
