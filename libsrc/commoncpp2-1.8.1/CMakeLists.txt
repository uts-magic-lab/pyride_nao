cmake_minimum_required(VERSION 2.8)
project(commoncpp2)
find_package(qibuild)

include_directories( "." "inc" )
add_definitions( -DHAVE_CONFIG_H -D_GNU_SOURCE -DCCXX_EXPORT_LIBRARY )

qi_create_lib( ccext2
  SRC config.h 
      src/numbers.cpp
      src/zstream.cpp
      src/socketport.cpp
      src/url.cpp
      src/xml.cpp
      src/persist.cpp
      src/engine.cpp
      src/digest.cpp
      src/cmdoptns.cpp
      src/date.cpp
      src/md5.cpp
      src/unix.cpp
      src/network.cpp
      src/serial.cpp
      src/urlstring.cpp
      src/tokenizer.cpp
      src/mime.cpp
      src/ssl.cpp
      src/applog.cpp
  SHARED
)

qi_create_lib( ccgnu2
  SRC src/thread.cpp
      src/mutex.cpp
      src/semaphore.cpp
      src/threadkey.cpp
      src/friends.cpp
      src/event.cpp
      src/slog.cpp
      src/dir.cpp
      src/file.cpp
      src/inaddr.cpp
      src/peer.cpp
      src/timer.cpp
      src/socket.cpp
      src/strchar.cpp
      src/simplesocket.cpp
      src/mempager.cpp
      src/keydata.cpp
      src/dso.cpp
      src/exception.cpp
      src/missing.cpp
      src/process.cpp
      src/string.cpp
      src/in6addr.cpp
      src/buffer.cpp
      src/lockfile.cpp
      src/nat.cpp
      src/runlist.cpp
      src/assoc.cpp
      src/pointer.cpp
      src/linked.cpp
      src/map.cpp
      src/cidr.cpp
      src/private.h
      src/nat.h
  SHARED
)

qi_install_header( inc/cc++/*.h SUBFOLDER cc++ )
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

qi_use_lib(ccgnu2 rt pthread dl)
qi_stage_lib(ccgnu2)

qi_use_lib(ccext2 rt pthread zlib)
qi_stage_lib(ccext2)
