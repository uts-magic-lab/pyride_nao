# CMake file for celt-0.11.1

cmake_minimum_required(VERSION 2.8)
project(celt-0.11.1)
find_package(qibuild)

include_directories( "." "src" )
add_definitions( -DHAVE_CONFIG_H -D_GNU_SOURCE )

qi_create_lib( celt
  SRC libcelt/bands.c libcelt/celt.c libcelt/cwrs.c 
      libcelt/ecintrin.h libcelt/entcode.c
      libcelt/entdec.c libcelt/entenc.c libcelt/header.c
      libcelt/kiss_fft.c libcelt/laplace.c libcelt/mathops.c
      libcelt/mdct.c libcelt/modes.c libcelt/pitch.c 
      libcelt/plc.c libcelt/quant_bands.c libcelt/rate.c 
      libcelt/vq.c
  SHARED
)

qi_install_header( libcelt/celt.h 
                   libcelt/celt_types.h
                   libcelt/celt_header.h )

#qi_use_lib(celt )
qi_stage_lib(celt)

# qi_create_lib(...)

# qi_create_bin(...)

