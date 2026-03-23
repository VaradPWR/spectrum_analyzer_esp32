# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/varad/esp-idf/components/bootloader/subproject"
  "/home/varad/spect2/build/bootloader"
  "/home/varad/spect2/build/bootloader-prefix"
  "/home/varad/spect2/build/bootloader-prefix/tmp"
  "/home/varad/spect2/build/bootloader-prefix/src/bootloader-stamp"
  "/home/varad/spect2/build/bootloader-prefix/src"
  "/home/varad/spect2/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/varad/spect2/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/varad/spect2/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
