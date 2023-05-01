# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/alexyen/esp/esp-idf/components/bootloader/subproject"
  "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader"
  "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader-prefix"
  "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader-prefix/tmp"
  "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader-prefix/src"
  "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
