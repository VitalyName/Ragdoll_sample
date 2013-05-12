@echo off

SET MinGW_dir="c:\MinGW\bin"
SET CMake_exe="C:\Program Files (x86)\CMake 2.8\bin\cmake.exe"

::set dirs
SET Project_sources_dir="."
SET Project_builded_dir="."


echo =======================================================
echo ======== Build projects with minGW ====================
echo =======================================================

MKDIR %Project_builded_dir%
CD %Project_builded_dir%

%CMake_exe% -G "MinGW Makefiles" %Project_sources_dir%

%CMake_exe% -D CMAKE_BUILD_TYPE:STRING=Release %Project_sources_dir%
mingw32-make.exe
mingw32-make.exe install

%CMake_exe% -D CMAKE_BUILD_TYPE:STRING=Debug %Project_sources_dir%
mingw32-make.exe
mingw32-make.exe install

pause