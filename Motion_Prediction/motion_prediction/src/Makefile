# Usage:
# for the first time: make install_dependencies; make init; make
# from next time, just make or make debug 
#
# This file internally uses cmake.
# Makefile is reusable. Simply change the EXE variable.
EXE=mainlib
BIN_FOLDER=../build_$(EXE)
BIN_TO_SRC=../../src
SRC_TO_WORK=../work
SRC_TO_EXE=../work/$(EXE)
#NUM_CORES=$(shell if test -f /proc/cpuinfo; then cat /proc/cpuinfo | grep processor | wc -l;	else echo 4;fi)
NUM_CORES=$(shell if test -f /proc/cpuinfo; then grep ^cpu\\scores /proc/cpuinfo | uniq | awk '{print $$4}';	else echo 4;fi)

# all depends on init_finished.txt
gui: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory -j $(NUM_CORES)

gcc: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;make --no-print-directory -j $(NUM_CORES)

rmlib: init_finished.txt  
	rm -f ../work/libmainlib*.so
	rm -f ../work/console/libmainlib*.so

# when build is up-to-date, libmainlib.so won't be updated. so in that case, touch main.cpp and then make console.
console: init_finished.txt
	touch main.cpp
	if ! test -d $(BIN_FOLDER)/build_linux_console; then mkdir -p $(BIN_FOLDER)/build_linux_console ; cd $(BIN_FOLDER)/build_linux_console;  cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Release -D BUILD_CONSOLE=ON $(BIN_TO_SRC);fi
	cd $(BIN_FOLDER)/build_linux_console;make --no-print-directory -j $(NUM_CORES)

# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p $(BIN_FOLDER)
	mkdir -p $(BIN_FOLDER)/build_linux;cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Release $(BIN_TO_SRC)
	mkdir -p $(BIN_FOLDER)/build_linux_debug;cd $(BIN_FOLDER)/build_linux_debug;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug $(BIN_TO_SRC)
	touch init_finished.txt

clean:
	rm -rf $(BIN_FOLDER)
	rm -f ../work/libmainlib*.so
	rm init_finished.txt
cleanall:
	rm -rf ../build
	make clean
	
# this would probably install more libraries than necessary, but I guess it doens't matter for most.
install_dependencies:
	cd ../../taesooLib;make build_dependencies
	make build_dependencies
	sudo apt-get install python3-pip libpython3-dev libcupti-dev 
	pip3 install numpy
	sudo apt install python3-numpy
	pip3 install -r requirements.txt

build_dependencies:
	mkdir -p ../dump
	mkdir -p ../dump/dump
	if ! test -d ../Resource; then cd ..;ln -sf ../muscle_simulator/Resource Resource;fi
	if ! test -d ../media; then cd ..;ln -sf ../taesooLib/media media;fi
	if ! test -d ../media12; then cd ..;ln -sf ../taesooLib/media12 media12;fi
	if ! test -d ../media13; then cd ..;ln -sf ../taesooLib/media13 media13;fi
	cp ../../taesooLib/work/readline.so ../work/ 

setSeperateWin: # useful when creating high-quality captures for a demo video. 
	cp ../Resource/ogreconfig_linux_sepwin.txt ../Resource/ogreconfig_personal.txt
unsetSeperateWin: # go back to the default layout
	rm -f ../Resource/ogreconfig_personal.txt

install_dependencies_mac:
	cd ../../taesooLib;make build_dependencies_mac
	make build_dependencies
	pip3 install -r requirements.txt

install_dependencies_intel:
	cd ../../taesooLib;make build_dependencies_intelmac
	make build_dependencies
	pip3 install -r requirements.txt
