if not exist "build_win" md "build_win"
cd build_win
cmake -D BUILD_EXE=ON .. 
@echo off
echo off
@SET /P uname=Press Enter to launch visual studio
start QP_controller.sln
exit
