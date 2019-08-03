wizard_stick project

All setup is  provided by this article http://www.iwasz.pl/electronics/stm32-on-ubuntu-linux-step-by-step/

few changes :

to instal cmake with ninja
sudo apt install cmake ninja-build

path in qt for cmake toolchain instead of /home/ use ~/ 
qt if need will rewrite it. Cause in my case, with default from article receive errors

project with qt as ide


to build without qt, create build folder

cd build/
cmake -DCMAKE_CXX_COMPILER=arm-none-eabi-g++ \
    -DCMAKE_C_COMPILER=arm-none-eabi-gcc \
    -DCMAKE_TOOLCHAIN_FILE=../stm32f1xx.cmake -GNinja ..


to start gdb server :
openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg
