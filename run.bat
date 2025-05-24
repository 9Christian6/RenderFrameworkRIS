if not exist "vcpkg" (
    git clone https://github.com/microsoft/vcpkg
    if %errorlevel% neq 0 exit /b %errorlevel%
    vcpkg\bootstrap-vcpkg.bat
)

if not exist build (
    vcpkg\vcpkg install sdl2 libpng libjpeg-turbo tiff yaml-cpp --triplet x64-windows
    if %errorlevel% neq 0 exit /b %errorlevel%

    cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=vcpkg/scripts/buildsystems/vcpkg.cmake -DDISABLE_GUI=1
    if %errorlevel% neq 0 exit /b %errorlevel%
)

cmake --build build --config Release
if %errorlevel% neq 0 exit /b %errorlevel%

build\src\Release\arty.exe %*