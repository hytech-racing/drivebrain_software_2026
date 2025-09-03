rm -rf .venv
rm -rf build
rm -rf cmake

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

conan profile detect
conan install . --build=missing --profile:build=default --profile:host=rpi_profile -of=cmake

mkdir build
cd build
cmake ..  -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../cmake/conan_toolchain.cmake
make -j