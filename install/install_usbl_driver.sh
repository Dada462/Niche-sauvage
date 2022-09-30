
cd $HOME
git clone https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver.git && cd seatrac_driver
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME ..
make -j4 install
echo $CMAKE_PREFIX_PATH
# echo "export CMAKE_PREFIX_PATH=$HOME/seatrac_driver:\$CMAKE_PREFIX_PATH" >> .bashrc
