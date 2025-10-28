# Exit on error
set -Eeo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)"

# Install TASER C++ dependencies
sudo apt update
sudo apt install -y \
	libboost-all-dev \
	libeigen3-dev \
	libgtest-dev \
	pybind11-dev

# Install Osqp and OsqpEigen
git clone --recursive --branch v0.6.3 https://github.com/osqp/osqp /tmp/taser/osqp
mkdir -p /tmp/taser/osqp/build && cd /tmp/taser/osqp/build
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX:PATH=/usr ..
make && sudo make install

git clone --branch v0.8.1 https://github.com/robotology/osqp-eigen.git /tmp/taser/osqp-eigen
mkdir -p /tmp/taser/osqp-eigen/build && cd /tmp/taser/osqp-eigen/build
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr ..
make && sudo make install
export OsqpEigen_DIR=/usr

# Install the TASER package
/isaac-sim/python.sh -m pip install --verbose -e "${PROJECT_DIR}"
