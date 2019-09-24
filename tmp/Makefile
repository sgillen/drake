# worlds worst makefile

all:
# mostly taken from the pybind11 tutorial page
	c++ -O3 -Wall -fpermissive -shared -std=c++11 -fPIC -D_GLIBCXX_USE_CXX11_ABI=1 `python3 -m pybind11 --includes` cc_regex.cc cc_regex_py.cc -o cc_regex`python3-config --extension-suffix`
clean:
	rm cc_regex.cpython-36m-x86_64-linux-gnu.so

