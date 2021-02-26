#include "utils.hpp"
#include <iostream>

int main(int argc, char const *argv[])
{
	Eigen::Matrix4d t;

	t << 0, -1,  0,      0,
		   0,  0,  1, 5.976,
		  -1,  0,  0, -0.26,
		   0,  0,  0,      1;

	std::cout << invertTransformMatrix(t) << std::endl;

	return 0;
}