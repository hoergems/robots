#ifndef __ROBOT_UTILS_HPP_
#define __ROBOT_UTILS_HPP_

using std::cout;
using std::endl;

namespace robotutils {

void printVector(std::vector<double> vec, std::string str) {
	cout << str << ": ";
	for (auto &k: vec) {
		cout << k << ", ";
	}
	cout << endl;
}

}

#endif