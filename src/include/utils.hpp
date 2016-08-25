#ifndef __ROBOT_UTILS_HPP_
#define __ROBOT_UTILS_HPP_
#include <iostream>

using std::cout;
using std::endl;

namespace robotutils
{

template<class T>
inline void printVector(std::vector<T> vec, std::string str)
{
    cout << str << ": ";
    for (auto & k : vec) {
        cout << k << ", ";
    }
    cout << endl;
}

template<class T>
void hash_combine(std::size_t& seed, T const& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

}

#endif
