#include <iostream>
#include <functional> // needed for std::function

// // callback function
// int add(int x, int y)
// {
//     return x + y;
// }

// // callback function
// int multiply(int x, int y)
// {
//     return x * y;
// }

// int caller(int x, int y, int (*func)(int, int))
// {
//     return func(x, y);
// }

// // with std::function
// // int caller(int x, int y, std::function<int(int, int)> func) {
// //     return func(x, y);
// //}

// int main()
// {
//     std::cout << caller(20, 10, &add) << '\n';
//     std::cout << caller(20, 10, &multiply) << '\n';
// }

// callback function 
int subtract(int x, int y)
{
    return x - y;
}

// callback function
int return_one()
{
    return 1;
}

int main()
{
    auto f = std::bind(&return_one); // f is a wrapper
    std::cout << f() << '\n';        // 1

    auto g = std::bind(&subtract, 10, std::placeholders::_1); // g is a wrapper
    std::cout << g(3) << '\n';                                // x is already set to 10, here we assign 3 to y

    auto h = std::bind(&subtract, std::placeholders::_1, 3); // h is a wrapper
    std::cout << h(10) << '\n';                              // y is already set to 3, here we assign 10 to x

    auto i = std::bind(&subtract, std::placeholders::_1, std::placeholders::_2); // i is a wrapper
    std::cout << i(10, 3) << '\n';                                               // we set x to 10 and y to 3
}