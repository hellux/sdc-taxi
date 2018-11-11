#include <iostream>

// TODO include opencv

extern "C" void ip_process(void);

void ip_process(void) {
    std::cout << "hej från c++" << std::endl;
}
