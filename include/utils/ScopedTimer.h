#pragma once

#include <string>
#include <chrono>
#include <iostream>
#include <utility>

class ScopedTimer {
public:
    explicit ScopedTimer(std::string  name) : name(std::move(name)), t_start(std::chrono::high_resolution_clock::now()) {}

    ~ScopedTimer() {
        std::chrono::duration<double, std::milli> ms = std::chrono::high_resolution_clock::now() - t_start;
        std::cout << name << ": " << ms.count() << "ms" << std::endl;
    }

private:
    const std::string name;
    const std::chrono::time_point<std::chrono::high_resolution_clock> t_start;
};
