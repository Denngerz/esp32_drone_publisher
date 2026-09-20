#include "../../include/mt/ConsoleLog.hpp"

#include <iostream>
#include <mutex>

namespace console
{

namespace
{
// One mutex for both streams: they share a terminal, so interleaving between
// them is just as unreadable as interleaving within one.
std::mutex& consoleMutex()
{
    static std::mutex m;
    return m;
}
} // namespace

void line(const std::string& text)
{
    std::lock_guard<std::mutex> lock(consoleMutex());
    std::cout << text << '\n';
}

void error(const std::string& text)
{
    std::lock_guard<std::mutex> lock(consoleMutex());
    std::cerr << text << '\n';
}

} // namespace console
