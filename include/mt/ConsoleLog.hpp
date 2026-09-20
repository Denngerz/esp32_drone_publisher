#pragma once
// ConsoleLog.hpp — one lock between everything that prints.
//
// Several threads narrate at once: the seeker's reader, the bay's reader and
// the mission. std::cout will not corrupt itself, but it is free to interleave
// two lines mid-word, and it does — "Bay reports Seeker holds VOG-17 mass=5
// track(s)" is what two readers printing at the same moment actually produced.
//
// Each call here emits one whole line under one lock, so a line is either
// there or not, never braided into another.

#include <string>

namespace console
{

void line(const std::string& text);    // to stdout
void error(const std::string& text);   // to stderr

} // namespace console
