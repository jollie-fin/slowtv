#include <iostream>
#include <fstream>
#include <string>
#include "telemetry.h"

using namespace std::literals;

int main(int argc, char **argv)
{
    std::ios::sync_with_stdio(false);

    if (argc != 3) {
        std::cerr << "Usage " << argv[0] << " input output" << std::endl;
        std::cerr << "Use - for stdin or stdout" << std::endl;
        return 1;
    }

    std::streambuf *in_buf = nullptr;
    std::streambuf *out_buf = nullptr;
    std::ofstream out_file;
    std::ifstream in_file;
    if (argv[2] == "-"s)
    {
        out_buf = std::cout.rdbuf();
    }
    else
    {
        out_file.open(argv[2]);
        out_buf = out_file.rdbuf();
    }

    if (argv[1] == "-"s)
    {
        in_buf = std::cin.rdbuf();
    }
    else
    {
        in_file.open(argv[1]);
        in_buf = in_file.rdbuf();
    }

    std::ostream out(out_buf);
    std::istream in(in_buf);

    try
    {
        decode(out, in);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
    catch(const char *e)
    {
        std::cerr << e << '\n';
        return 1;
    }

    return 0;
}