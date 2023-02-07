#ifndef GP_DATA_WRITER_H
#define GP_DATA_WRITER_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class GnuPlotDataWriter
{
public:
    GnuPlotDataWriter() = default;
    ~GnuPlotDataWriter() = default;

    static void writeDataDat(const std::vector<std::string>& titles,
                             const std::vector<std::vector<double>>& data,
                             const std::string& filename = "pid_controller.dat")
    {
        std::ofstream file;
        file.open(filename, std::fstream::out | std::fstream::trunc);

        file << "# ";
        for (const auto& title : titles)
            file << title << "\t";
        file << "\n";

        for (size_t row = 0; row < data[0].size(); row++)
        {
            for (size_t col = 0; col < data.size(); col++)
            {
                file << std::to_string(data[col][row]) << "\t";
            }
            file << "\n";
        }
    }
};

#endif