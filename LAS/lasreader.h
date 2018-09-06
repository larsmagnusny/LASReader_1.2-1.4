#ifndef LASREADER_H
#define LASREADER_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <chrono>
#include "LAS/headerstock.h"
#include "vec3.h"

using namespace std;
using namespace gsl;

struct Point
{
    long X;
    long Y;
    long Z;
};

typedef vector<double> Array;
typedef vector<Array> Matrix;
typedef vector<Matrix> Image;

class LASReader
{
public:
    LASReader(string filename);
    ~LASReader();
    void readPointDataFormat(ifstream& s);
    double** makeHeightmapFromPointData(unsigned int &height, unsigned int &width);
    Matrix getGaussian(int height, int width, double sigma);
    double** applyFilter(double** image, Matrix &filter, int& width, int& height);
    Point getPoint(unsigned long long index);

    Point* point();

    unsigned long long mNumPoints{0};
    HeaderStock mHeader;
private:
    template<typename T>
    std::istream &binaryRead(std::istream &stream, T &value);

    template<typename T>
    void binaryConvert(char* buffer, T &value);

    Point* mPointData = nullptr;

    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;


    char ushortbuffer[2];
    char ulongbuffer[4];
    char ulonglongbuffer[8];
};

template<typename T>
std::istream &LASReader::binaryRead(std::istream &stream, T &value)
{
    return stream.read(reinterpret_cast<char*>(&value), sizeof(T));
}

template<typename T>
void LASReader::binaryConvert(char* buffer, T &value)
{
    memcpy(reinterpret_cast<char*>(&value), buffer, sizeof(T));
}

#endif // LASREADER_H
