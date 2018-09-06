#include "lasreader_alex.h"
#include <iostream>
#include <fstream>
#include <QDebug>
#include "vertex.h"
#include "vec2.h"
#include "vec3.h"

LASReaderAlex::LASReaderAlex(const std::string &path)
{
    mHeaderStock = HeaderStock();
    std::ifstream inf(path, std::ios::binary);
    readHeaderStock(inf);
    readPointData(inf);
}

void LASReaderAlex::readHeaderStock(std::istream &stream)
{
    binaryRead(stream, mHeaderStock.fileSignature);
    binaryRead(stream, mHeaderStock.fileSourceId);
    binaryRead(stream, mHeaderStock.globalEncoding);
    binaryRead(stream, mHeaderStock.projectID_GUID_data1);
    binaryRead(stream, mHeaderStock.projectID_GUID_data2);
    binaryRead(stream, mHeaderStock.projectID_GUID_data3);
    binaryRead(stream, mHeaderStock.projectID_GUID_data4);
    binaryRead(stream, mHeaderStock.versionMajor);
    binaryRead(stream, mHeaderStock.versionMinor);
    binaryRead(stream, mHeaderStock.systemIdentifier);
    binaryRead(stream, mHeaderStock.generatingSoftware);
    binaryRead(stream, mHeaderStock.fileCreationDayOfYear);
    binaryRead(stream, mHeaderStock.fileCreationYear);
    binaryRead(stream, mHeaderStock.headerSize);
    binaryRead(stream, mHeaderStock.offsetToPointData);
    binaryRead(stream, mHeaderStock.numberOfVariableLengthRecords);
    binaryRead(stream, mHeaderStock.pointDataRecordFormat);
    binaryRead(stream, mHeaderStock.pointDataRecordLength);
    binaryRead(stream, mHeaderStock.legacyNumberOfPointRecords);
    binaryRead(stream, mHeaderStock.legacyNumberOfPointsByReturn);
    binaryRead(stream, mHeaderStock.xScaleFactor);
    binaryRead(stream, mHeaderStock.yScaleFactor);
    binaryRead(stream, mHeaderStock.zScaleFactor);
    binaryRead(stream, mHeaderStock.xOffset);
    binaryRead(stream, mHeaderStock.yOffset);
    binaryRead(stream, mHeaderStock.zOffset);
    binaryRead(stream, mHeaderStock.maxX);
    binaryRead(stream, mHeaderStock.minX);
    binaryRead(stream, mHeaderStock.maxY);
    binaryRead(stream, mHeaderStock.minY);
    binaryRead(stream, mHeaderStock.maxZ);
    binaryRead(stream, mHeaderStock.minZ);
}

void LASReaderAlex::readPointData(std::istream &stream)
{
    long x, y, z;
    float decimals = 100.0f;
    gsl::Vec3 pos;
    mPointData.reserve(mHeaderStock.legacyNumberOfPointRecords);

    // Reads all points
    for(unsigned int i = 0; i < mHeaderStock.legacyNumberOfPointRecords; ++i)
    {
        stream.seekg(mHeaderStock.offsetToPointData + mHeaderStock.pointDataRecordLength * i);
        binaryRead(stream, x);
        binaryRead(stream, y);
        binaryRead(stream, z);
        pos.setX((static_cast<float>(x) / decimals) - (static_cast<float>(mHeaderStock.minX)));
        pos.setZ((static_cast<float>(y) / decimals) - (static_cast<float>(mHeaderStock.minY)));
        pos.setY((static_cast<float>(z) / decimals) - (static_cast<float>(mHeaderStock.minZ)));
        mPointData.push_back(Vertex{pos, gsl::Vec3{}, gsl::Vec2{}});
    }
}

std::vector<Vertex> &LASReaderAlex::getPointData()
{
    return  mPointData;
}

const HeaderStock &LASReaderAlex::getHeaderStock()
{
    return mHeaderStock;
}
