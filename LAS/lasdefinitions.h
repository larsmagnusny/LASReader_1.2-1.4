#ifndef LASDEFINITIONS_H
#define LASDEFINITIONS_H

struct PUBLIC_HEADER_BLOCK
{
    char fileSignature[4];
    unsigned short fileSourceId;
    unsigned short globalEncoding;
    unsigned long projectID_GUID_data1;
    unsigned short projectID_GUID_data2;
    unsigned short projectID_GUID_data3;
    unsigned char projectID_GUID_data4[8];
    unsigned char versionMajor;
    unsigned char versionMinor;
    char systemIdentifier[32];
    char generatingSoftware[32];
    unsigned short fileCreationDayOfYear;
    unsigned short fileCreationYear;
    unsigned short headerSize;
    unsigned long offsetToPointData;
    unsigned long numberOfVariableLengthRecords;
    unsigned char pointDataRecordFormat;
    unsigned short pointDataRecordLength;
    unsigned long legacyNumberOfPointRecords;
    unsigned long legacyNumberOfPointsByReturn[5];
    double xScaleFactor;
    double yScaleFactor;
    double zScaleFactor;
    double xOffset;
    double yOffset;
    double zOffset;
    double maxX;
    double minX;
    double maxY;
    double minY;
    double maxZ;
    double minZ;
    unsigned long long startOfWaveformDataPacketRecord;
    unsigned long long startOfFirstExtendedVariableLengthRecord;
    unsigned long numberOfExtendedVariableLengthRecords;
    unsigned long long numberOfPointRecords;
    unsigned long long numberOfPointsByReturn[15];
};

#endif // LASDEFINITIONS_H
