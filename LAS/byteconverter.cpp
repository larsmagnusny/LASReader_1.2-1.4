#include "byteconverter.h"

void ByteConverter::bytesToLong(char *bytes, long *result)
{
    BytesToLong btl;
    btl.b = bytes[0];
    btl.c = bytes[1];
    btl.d = bytes[2];
    btl.e = bytes[3];

    *result = btl.bcde;
}

void ByteConverter::bytesToUnsignedLong(char *bytes, unsigned long *result)
{
    BytesToULong btul;
    btul.b = bytes[0];
    btul.c = bytes[1];
    btul.d = bytes[2];
    btul.e = bytes[3];

    *result = btul.bcde;
}

void ByteConverter::bytesToUnsignedLongLong(char *bytes, unsigned long long *result)
{
    BytesToULongLong btull;

    btull.a = bytes[0];
    btull.b = bytes[1];
    btull.c = bytes[2];
    btull.d = bytes[3];
    btull.e = bytes[4];
    btull.f = bytes[5];
    btull.g = bytes[6];
    btull.h = bytes[7];

    *result = btull.abcdefgh;
}

void ByteConverter::bytesToUnsignedShort(char *bytes, unsigned short *result)
{
    BytesToUShort bts;
    bts.b = bytes[0];
    bts.c = bytes[1];

    *result = bts.bc;
}

void ByteConverter::bytesToDouble(char *bytes, double* result)
{
    BytesToDouble btd;

    btd.a = bytes[0];
    btd.b = bytes[1];
    btd.c = bytes[2];
    btd.d = bytes[3];
    btd.e = bytes[4];
    btd.f = bytes[5];
    btd.g = bytes[6];
    btd.h = bytes[7];

    *result = btd.abcdefgh;
}

void ByteConverter::bytesToFloat(char *bytes, float *result)
{
    BytesToFloat btf;

    btf.a = bytes[0];
    btf.b = bytes[1];
    btf.c = bytes[2];
    btf.d = bytes[3];

    *result = btf.abcd;
}
