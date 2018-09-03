#ifndef BYTECONVERTER_H
#define BYTECONVERTER_H

#include <iostream>

typedef uint8_t byte;

struct BytesToUShort
{
    union
    {
        std::uint16_t bc;

        struct
        {
            byte b;
            byte c;
        };
    };
};

struct BytesToLong
{
    union
    {
        std::int32_t bcde;

        struct
        {
            byte b;
            byte c;
            byte d;
            byte e;
        };
    };
};

struct BytesToULong
{
    union
    {
        std::uint32_t bcde;

        struct
        {
            byte b;
            byte c;
            byte d;
            byte e;
        };
    };
};

struct BytesToFloat
{
    union
    {
        float abcd;

        struct
        {
            byte a;
            byte b;
            byte c;
            byte d;
        };
    };
};

struct BytesToDouble
{
    union
    {
        double abcdefgh;

        struct
        {
            byte a;
            byte b;
            byte c;
            byte d;
            byte e;
            byte f;
            byte g;
            byte h;
        };
    };
};

struct BytesToULongLong
{
    union
    {
        unsigned long long abcdefgh;

        struct
        {
            byte a;
            byte b;
            byte c;
            byte d;
            byte e;
            byte f;
            byte g;
            byte h;
        };
    };
};

class ByteConverter
{
public:
    static void bytesToLong(char* bytes, long *result);
    static void bytesToUnsignedLong(char* bytes, unsigned long *result);
    static void bytesToUnsignedLongLong(char* bytes, unsigned long long *result);
    static void bytesToUnsignedShort(char* bytes, unsigned short *result);
    static void bytesToDouble(char* bytes, double *result);
    static void bytesToFloat(char* bytes, float *result);
};

#endif // BYTECONVERTER_H
