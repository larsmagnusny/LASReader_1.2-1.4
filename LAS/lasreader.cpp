#include "lasreader.h"
#include "constants.h"
#include "gltypes.h"
#include <limits>
#include <cassert>

LASReader::LASReader(string filename)
{
    ifstream s(assetFilePath + filename, ios::in | ios::binary);

    start = std::chrono::high_resolution_clock::now();

    if(s.is_open())
    {
        binaryRead(s, mHeader.fileSignature);
        binaryRead(s, mHeader.fileSourceId);
        binaryRead(s, mHeader.globalEncoding);
        binaryRead(s, mHeader.projectID_GUID_data1);
        binaryRead(s, mHeader.projectID_GUID_data2);
        binaryRead(s, mHeader.projectID_GUID_data3);
        binaryRead(s, mHeader.projectID_GUID_data4);
        binaryRead(s, mHeader.versionMajor);
        binaryRead(s, mHeader.versionMinor);
        binaryRead(s, mHeader.systemIdentifier);
        binaryRead(s, mHeader.generatingSoftware);
        binaryRead(s, mHeader.fileCreationDayOfYear);
        binaryRead(s, mHeader.fileCreationYear);
        binaryRead(s, mHeader.headerSize);
        binaryRead(s, mHeader.offsetToPointData);
        binaryRead(s, mHeader.numberOfExtendedVariableLengthRecords);
        binaryRead(s, mHeader.pointDataRecordFormat);
        binaryRead(s, mHeader.pointDataRecordLength);
        binaryRead(s, mHeader.legacyNumberOfPointRecords);
        binaryRead(s, mHeader.legacyNumberOfPointsByReturn);
        binaryRead(s, mHeader.xScaleFactor);
        binaryRead(s, mHeader.yScaleFactor);
        binaryRead(s, mHeader.zScaleFactor);
        binaryRead(s, mHeader.xOffset);
        binaryRead(s, mHeader.yOffset);
        binaryRead(s, mHeader.zOffset);
        binaryRead(s, mHeader.maxX);
        binaryRead(s, mHeader.minX);
        binaryRead(s, mHeader.maxY);
        binaryRead(s, mHeader.minY);
        binaryRead(s, mHeader.maxZ);
        binaryRead(s, mHeader.minZ);

        // IF THE VERSION IS 1.3, theese can be read:
        if(mHeader.versionMajor == 1 && mHeader.versionMinor == 3)
        {
            binaryRead(s, mHeader.startOfWaveformDataPacketRecord);
            // IF THE VERSION IS 1.4, theese can be read:
            if(mHeader.versionMinor == 4)
            {
                binaryRead(s, mHeader.startOfFirstExtendedVariableLengthRecord);
                binaryRead(s, mHeader.numberOfExtendedVariableLengthRecords);
                binaryRead(s, mHeader.numberOfPointRecords);
                binaryRead(s, mHeader.numberOfPointsByReturn);
            }
        }

        cout << "Done reading header..." << endl;
		
        readPointDataFormat(s);

        s.close();
    }
    else
    {
        cout << "Failed to open: " << filename << endl;
    }
}

LASReader::~LASReader()
{
    if(mPointData)
        delete[] mPointData;
}

void LASReader::readPointDataFormat(ifstream &s)
{
    bool using14 = mHeader.versionMajor == 1 && mHeader.versionMinor == 4;

    size_t currentPos = mHeader.offsetToPointData;

    long currentPoint[3];

    mNumPoints = using14 ? mHeader.numberOfPointRecords : mHeader.legacyNumberOfPointRecords;

    if(mPointData)
        delete[] mPointData;

    mPointData = new Point[mNumPoints];

    int recordsToRead = 10000;
    long long recordsLeft = static_cast<long long>(mNumPoints);
    long long recordsRead = 0;
    long long totalPointsRead = 0;

    char* data = new char[recordsToRead * mHeader.pointDataRecordLength];

    // Try to read all the point data records
    s.seekg(mHeader.offsetToPointData);

    while(recordsLeft > 0)
    {
        if(recordsLeft - recordsToRead >= 0)
        {
            s.read(data, static_cast<int>(recordsToRead * mHeader.pointDataRecordLength));
            recordsRead = recordsToRead;
        }
        else
        {
            s.read(data, static_cast<int>(recordsLeft * mHeader.pointDataRecordLength));
            recordsRead = recordsLeft;
        }

        recordsLeft -= recordsRead;

        for(long long i = 0; i < recordsRead; i++)
        {
            binaryConvert(&data[i * mHeader.pointDataRecordLength], mPointData[totalPointsRead + i].X);
            binaryConvert(&data[i * mHeader.pointDataRecordLength + 4], mPointData[totalPointsRead + i].Y);
            binaryConvert(&data[i * mHeader.pointDataRecordLength + 8], mPointData[totalPointsRead + i].Z);
        }

        totalPointsRead += recordsRead;
    }

    delete[] data;

    end = std::chrono::high_resolution_clock::now();

    cout << "Done reading " << mNumPoints <<  " points in ";
    cout << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    cout << " milliseconds." << endl;
}

double** LASReader::makeHeightmapFromPointData(unsigned int& height, unsigned int& width)
{
    // Get the maximum x and y values a vertex in our dataset can have.
    long maxX = 0;
    long minX = numeric_limits<long>::max();

    long maxY = 0;
    long minY = numeric_limits<long>::max();

    long maxZ = 0;
    long minZ = numeric_limits<long>::max();

    for(unsigned long long i = 0; i < mNumPoints; i++)
    {
        if(mPointData[i].X > maxX)
            maxX = mPointData[i].X;
        if(mPointData[i].X < minX)
            minX = mPointData[i].X;
        if(mPointData[i].Y > maxY)
            maxY = mPointData[i].Y;
        if(mPointData[i].Y < minY)
            minY = mPointData[i].Y;
        if(mPointData[i].Z > maxZ)
            maxZ = mPointData[i].Z;
        if(mPointData[i].Z < minZ)
            minZ = mPointData[i].Z;
    }

    // This is where we store the sum of delta(x, y, z)
    // And where we store the number of delta(x, y, z) values
    long long int sumdx = 0;
    long long int numdx = 0;
    long long int sumdy = 0;
    long long int numdy = 0;
    long long int sumdz = 0;
    long long int numdz = 0;
    long d;
    Point* lastPoint = &mPointData[0];

    for(unsigned long long i = 1; i < mNumPoints; i++)
    {
        d = abs(mPointData[i].X - lastPoint->X);
        if(d != 0)
        {
            sumdx += d;
            numdx++;
        }
        d = abs(mPointData[i].Y - lastPoint->Y);
        if(d != 0)
        {
            sumdy += d;
            numdy++;
        }

        d = abs(mPointData[i].Z - lastPoint->Z);
        if(d != 0)
        {
            sumdz += d;
            numdz++;
        }

        lastPoint = &mPointData[i];
    }

    // These store the average of the distance between each point
    // so that we can determine the resolution of our hightmap
    long dx = static_cast<long>(sumdx / numdx);
    long dy = static_cast<long>(sumdy / numdy);
    long dz = static_cast<long>(sumdz / numdz);

    // This is the value range of the points
    double deltaX = maxX - minX;
    double deltaY = maxY - minY;
    double deltaZ = maxZ - minZ;

    // dx and dy can be different
    // we want the highest possible resolution
    // so we choose the smallest of them as dimensionsX
    int dimensionsX = 0;
    int dimensionsY = 0;
    if(dx < dy)
    {
        dimensionsX = static_cast<int>(deltaX / (2*dx));
        dimensionsY = static_cast<int>((deltaY / deltaX) * dimensionsX);
    }
    else if(dy <= dx)
    {
        dimensionsY = static_cast<int>(deltaY / (2*dy));
        dimensionsX = static_cast<int>((deltaX / deltaY) * dimensionsY);
    }

    int dimensionsZ = static_cast<int>(deltaZ / dz);

    double min = numeric_limits<double>::min();


    double** heightmap = new double*[dimensionsY];

    for(int i = 0; i < dimensionsY; i++)
    {
        heightmap[i] = new double[dimensionsX]{min};
    }
    // Map the point between 0 and dimensionX
    // (maxX - pX) / deltaX * dimensionX

    // Map the point between 0 and dimensionY
    // (maxY - pY) / deltaY * dimensionY

    // Map the data
    int p_x, p_y;
    double p_z;
    for(size_t i = 0; i < mNumPoints; i++)
    {
        p_x = static_cast<int>(dimensionsX*(mPointData[i].X - minX) / deltaX);
        p_y = static_cast<int>(dimensionsY*(mPointData[i].Y - minY) / deltaY);
        p_z = dimensionsZ * (mPointData[i].Z - minZ) / deltaZ;

        if(p_x >= dimensionsX)
            p_x = dimensionsX - 1;
        if(p_y >= dimensionsY)
            p_y = dimensionsY - 1;

        heightmap[p_y][p_x] = p_z;
    }


    // Find out where there is no data in a our heightmap
    // AKA heightmap[a][b] = numeric_limits<double>::min()
    // if it has no measurements in its cell
    int numxf, numxb, diff, i, j, a;
    double d_y, l;
    for(i = 0; i < dimensionsY; i++)
    {
        for(j = 0; j < dimensionsX; j++)
        {
            if(heightmap[i][j] <= min)
            {
                numxf = j;
                numxb = j;

                // Go forward untill you find a number
                while(heightmap[i][numxf] <= min &&
                      numxf < dimensionsX)
                    numxf++;
                // Go backward untill you find a number
                while(heightmap[i][numxb] <= min &&
                      numxb > 0)
                    numxb--;

                diff = numxf - numxb;
                d_y = heightmap[i][numxf] - heightmap[i][numxb];
                l = sqrt(diff*diff + d_y*d_y);
                d_y /= l > 0 ? l : d_y;

                for(a = 0; a < diff; a++)
                {
                    heightmap[i][numxb + a] = heightmap[i][numxb] + d_y * a;
                }
            }
        }
    }

    // Apply a gaussian filter on the heightmap to smooth it
    // ( get's wrid of those pesky sharp bumps )

    Matrix filter = getGaussian(5, 5, 60.0);

    heightmap = applyFilter(heightmap, filter, dimensionsX, dimensionsY);

    // Try to downscale to half it's size, helps smooth it further
    unsigned int dW = static_cast<unsigned int>(dimensionsX / 2);
    unsigned int dH = static_cast<unsigned int>(dimensionsY / 2);

    double** downscaled = new double*[dH];
    for(unsigned int i = 0; i < dH; i++)
        downscaled[i] = new double[dW];

    double t, r1, u1, r1u1;
    // Each pixel on downscaled needs to be the average of four pixels
    for(i = 0; i < dimensionsY-1; i += 2)
    {
        for(j = 0; j < dimensionsX-1; j += 2)
        {
            t = heightmap[i][j];
            r1 = heightmap[i][j+1];
            u1 = heightmap[i+1][j];

            r1u1 = heightmap[i+1][j+1];

            downscaled[i / 2][j / 2] = (t + r1 + u1 + r1u1) / 4.0;
        }
    }

    // No longer need the 2x scale heightmap
    for(int i = 0; i < dimensionsY; i++)
        delete[] heightmap[i];
    delete[] heightmap;
    heightmap = nullptr;

    height = dH;
    width = dW;

    return downscaled;
}

Matrix LASReader::getGaussian(int height, int width, double sigma)
{
    Matrix kernel(height, Array(width));
    double sum=0.0;
    int i,j;

    for (i=0 ; i<height ; i++) {
        for (j=0 ; j<width ; j++) {
            kernel[i][j] = exp(-(i*i+j*j)/(2*sigma*sigma))/(2*M_PI*sigma*sigma);
            sum += kernel[i][j];
        }
    }

    for (i=0 ; i<height ; i++) {
        for (j=0 ; j<width ; j++) {
            kernel[i][j] /= sum;
        }
    }

    return kernel;
}

double** LASReader::applyFilter(double** image, Matrix &filter, int& width, int& height)
{
    assert(filter.size()!=0);

    int filterHeight = filter.size();
    int filterWidth = filter[0].size();
    int newImageHeight = height-filterHeight+1;
    int newImageWidth = width-filterWidth+1;
    int i,j,h,w;

    double** newImage = new double*[newImageHeight];
    for(int i = 0; i < newImageHeight; i++)
        newImage[i] = new double[newImageWidth]{0};

    for (i=0 ; i<newImageHeight ; i++) {
        for (j=0 ; j<newImageWidth ; j++) {
            for (h=i ; h<i+filterHeight ; h++) {
                for (w=j ; w<j+filterWidth ; w++) {
                    newImage[i][j] += filter[h-i][w-j]*image[h][w];
                }
            }
        }
    }

    for(i = 0; i < height; i++)
        delete[] image[i];
    delete[] image;

    width = newImageWidth;
    height = newImageHeight;

    return newImage;
}

Point LASReader::getPoint(unsigned long long index)
{
    return mPointData[index];
}

Point *LASReader::point()
{
    return mPointData;
}
