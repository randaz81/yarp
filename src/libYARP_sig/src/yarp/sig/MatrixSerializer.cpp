/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/sig/MatrixSerializer.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>

using namespace yarp::sig;
using namespace yarp::os;

#define RES(v) ((std::vector<T> *)v)

// network stuff
#include <yarp/os/NetInt32.h>

YARP_BEGIN_PACK
class MatrixPortContentHeader
{
public:
    yarp::os::NetInt32 outerListTag{ 0 };
    yarp::os::NetInt32 outerListLen{ 0 };
    yarp::os::NetInt32 rowsTag{ 0 };
    yarp::os::NetInt32 rows{ 0 };
    yarp::os::NetInt32 colsTag{ 0 };
    yarp::os::NetInt32 cols{ 0 };
    yarp::os::NetInt32 listTag{ 0 };
    yarp::os::NetInt32 listLen{ 0 };

    MatrixPortContentHeader() = default;
};
YARP_END_PACK

bool MatrixSerializer::read(yarp::os::ConnectionReader& connection) {
    // auto-convert text mode interaction
    connection.convertTextMode();
    MatrixPortContentHeader header;
    bool ok = connection.expectBlock((char*)&header, sizeof(header));
    if (!ok) {
        return false;
    }
    size_t r = mStorage->rows();
    size_t c = mStorage->cols();
    if (header.listLen > 0)
    {
        if (r != (size_t)(header.rows) || c != (size_t)(header.cols))
        {
            mStorage->resize(header.rows, header.cols);
        }

        int l = 0;
        double* tmp = mStorage->data();
        for (l = 0; l < header.listLen; l++) {
            tmp[l] = connection.expectFloat64();
        }
    }
    else {
        return false;
    }

    return true;
}


bool MatrixSerializer::write(yarp::os::ConnectionWriter& connection) const {
    MatrixPortContentHeader header;

    //header.totalLen = sizeof(header)+sizeof(double)*this->size();
    header.outerListTag = BOTTLE_TAG_LIST;
    header.outerListLen = 3;
    header.rowsTag = BOTTLE_TAG_INT32;
    header.colsTag = BOTTLE_TAG_INT32;
    header.listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_FLOAT64;
    header.rows = mStorageConst->rows();
    header.cols = mStorageConst->cols();
    header.listLen = header.rows * header.cols;

    connection.appendBlock((char*)&header, sizeof(header));

    int l = 0;
    const double* tmp = mStorageConst->data();
    for (l = 0; l < header.listLen; l++) {
        connection.appendFloat64(tmp[l]);
    }

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();

    return true;
}
