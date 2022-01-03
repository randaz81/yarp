/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_SIG_MATRIX_SERIALIZER_H
#define YARP_SIG_MATRIX_SERIALIZER_H

#include <yarp/os/Serializer.h>
#include <yarp/sig/Matrix.h>

 /**
 * \file MatrixSerializer.h contains the definition of a MatrixSerializer type
 */

 /**
 * \ingroup sig_class
 *
 * Matrix Serializer.
 */
namespace yarp::sig {

class YARP_sig_API MatrixSerializer : public yarp::os::Serializer<yarp::sig::Matrix>
{
public:
    using Serializer::Serializer;

    /*
     * Read matrix from a connection.
     * return true iff a matrix was read correctly
     */
    bool read(yarp::os::ConnectionReader& connection) override;

    /**
     * Write matrix to a connection.
     * return true iff a matrix was written correctly
     */
    bool write(yarp::os::ConnectionWriter& connection) const override;
};

}

#endif // YARP_SIG_MATRIX_SERIALIZER_H
