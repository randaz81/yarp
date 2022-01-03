/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_OS_SERIALIZER_H
#define YARP_OS_SERIALIZER_H

#include <yarp/os/Portable.h>
#include <yarp/os/Log.h>

namespace yarp::os {

/**
 * \ingroup comm_class
 *
 * A base template serializer. Methods read/write should be overrided by the specific data type to serialize.
 *
 */
template <typename T>
class Serializer : public yarp::os::Portable
{
public:
    Serializer() :
        mStorage(new T),
        mStorageConst(std::as_const(mStorage)),
        isOwned(true)
    {
    }

    explicit Serializer(T& in) :
        mStorage(&in),
        mStorageConst(std::as_const(mStorage))
    {
    }

    // This constructor takes a const T as input.
    // If the non-const get() method is used, it should probably throw an
    // exception.
    explicit Serializer(const T& in) :
        mStorageConst(&in)
    {
    }

    explicit Serializer(T&& in) noexcept :
        mStorage(new T(std::move(in))),
        mStorageConst(std::as_const(mStorage)),
        isOwned(true)
    {
    }

    Serializer(const Serializer& in) = delete;
    Serializer(Serializer&& in) noexcept= delete;
    Serializer& operator=(const Serializer& in) = delete;
    Serializer& operator=(Serializer&&) = delete;

    ~Serializer() override
    {
        if (isOwned) {
            delete mStorage;
        }
    }

    bool read(yarp::os::ConnectionReader& connection) override = 0;

    bool write(yarp::os::ConnectionWriter& connection) const override = 0;

    /* [[nodiscard]] */ T& get() { yAssert(mStorage); return *mStorage; }
    /* [[nodiscard]] */ const T& get() const { yAssert(mStorageConst); return *mStorageConst; }

    T* take() {
        if (!isOwned) {
            return nullptr;
        }

        mStorageConst = nullptr;
        isOwned = false;
        return std::exchange(mStorage, nullptr);
    }

protected:
    T* mStorage {nullptr};
    const T* mStorageConst {nullptr};

private:
    bool isOwned {false};
};

} // namespace yarp::os

#endif // YARP_OS_SERIALIZER_H
