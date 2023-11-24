/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <yarp/dev/ReturnValue.h>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::dev;

TEST_CASE("dev::ReturnValue", "[yarp::dev]")
{
    SECTION("test block 1")
    {
        yarp_ret_value val_f1(false);
        yarp_ret_value val_f2(false);

        yarp_ret_value val_t1(true);
        yarp_ret_value val_t2(true);

        CHECK(val_f1 == val_f2);
        CHECK(val_t1 == val_t2);
        CHECK(val_f1 != val_t1);
        CHECK(val_t1);
        CHECK(!val_f1);

        std::string sf = val_f1.toString();
        std::string st = val_t1.toString();
    }

    SECTION("test block 2")
    {
        yarp_ret_value val1;
        CHECK(val1 == yarp_ret_value::return_code::return_value_unitialized);

        std::string s;
        val1 = yarp_ret_value::return_code::return_value_ok;
        s = val1.toString();
        CHECK(val1);
        CHECK(s!="unknown");

        val1 = yarp_ret_value::return_code::return_value_error_generic;
        s = val1.toString();
        CHECK(!val1);
        CHECK(s != "unknown");

        val1 = yarp_ret_value::return_code::return_value_error_method_failed;
        s = val1.toString();
        CHECK(!val1);
        CHECK(s != "unknown");

        val1 = yarp_ret_value::return_code::return_value_error_not_implemented_by_device;
        s = val1.toString();
        CHECK(!val1);
        CHECK(s != "unknown");

        val1 = yarp_ret_value::return_code::return_value_error_nws_nwc_communication_error;
        s = val1.toString();
        CHECK(!val1);
        CHECK(s != "unknown");

        val1 = yarp_ret_value::return_code::return_value_unitialized;
        s = val1.toString();
        CHECK(!val1);
        CHECK(s != "unknown");
    }

    SECTION("test block 3")
    {
        yarp_ret_value val1;
        val1 = yarp_ret_value::return_code::return_value_ok;
        yarp_ret_value val2(val1);
        CHECK(val2);
        CHECK(val2 == yarp_ret_value::return_code::return_value_ok);

        val1 = yarp_ret_value::return_code::return_value_error_method_failed;
        yarp_ret_value val3 = val1;
        CHECK(!val3);
        CHECK(val3 == yarp_ret_value::return_code::return_value_error_method_failed);
    }

    SECTION("test block 4")
    {
        yarp_ret_value val_f1(false);
        yarp_ret_value val_t1(true);
        bool bool_f1 = val_f1;
        bool bool_t1 = val_t1;
        CHECK (bool_f1 == false);
        CHECK (bool_t1 == true);

        yarp_ret_value val_f2(yarp_ret_value::return_code::return_value_error_method_failed);
        yarp_ret_value val_t2(yarp_ret_value::return_code::return_value_ok);
        bool bool_f2 = val_f2;
        bool bool_t2 = val_t2;
        CHECK(bool_f2 == false);
        CHECK(bool_t2 == true);
    }
}
