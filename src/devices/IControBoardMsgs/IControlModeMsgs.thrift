/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

struct return_getControlMode_singlej
{
  1: bool      retvalue;
  2: i32       mode;
}

struct return_getControlModes_somej
{
  1: bool      retvalue;
  2: list<i32> mode;
}

struct return_getControlModes_allj
{
  1: bool      retvalue;
  2: list<i32> mode;
}

struct return_setControlMode_singlej
{
  1: bool   retvalue;
}

struct return_setControlModes_somej
{
  1: bool   retvalue;
}

struct return_setControlModes_allj
{
  1: bool   retvalue;
}

service IControlModeMsgsRPC
{
  return_getControlMode_singlej  getControlMode_singlej  (1: i16 j) ;
  return_getControlModes_somej   getControlModes_somej   (1: list<i16> jnts) ;
  return_getControlModes_allj    getControlModes_allj    () ;
  return_setControlMode_singlej  setControlMode_singlej  (1: i16 j, 2: i32 mode) ;
  return_setControlModes_somej   setControlModes_somej   (1: list<i16> jnts, 2: list<i32> modes) ;
  return_setControlModes_allj    setControlModes_allj    (1: list<i32> modes) ;
}
