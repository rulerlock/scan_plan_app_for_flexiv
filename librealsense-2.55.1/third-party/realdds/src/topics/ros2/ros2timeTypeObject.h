// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2023 Intel Corporation. All Rights Reserved.

/*!
 * @file TimeTypeObject.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_STD_MSGS_MSG_TIME_TYPE_OBJECT_H_
#define _FAST_DDS_GENERATED_STD_MSGS_MSG_TIME_TYPE_OBJECT_H_


#include <fastrtps/types/TypeObject.h>
#include <map>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(Time_SOURCE)
#define Time_DllAPI __declspec( dllexport )
#else
#define Time_DllAPI __declspec( dllimport )
#endif // Time_SOURCE
#else
#define Time_DllAPI
#endif
#else
#define Time_DllAPI
#endif // _WIN32

using namespace eprosima::fastrtps::types;

eProsima_user_DllExport void registerTimeTypes();

namespace std_msgs {
    namespace msg {
        eProsima_user_DllExport const TypeIdentifier* GetTimeIdentifier(bool complete = false);
        eProsima_user_DllExport const TypeObject* GetTimeObject(bool complete = false);
        eProsima_user_DllExport const TypeObject* GetMinimalTimeObject();
        eProsima_user_DllExport const TypeObject* GetCompleteTimeObject();

    } // namespace msg
} // namespace std_msgs

#endif // _FAST_DDS_GENERATED_STD_MSGS_MSG_TIME_TYPE_OBJECT_H_