// Generated by gencpp from file mavros_msgs/CommandTriggerInterval.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_COMMANDTRIGGERINTERVAL_H
#define MAVROS_MSGS_MESSAGE_COMMANDTRIGGERINTERVAL_H

#include <ros/service_traits.h>


#include <mavros_msgs/CommandTriggerIntervalRequest.h>
#include <mavros_msgs/CommandTriggerIntervalResponse.h>


namespace mavros_msgs
{

struct CommandTriggerInterval
{

typedef CommandTriggerIntervalRequest Request;
typedef CommandTriggerIntervalResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CommandTriggerInterval
} // namespace mavros_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mavros_msgs::CommandTriggerInterval > {
  static const char* value()
  {
    return "b16f28a04389d5d47ddaa9e025e7383a";
  }

  static const char* value(const ::mavros_msgs::CommandTriggerInterval&) { return value(); }
};

template<>
struct DataType< ::mavros_msgs::CommandTriggerInterval > {
  static const char* value()
  {
    return "mavros_msgs/CommandTriggerInterval";
  }

  static const char* value(const ::mavros_msgs::CommandTriggerInterval&) { return value(); }
};


// service_traits::MD5Sum< ::mavros_msgs::CommandTriggerIntervalRequest> should match
// service_traits::MD5Sum< ::mavros_msgs::CommandTriggerInterval >
template<>
struct MD5Sum< ::mavros_msgs::CommandTriggerIntervalRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::CommandTriggerInterval >::value();
  }
  static const char* value(const ::mavros_msgs::CommandTriggerIntervalRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::CommandTriggerIntervalRequest> should match
// service_traits::DataType< ::mavros_msgs::CommandTriggerInterval >
template<>
struct DataType< ::mavros_msgs::CommandTriggerIntervalRequest>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::CommandTriggerInterval >::value();
  }
  static const char* value(const ::mavros_msgs::CommandTriggerIntervalRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mavros_msgs::CommandTriggerIntervalResponse> should match
// service_traits::MD5Sum< ::mavros_msgs::CommandTriggerInterval >
template<>
struct MD5Sum< ::mavros_msgs::CommandTriggerIntervalResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::CommandTriggerInterval >::value();
  }
  static const char* value(const ::mavros_msgs::CommandTriggerIntervalResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::CommandTriggerIntervalResponse> should match
// service_traits::DataType< ::mavros_msgs::CommandTriggerInterval >
template<>
struct DataType< ::mavros_msgs::CommandTriggerIntervalResponse>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::CommandTriggerInterval >::value();
  }
  static const char* value(const ::mavros_msgs::CommandTriggerIntervalResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_COMMANDTRIGGERINTERVAL_H
