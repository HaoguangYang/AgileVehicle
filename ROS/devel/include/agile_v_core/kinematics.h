// Generated by gencpp from file agile_v_core/kinematics.msg
// DO NOT EDIT!


#ifndef AGILE_V_CORE_MESSAGE_KINEMATICS_H
#define AGILE_V_CORE_MESSAGE_KINEMATICS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace agile_v_core
{
template <class ContainerAllocator>
struct kinematics_
{
  typedef kinematics_<ContainerAllocator> Type;

  kinematics_()
    : CM_Velocity()
    , CM_Acceleration()
    , Vehicle_Heading(0.0)
    , CM_AngularVel(0.0)
    , CM_AngularAcc(0.0)
    , Wheel_LinearVel()
    , Wheel_SteerAngl()
    , Time_Stamp()  {
      CM_Velocity.assign(0.0);

      CM_Acceleration.assign(0.0);

      Wheel_LinearVel.assign(0.0);

      Wheel_SteerAngl.assign(0.0);
  }
  kinematics_(const ContainerAllocator& _alloc)
    : CM_Velocity()
    , CM_Acceleration()
    , Vehicle_Heading(0.0)
    , CM_AngularVel(0.0)
    , CM_AngularAcc(0.0)
    , Wheel_LinearVel()
    , Wheel_SteerAngl()
    , Time_Stamp()  {
  (void)_alloc;
      CM_Velocity.assign(0.0);

      CM_Acceleration.assign(0.0);

      Wheel_LinearVel.assign(0.0);

      Wheel_SteerAngl.assign(0.0);
  }



   typedef boost::array<double, 2>  _CM_Velocity_type;
  _CM_Velocity_type CM_Velocity;

   typedef boost::array<double, 2>  _CM_Acceleration_type;
  _CM_Acceleration_type CM_Acceleration;

   typedef double _Vehicle_Heading_type;
  _Vehicle_Heading_type Vehicle_Heading;

   typedef double _CM_AngularVel_type;
  _CM_AngularVel_type CM_AngularVel;

   typedef double _CM_AngularAcc_type;
  _CM_AngularAcc_type CM_AngularAcc;

   typedef boost::array<double, 4>  _Wheel_LinearVel_type;
  _Wheel_LinearVel_type Wheel_LinearVel;

   typedef boost::array<double, 4>  _Wheel_SteerAngl_type;
  _Wheel_SteerAngl_type Wheel_SteerAngl;

   typedef ros::Time _Time_Stamp_type;
  _Time_Stamp_type Time_Stamp;




  typedef boost::shared_ptr< ::agile_v_core::kinematics_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::agile_v_core::kinematics_<ContainerAllocator> const> ConstPtr;

}; // struct kinematics_

typedef ::agile_v_core::kinematics_<std::allocator<void> > kinematics;

typedef boost::shared_ptr< ::agile_v_core::kinematics > kinematicsPtr;
typedef boost::shared_ptr< ::agile_v_core::kinematics const> kinematicsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::agile_v_core::kinematics_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::agile_v_core::kinematics_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace agile_v_core

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'agile_v_core': ['/mnt/Data/AgileV_dev/ROS/src/agile_v_core/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::agile_v_core::kinematics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::agile_v_core::kinematics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::agile_v_core::kinematics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::agile_v_core::kinematics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agile_v_core::kinematics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agile_v_core::kinematics_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::agile_v_core::kinematics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "106ad333403d5f1ecfca19cf013ec05f";
  }

  static const char* value(const ::agile_v_core::kinematics_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x106ad333403d5f1eULL;
  static const uint64_t static_value2 = 0xcfca19cf013ec05fULL;
};

template<class ContainerAllocator>
struct DataType< ::agile_v_core::kinematics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "agile_v_core/kinematics";
  }

  static const char* value(const ::agile_v_core::kinematics_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::agile_v_core::kinematics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Vehicle Center Kinematics on Vehicle Coordinates, MKS Units Mandatory.\n\
float64[2]   CM_Velocity\n\
float64[2]   CM_Acceleration\n\
float64     Vehicle_Heading\n\
float64     CM_AngularVel\n\
float64     CM_AngularAcc\n\
\n\
# Wheel Kinematics\n\
float64[4]   Wheel_LinearVel\n\
float64[4]   Wheel_SteerAngl\n\
\n\
# Time Stamp\n\
time        Time_Stamp\n\
";
  }

  static const char* value(const ::agile_v_core::kinematics_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::agile_v_core::kinematics_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.CM_Velocity);
      stream.next(m.CM_Acceleration);
      stream.next(m.Vehicle_Heading);
      stream.next(m.CM_AngularVel);
      stream.next(m.CM_AngularAcc);
      stream.next(m.Wheel_LinearVel);
      stream.next(m.Wheel_SteerAngl);
      stream.next(m.Time_Stamp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct kinematics_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::agile_v_core::kinematics_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::agile_v_core::kinematics_<ContainerAllocator>& v)
  {
    s << indent << "CM_Velocity[]" << std::endl;
    for (size_t i = 0; i < v.CM_Velocity.size(); ++i)
    {
      s << indent << "  CM_Velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.CM_Velocity[i]);
    }
    s << indent << "CM_Acceleration[]" << std::endl;
    for (size_t i = 0; i < v.CM_Acceleration.size(); ++i)
    {
      s << indent << "  CM_Acceleration[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.CM_Acceleration[i]);
    }
    s << indent << "Vehicle_Heading: ";
    Printer<double>::stream(s, indent + "  ", v.Vehicle_Heading);
    s << indent << "CM_AngularVel: ";
    Printer<double>::stream(s, indent + "  ", v.CM_AngularVel);
    s << indent << "CM_AngularAcc: ";
    Printer<double>::stream(s, indent + "  ", v.CM_AngularAcc);
    s << indent << "Wheel_LinearVel[]" << std::endl;
    for (size_t i = 0; i < v.Wheel_LinearVel.size(); ++i)
    {
      s << indent << "  Wheel_LinearVel[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Wheel_LinearVel[i]);
    }
    s << indent << "Wheel_SteerAngl[]" << std::endl;
    for (size_t i = 0; i < v.Wheel_SteerAngl.size(); ++i)
    {
      s << indent << "  Wheel_SteerAngl[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Wheel_SteerAngl[i]);
    }
    s << indent << "Time_Stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.Time_Stamp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AGILE_V_CORE_MESSAGE_KINEMATICS_H
