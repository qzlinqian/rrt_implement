// Generated by gencpp from file rrt_implement/ellipsoid.msg
// DO NOT EDIT!


#ifndef RRT_IMPLEMENT_MESSAGE_ELLIPSOID_H
#define RRT_IMPLEMENT_MESSAGE_ELLIPSOID_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rrt_implement
{
template <class ContainerAllocator>
struct ellipsoid_
{
  typedef ellipsoid_<ContainerAllocator> Type;

  ellipsoid_()
    : semi_axes()
    , center()
    , epsilon(0.0)
    , angle(0.0)  {
    }
  ellipsoid_(const ContainerAllocator& _alloc)
    : semi_axes(_alloc)
    , center(_alloc)
    , epsilon(0.0)
    , angle(0.0)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _semi_axes_type;
  _semi_axes_type semi_axes;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _center_type;
  _center_type center;

   typedef double _epsilon_type;
  _epsilon_type epsilon;

   typedef double _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::rrt_implement::ellipsoid_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rrt_implement::ellipsoid_<ContainerAllocator> const> ConstPtr;

}; // struct ellipsoid_

typedef ::rrt_implement::ellipsoid_<std::allocator<void> > ellipsoid;

typedef boost::shared_ptr< ::rrt_implement::ellipsoid > ellipsoidPtr;
typedef boost::shared_ptr< ::rrt_implement::ellipsoid const> ellipsoidConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rrt_implement::ellipsoid_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rrt_implement::ellipsoid_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rrt_implement

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'visualization_msgs': ['/opt/ros/kinetic/share/visualization_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'rrt_implement': ['/home/lesley/ros-practice/summer/src/rrt_implement/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rrt_implement::ellipsoid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rrt_implement::ellipsoid_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rrt_implement::ellipsoid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rrt_implement::ellipsoid_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rrt_implement::ellipsoid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rrt_implement::ellipsoid_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rrt_implement::ellipsoid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "430a0a15ec989ee778cfad0bf53a39f3";
  }

  static const char* value(const ::rrt_implement::ellipsoid_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x430a0a15ec989ee7ULL;
  static const uint64_t static_value2 = 0x78cfad0bf53a39f3ULL;
};

template<class ContainerAllocator>
struct DataType< ::rrt_implement::ellipsoid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rrt_implement/ellipsoid";
  }

  static const char* value(const ::rrt_implement::ellipsoid_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rrt_implement::ellipsoid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] semi_axes\n\
float64[] center\n\
float64 epsilon\n\
float64 angle\n\
";
  }

  static const char* value(const ::rrt_implement::ellipsoid_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rrt_implement::ellipsoid_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.semi_axes);
      stream.next(m.center);
      stream.next(m.epsilon);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ellipsoid_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rrt_implement::ellipsoid_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rrt_implement::ellipsoid_<ContainerAllocator>& v)
  {
    s << indent << "semi_axes[]" << std::endl;
    for (size_t i = 0; i < v.semi_axes.size(); ++i)
    {
      s << indent << "  semi_axes[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.semi_axes[i]);
    }
    s << indent << "center[]" << std::endl;
    for (size_t i = 0; i < v.center.size(); ++i)
    {
      s << indent << "  center[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.center[i]);
    }
    s << indent << "epsilon: ";
    Printer<double>::stream(s, indent + "  ", v.epsilon);
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RRT_IMPLEMENT_MESSAGE_ELLIPSOID_H