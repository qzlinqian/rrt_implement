// Generated by gencpp from file rrt_implement/pointsArray.msg
// DO NOT EDIT!


#ifndef RRT_IMPLEMENT_MESSAGE_POINTSARRAY_H
#define RRT_IMPLEMENT_MESSAGE_POINTSARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <rrt_implement/point.h>

namespace rrt_implement
{
template <class ContainerAllocator>
struct pointsArray_
{
  typedef pointsArray_<ContainerAllocator> Type;

  pointsArray_()
    : points()
    , name()  {
    }
  pointsArray_(const ContainerAllocator& _alloc)
    : points(_alloc)
    , name(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::rrt_implement::point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rrt_implement::point_<ContainerAllocator> >::other >  _points_type;
  _points_type points;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;





  typedef boost::shared_ptr< ::rrt_implement::pointsArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rrt_implement::pointsArray_<ContainerAllocator> const> ConstPtr;

}; // struct pointsArray_

typedef ::rrt_implement::pointsArray_<std::allocator<void> > pointsArray;

typedef boost::shared_ptr< ::rrt_implement::pointsArray > pointsArrayPtr;
typedef boost::shared_ptr< ::rrt_implement::pointsArray const> pointsArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rrt_implement::pointsArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rrt_implement::pointsArray_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::rrt_implement::pointsArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rrt_implement::pointsArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rrt_implement::pointsArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rrt_implement::pointsArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rrt_implement::pointsArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rrt_implement::pointsArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rrt_implement::pointsArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "83c7d2c8f10e3061a55059bddbcbaf2a";
  }

  static const char* value(const ::rrt_implement::pointsArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x83c7d2c8f10e3061ULL;
  static const uint64_t static_value2 = 0xa55059bddbcbaf2aULL;
};

template<class ContainerAllocator>
struct DataType< ::rrt_implement::pointsArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rrt_implement/pointsArray";
  }

  static const char* value(const ::rrt_implement::pointsArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rrt_implement::pointsArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "point[] points\n\
string name\n\
================================================================================\n\
MSG: rrt_implement/point\n\
float64 x\n\
float64 y\n\
";
  }

  static const char* value(const ::rrt_implement::pointsArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rrt_implement::pointsArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pointsArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rrt_implement::pointsArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rrt_implement::pointsArray_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rrt_implement::point_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RRT_IMPLEMENT_MESSAGE_POINTSARRAY_H