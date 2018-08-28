// Generated by gencpp from file rrt_implement/trajectory.msg
// DO NOT EDIT!


#ifndef RRT_IMPLEMENT_MESSAGE_TRAJECTORY_H
#define RRT_IMPLEMENT_MESSAGE_TRAJECTORY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <rrt_implement/position.h>
#include <rrt_implement/ellipsoid.h>

namespace rrt_implement
{
template <class ContainerAllocator>
struct trajectory_
{
  typedef trajectory_<ContainerAllocator> Type;

  trajectory_()
    : point3d()
    , model()  {
    }
  trajectory_(const ContainerAllocator& _alloc)
    : point3d(_alloc)
    , model(_alloc)  {
  (void)_alloc;
    }



   typedef  ::rrt_implement::position_<ContainerAllocator>  _point3d_type;
  _point3d_type point3d;

   typedef  ::rrt_implement::ellipsoid_<ContainerAllocator>  _model_type;
  _model_type model;





  typedef boost::shared_ptr< ::rrt_implement::trajectory_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rrt_implement::trajectory_<ContainerAllocator> const> ConstPtr;

}; // struct trajectory_

typedef ::rrt_implement::trajectory_<std::allocator<void> > trajectory;

typedef boost::shared_ptr< ::rrt_implement::trajectory > trajectoryPtr;
typedef boost::shared_ptr< ::rrt_implement::trajectory const> trajectoryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rrt_implement::trajectory_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rrt_implement::trajectory_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::rrt_implement::trajectory_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rrt_implement::trajectory_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rrt_implement::trajectory_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rrt_implement::trajectory_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rrt_implement::trajectory_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rrt_implement::trajectory_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rrt_implement::trajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ea0b13cc661285875c8beb2b9c2f22c6";
  }

  static const char* value(const ::rrt_implement::trajectory_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xea0b13cc66128587ULL;
  static const uint64_t static_value2 = 0x5c8beb2b9c2f22c6ULL;
};

template<class ContainerAllocator>
struct DataType< ::rrt_implement::trajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rrt_implement/trajectory";
  }

  static const char* value(const ::rrt_implement::trajectory_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rrt_implement::trajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "position point3d\n\
ellipsoid model\n\
================================================================================\n\
MSG: rrt_implement/position\n\
float64 x\n\
float64 y\n\
float64 phi\n\
\n\
================================================================================\n\
MSG: rrt_implement/ellipsoid\n\
float64[] semi_axes\n\
float64[] center\n\
float64 epsilon\n\
float64 angle\n\
";
  }

  static const char* value(const ::rrt_implement::trajectory_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rrt_implement::trajectory_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point3d);
      stream.next(m.model);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct trajectory_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rrt_implement::trajectory_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rrt_implement::trajectory_<ContainerAllocator>& v)
  {
    s << indent << "point3d: ";
    s << std::endl;
    Printer< ::rrt_implement::position_<ContainerAllocator> >::stream(s, indent + "  ", v.point3d);
    s << indent << "model: ";
    s << std::endl;
    Printer< ::rrt_implement::ellipsoid_<ContainerAllocator> >::stream(s, indent + "  ", v.model);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RRT_IMPLEMENT_MESSAGE_TRAJECTORY_H
