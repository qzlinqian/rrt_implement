#!/usr/bin/env python

from collision_rospy.srv import *
from collision_rospy.msg import ellipsoid
from collision_rospy.msg import point2d
import rospy
import numpy as np
import random as rn
import math

class ellipsoid_gen:
    def __init__(self, name_='ellipsoid_points_gen_server'):
        self.name = name_

    def handle_ellipsoid_points_gen(self,req):
        semi_axes = np.asarray(req.ellipsoid.semi_axes)
        a = semi_axes[0]
        b = semi_axes[1]
        #0 < n <= 2
        n = req.ellipsoid.epsilon
        #pi_step = 0.02
        pi_step = math.pi*req.steps
        t = np.arange(0.0,math.pi,pi_step)
        power = np.vectorize(math.pow)
        cos_x = power(np.cos(t), 2.0/n)
        xp = a * cos_x
        xn = (-1) * xp
        x = list(xp)
        x.extend(xn)
        sin_y = power(np.sin(t), 2.0/n)
        yp = b * sin_y
        yn = (-1) * yp
        y = list(yp)
        y.extend(yn)
        formula = list(zip(x,y))
        
        #rotated_x = []
        #rotated_y = []
        points = []
        for i in range(len(formula)):
          aux = formula[i]
          x_aux, y_aux = self.rotation(aux[0], aux[1], req.ellipsoid.angle)
          #rotated_x.append(x_aux)
          #rotated_y.append(y_aux)
          point_aux = point2d(x = x_aux + req.ellipsoid.center[0], y=y_aux + req.ellipsoid.center[1])
          points.append(point_aux)
          #Return points with rotation with translation           
        return ellipsoid_pointsResponse(points = points)

    def ellipsoid_gen_server(self):
        rospy.init_node(self.name+'_node')
        s = rospy.Service(self.name, ellipsoid_points, self.handle_ellipsoid_points_gen)
        print "Ready to generate ellipsoids."
        rospy.spin()

    def rotation(self, x_,y_, angle_):
      x = (x_ * math.cos(angle_)) + (y_ * math.sin(angle_))
      y = ((-1)*x_*math.sin(angle_)) + (y_ * math.cos(angle_))
      return x , y



if __name__ == "__main__":
    ellip_gen = ellipsoid_gen()
    ellip_gen.ellipsoid_gen_server()
