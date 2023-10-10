from scipy.interpolate import CubicSpline
import math
import numpy as np

def path_spline(x_path, y_path):
    x_diff = np.diff(x_path)
    y_diff = np.diff(y_path)
    phi = np.unwrap(np.arctan2(y_diff, x_diff))
    phi_init = phi[0]
    phi = np.hstack(( phi_init, phi  ))
    arc = np.cumsum( np.sqrt( x_diff**2+y_diff**2 )   )
    arc_length = arc[-1]
    arc_vec = np.linspace(0, arc_length, np.shape(x_path)[0])
    cs_x_path = CubicSpline(arc_vec, x_path)
    cs_y_path = CubicSpline(arc_vec, y_path)
    
    cs_x_path_der=cs_x_path.derivative()
    cs_y_path_der=cs_y_path.derivative()
    cs_phi_path = CubicSpline(arc_vec, phi)
    return cs_x_path, cs_y_path, cs_phi_path, cs_x_path_der,cs_y_path_der,arc_length, arc_vec



def waypoint_generator(x_global_init, y_global_init, x_path_data, y_path_data, arc_vec, cs_x_path, cs_y_path,cs_x_path_der,cs_y_path_der, cs_phi_path, arc_length,dt,v):
    idx = np.argmin( np.sqrt((x_global_init-x_path_data)**2+(y_global_init-y_path_data)**2))
    arc_curr = arc_vec[idx]
    arc_pred = arc_curr + v*dt
    x_waypoints = cs_x_path(arc_pred)
    y_waypoints =  cs_y_path(arc_pred)
    x_waypoints_der = cs_x_path_der(arc_pred)
    y_waypoints_der =  cs_y_path_der(arc_pred)
    phi_Waypoints = cs_phi_path(arc_pred)
    return x_waypoints, y_waypoints,x_waypoints_der,y_waypoints_der, phi_Waypoints