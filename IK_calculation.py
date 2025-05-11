import numpy as np
from numpy import sin,cos

def calculate_cable_lengths(pose, container_size=60, a=7.5, b=7.5, c=9):
    """
    Calculate the cable lengths for a 6-DOF, 8-cable driven parallel robot.
    
    The cable lengths are computed using the provided symbolic formula.
    The platform pose is given as [x, y, z, alpha, beta, gamma], where:
        - x, y, z: Coordinates of the platform center.
        - alpha: Roll, beta: Pitch, gamma: Yaw (in radians, using the ZYX convention).
    
    Default values for container_size, a, b, c are 60, 7.5, 7.5, and 9 respectively.
    
    Returns:
        Q: An 1x8 array containing the cable lengths.
    """
    x = pose[0]
    y = pose[1]
    z = pose[2]
    alpha = pose[3]
    beta = pose[4]
    gamma = pose[5]
    # Cable 1
    Term1_x = container_size - x + (b*(np.cos(gamma)*np.sin(alpha) - np.cos(alpha)*np.sin(beta)*np.sin(gamma)))/2 \
                + (c*(np.sin(alpha)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)*np.sin(beta)))/2 \
              - (a*np.cos(alpha)*np.cos(beta))/2
    Term1_y = y - container_size + (b*(np.cos(alpha)*np.cos(gamma) + np.sin(alpha)*np.sin(beta)*np.sin(gamma)))/2 \
              + (c*(np.cos(alpha)*np.sin(gamma) - np.cos(gamma)*np.sin(alpha)*np.sin(beta)))/2 \
              + (a*np.cos(beta)*np.sin(alpha))/2
    Term1_z = container_size - z + (a*np.sin(beta))/2 + (c*np.cos(beta)*np.cos(gamma))/2 \
              - (b*np.cos(beta)*np.sin(gamma))/2
    Q1 = np.sqrt(Term1_x**2 + Term1_y**2 + Term1_z**2 - 3.6**2)
    
    # Cable 2
    Term2_x = x - container_size + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 \
              - (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 \
              + (a*cos(alpha)*cos(beta))/2
    Term2_y = container_size - z + (a*sin(beta))/2 \
              + (c*cos(beta)*cos(gamma))/2 \
              + (b*cos(beta)*sin(gamma))/2
    Term2_z = y - (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 \
              + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 \
              + (a*cos(beta)*sin(alpha))/2
    Q2 = np.sqrt(Term2_x**2 + Term2_y**2 + Term2_z**2 - 3.6**2)
    
    # Cable 3
    Term3_x = container_size - z - (a*sin(beta))/2 + (c*cos(beta)*cos(gamma))/2 + (b*cos(beta)*sin(gamma))/2
    Term3_y = x + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 \
        - (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 \
        - (a*cos(alpha)*cos(beta))/2
    Term3_z = y - (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 \
        + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 \
        - (a*cos(beta)*sin(alpha))/2
    Q3 = np.sqrt(Term3_x**2 + Term3_y**2 + Term3_z**2 - 3.6**2)
    
    # Cable 4
    Term4_x = y - container_size + (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 \
        + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 \
        - (a*cos(beta)*sin(alpha))/2
    Term4_y = z - container_size + (a*sin(beta))/2 \
        - (c*cos(beta)*cos(gamma))/2 \
        + (b*cos(beta)*sin(gamma))/2
    Term4_z = (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 \
        - x + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 \
        + (a*cos(alpha)*cos(beta))/2
    Q4 = np.sqrt(Term4_x**2 + Term4_y**2 + Term4_z**2 - 3.6**2)
    
    # Cable 5
    Term5_x = z - (a*sin(beta))/2 \
        + cos(beta)*sin(gamma)*(b/2 - 3/2) \
        + (c*cos(beta)*cos(gamma))/2
    Term5_y = x - container_size - (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) \
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 \
        + (a*cos(alpha)*cos(beta))/2
    Term5_z = y - container_size + (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) \
        - (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 \
        + (a*cos(beta)*sin(alpha))/2
    Q5 = np.sqrt(Term5_x**2 + Term5_y**2 + Term5_z**2 - 3.6**2)
    
    # Cable 6
    Term6_x = y - (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) \
        - (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 \
        + (a*cos(beta)*sin(alpha))/2
    Term6_y = z - (a*sin(beta))/2 \
        - cos(beta)*sin(gamma)*(b/2 - 3/2) \
        + (c*cos(beta)*cos(gamma))/2
    Term6_z = x - container_size + (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) \
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 \
        + (a*cos(alpha)*cos(beta))/2
    Q6 = np.sqrt(Term6_x**2 + Term6_y**2 + Term6_z**2 - 3.6**2)
    
    # Cable 7
    Term7_x = x + (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) \
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 \
        - (a*cos(alpha)*cos(beta))/2
    Term7_y = z + (a*sin(beta))/2 \
        - cos(beta)*sin(gamma)*(b/2 - 3/2) \
        + (c*cos(beta)*cos(gamma))/2
    Term7_z = (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) \
        - y + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 \
        + (a*cos(beta)*sin(alpha))/2
    Q7 = np.sqrt(Term7_x**2 + Term7_y**2 + Term7_z**2 - 3.6**2)
    
    # Cable 8
    Term8_x = x - (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) \
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 \
        - (a*cos(alpha)*cos(beta))/2
    Term8_y = z + (a*sin(beta))/2 + cos(beta)*sin(gamma)*(b/2 - 3/2) + (c*cos(beta)*cos(gamma))/2
    Term8_z = container_size - y - (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) \
        + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 \
        + (a*cos(beta)*sin(alpha))/2
    Q8 = np.sqrt(Term8_x**2 + Term8_y**2 + Term8_z**2 - 3.6**2)
    
    # Combine the eight cable lengths into an 1x8 array.
    Q = [Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8]
    Q = [float(val) for val in Q]
    return Q

