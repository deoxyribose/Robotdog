import numpy as np

#Function to calculate roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
def xyztoang(foot_x, foot_y, foot_z, hip_width, thigh_length, calf_length):
    """
    Calculate the roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
    """
    hip_to_foot_distance_along_y_z_plane = np.sqrt(foot_y**2 + foot_z**2)
    bent_leg_length = np.sqrt(hip_to_foot_distance_along_y_z_plane**2 - hip_width**2)
    
    gamma_yz =  -np.arctan(foot_y / foot_z)
    gamma_h_offset =  -np.arctan(-hip_width / bent_leg_length)
    gamma = gamma_yz - gamma_h_offset
    
    # print("hip_to_foot_distance_along_y_z_plane:", hip_to_foot_distance_along_y_z_plane)
    # print("bent_leg_length:", bent_leg_length)
    # print("gamma:", gamma)

    lxzp = np.sqrt(bent_leg_length**2 + foot_x**2)
    n = (lxzp**2 - calf_length**2 - thigh_length**2) / (2*thigh_length)
    beta =  -np.arccos(n / calf_length)
    # assert -1 <= n / calf_length <= 1, f"n / calf_length = {n / calf_length, n, calf_length}"

    alpha_xzp =  -np.arctan(foot_x / bent_leg_length)
    alpha_off = np.arccos((thigh_length + n) / lxzp)
    alpha = alpha_xzp + alpha_off
    if any( np.isnan([gamma,alpha,beta])):
        print([gamma,alpha,beta])
        print(foot_x, foot_y, foot_z, hip_width,thigh_length,calf_length)
        # pass
    return [gamma,alpha,beta]
