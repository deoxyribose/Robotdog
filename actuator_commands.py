import numpy as np

#Function to calculate roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
def xyztoang(x, y, z, yoffh, hu, hl):
    dyz = np.sqrt(y**2 + z**2)
    lyz = np.sqrt(dyz**2 - yoffh**2)
    gamma_yz =  -np.arctan(y / z)
    gamma_h_offset =  -np.arctan(-yoffh / lyz)
    gamma = gamma_yz - gamma_h_offset
    
    lxzp = np.sqrt(lyz**2 + x**2)
    n = (lxzp**2 - hl**2 - hu**2) / (2*hu)
    beta =  -np.arccos(n / hl)
    
    alfa_xzp =  -np.arctan(x / lyz)
    alfa_off = np.arccos((hu + n) / lxzp)
    alfa = alfa_xzp + alfa_off
    if any( np.isnan([gamma,alfa,beta])):
        print([gamma,alfa,beta])
        print(x,y,z,yoffh,hu,hl)
        # pass
    return [gamma,alfa,beta]
