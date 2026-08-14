import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/minh/git_gim_ws/GIM_Arm_3_DOF/install/gim_control'
