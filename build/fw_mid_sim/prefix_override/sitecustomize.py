import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/share/yuhesen/fw-mid-nav2-sim/install/fw_mid_sim'
