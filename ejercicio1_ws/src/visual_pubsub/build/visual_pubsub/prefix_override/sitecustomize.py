import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ingkurby/Escritorio/robotica2_ws/src/visual_pubsub/install/visual_pubsub'
