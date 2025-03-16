builtin_interfaces/Time stamp        # primary unix timestamp. It is the sensor time for the driver nodes, and the timestamp where multiple input messages are synchronized for the other nodes
builtin_interfaces/Time pub_stamp    # message publishing unix timestamp
uint32 index                         # publish index, starting from 0 on startup
String frame_id                      # present the coordinate system of the msg