# All objects on the table at one capture (docs/perception_architecture.md, section 8.1).

std_msgs/Header header             # frame_id: world
string snapshot_id
float64[4] table_plane             # a, b, c, d with a*x + b*y + c*z + d = 0
ObjectState[] objects
