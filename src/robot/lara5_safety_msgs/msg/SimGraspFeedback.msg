# Simulation contact/ground-truth feedback, NOT a perception pose source.
std_msgs/Header header
string object_name
geometry_msgs/Pose object_pose
geometry_msgs/Pose object_in_flange
string[] contact_links
# Sum of positive normal forces (N), one entry per contact_links element.
# Physics observations only; not a calibrated hardware force measurement.
float64[] contact_normal_forces
bool opposing_contacts
bool supported
float64 linear_speed
