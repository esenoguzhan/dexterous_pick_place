# One object in a scene snapshot (docs/perception_architecture.md, sections 5.8 and 8.1).

uint8 SYM_CONTINUOUS=0
uint8 SYM_NONE=1

string id                          # unique within a snapshot, e.g. "obj_03"
string label                       # gallery name, or "unknown"
float32 label_score                # gallery similarity
geometry_msgs/Pose pose            # world frame; yaw about the table normal
geometry_msgs/Vector3 size         # length, width, height [m]
geometry_msgs/Point[] footprint    # polygon on the table plane, world frame
uint8 symmetry                     # SYM_CONTINUOUS, SYM_NONE, or k for k-fold
bool up_side_matters
string up_side                     # gallery view tag, empty if unknown
string support_id                  # "table" or another object's id
float32[] embedding                # DINOv2, L2-normalised
sensor_msgs/PointCloud2 cloud      # world frame, downsampled to 3 mm
uint8 cameras_seen                 # bitmask: 1 = camera_1, 2 = camera_2
bool depth_reliable
float32 confidence
