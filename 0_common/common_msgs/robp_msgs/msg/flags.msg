Header header

int8 arm_mission   # 0 - No action, 1 - Pick, 2 - Place
int8 path_gen_mission   # -1 - Explore, 0 - No action, 1 - Goal, 2 - Origo

bool perception   # When true, object removed
bool path_control   # When true, we send value to the motors
bool explorer   # When true, explorer done

geometry_msgs/Pose target

string object_id
