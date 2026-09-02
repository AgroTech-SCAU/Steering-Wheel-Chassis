# 一帧检测结果：带采集时间戳、逆时针排序后带角标签的检测列表
std_msgs/Header header
bool is_final_best   # false=实时帧；true=停止检测时重发的最佳帧（header仍为原始采集时间）
DetectionCenter[] detections
