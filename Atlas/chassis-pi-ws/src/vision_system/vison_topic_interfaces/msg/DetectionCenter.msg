# 单个检测目标的中心点 + 按像素坐标分配的角标签
int32 cls_id
float64 u
float64 v
float64 conf
string cls_name
int32 corner_index   # 0=左上 1=右上 2=右下 3=左下, -1=无法归类
