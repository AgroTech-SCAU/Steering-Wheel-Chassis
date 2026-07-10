# vision_system

当前目录基于最新 `vision_system.zip`；用于智能分拣标识识别；待派送区目标检测和机械臂视觉动作

## 组成

```text
rui_vison/vison_topic
  最新 ONNX 模型和 /vision_detect 服务

racom_vision/atlas_racom_vision_backend
  像素目标适配和智能分拣园区映射

atlas_vision_pollination_backend
  手眼变换；机械臂动作序列；吸盘控制和运输动作

handeye_calibration_tool
  手眼标定工具
```

## 模型配置

统一 YAML

```yaml
system:
  paths:
    vision_model_path: ""
    vision_labels_path: ""
```

留空使用

```text
rui_vison/vison_topic/resource/best.onnx
```

外部路径不存在时视觉节点明确退出；不会静默回退到旧模型

详细配置见 `chassis-pi-ws/docs/视觉模型配置与标定.md`
