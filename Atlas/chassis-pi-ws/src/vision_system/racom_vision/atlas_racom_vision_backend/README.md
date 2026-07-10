# atlas_racom_vision_backend

该功能包把最新 `vison_topic` 检测结果转换为 Atlas 任务接口

## 节点

```text
racom_camera_target_service
  /vision_detect → /vision/detect_camera_target

sorting_rule_service
  /vision_detect → /vision/classify_sorting_rule
```

## 配置

```text
config/racom_camera_target.yaml
config/sorting_rule.yaml
```

`sorting_rule.yaml` 负责左右槽位与园区对应；类别别名和多样本识别

整车启动

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

模型路径不在本包配置；统一在 `autonomous_transport.yaml` 的 `system.paths.vision_model_path` 设置
