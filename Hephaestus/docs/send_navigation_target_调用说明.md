# send_navigation_target 调用说明

## 一，服务接口

```text
Service: /navigate_to_target
Type:    send_navigation_target/srv/NavigateToTarget
```

请求：

```yaml
waypoint_id: "P1"
```

响应：

```yaml
success: true
message: "成功到达 P1"
```

## 二，命令行测试

```bash
ros2 service call /navigate_to_target send_navigation_target/srv/NavigateToTarget "{waypoint_id: 'P1'}"
```

## 三，状态机使用方式

状态机只发送航点 ID，不直接传坐标，不修改 Nav2 内部

航点坐标仍由 `send_navigation_target` 内部表维护

