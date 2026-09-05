# Atlas PC Teleop App

该目录是 PC 端主从臂遥操作 GUI，不是命令行遥操作脚本，也不属于树莓派 ROS2 自主任务栈

## 从源码运行

Windows PowerShell：

```powershell
cd Atlas\chassis-pc-ws
py -m pip install -r requirements.txt
py scripts\main.py
```

Linux：

```bash
cd Atlas/chassis-pc-ws
python3 -m pip install -r requirements.txt
python3 scripts/main.py
```

App 启动后点击“扫描串口”，选择或填写“主臂串口”和“从臂 MCU 串口”，应用配置后点击“开始运行”；两个设备不能选择同一个串口；串口被其他程序占用时应先释放

## Windows 打包

先安装打包工具，再从 `scripts` 目录构建：

```powershell
cd Atlas\chassis-pc-ws
py -m pip install -r requirements.txt
py -m pip install pyinstaller
cd scripts
py -m PyInstaller `
    --noconfirm `
    --clean `
    --name AtlasTeleop `
    --windowed `
    --onedir `
    --icon ".\assets\app.ico" `
    --add-data ".\qml:qml" `
    --add-data ".\assets:assets" `
    ".\main.py"
```
