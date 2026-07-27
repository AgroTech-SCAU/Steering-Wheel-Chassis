打包:

```powershell
python -m PyInstaller `
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