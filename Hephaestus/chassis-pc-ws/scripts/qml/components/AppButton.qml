import QtQuick
import QtQuick.Controls

Button {
    id: control
    property string variant: "secondary"   // primary | secondary | danger | ghost
    property bool compact: false

    implicitHeight: compact ? 34 : 42
    implicitWidth: Math.max(compact ? 88 : 112, contentItem.implicitWidth + 30)
    leftPadding: 15
    rightPadding: 15
    font.pixelSize: compact ? 13 : 14
    font.weight: Font.DemiBold

    contentItem: Text {
        text: control.text
        color: !control.enabled ? "#9AA7B3"
              : control.variant === "primary" ? "#FFFFFF"
              : control.variant === "danger" ? "#FFFFFF"
              : "#2C3A47"
        font: control.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }

    background: Rectangle {
        radius: 10
        border.width: control.variant === "ghost" ? 0 : 1
        border.color: !control.enabled ? "#E1E7ED"
                    : control.variant === "primary" ? "#0F7DB2"
                    : control.variant === "danger" ? "#C94A5A"
                    : control.hovered ? "#AFC1CF" : "#CAD5DF"
        color: !control.enabled ? "#F1F4F7"
             : control.variant === "primary" ? (control.down ? "#126F9F" : "#178BC3")
             : control.variant === "danger" ? (control.down ? "#A43644" : "#C44756")
             : control.variant === "ghost" ? (control.hovered ? "#EAF2F8" : "transparent")
             : control.down ? "#E6EEF4" : control.hovered ? "#F3F7FA" : "#FFFFFF"
        Behavior on color { ColorAnimation { duration: 120 } }
    }
}
