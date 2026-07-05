import QtQuick
import QtQuick.Controls

Button {
    id: control
    property bool selected: false
    property string symbol: "•"

    implicitHeight: 46
    leftPadding: 14
    rightPadding: 12

    contentItem: Row {
        spacing: 12
        Text {
            width: 22
            text: control.symbol
            color: control.selected ? "#1687C0" : "#7A8997"
            font.pixelSize: 17
            font.weight: Font.DemiBold
            verticalAlignment: Text.AlignVCenter
            anchors.verticalCenter: parent.verticalCenter
        }
        Text {
            text: control.text
            color: control.selected ? "#116B99" : "#536273"
            font.pixelSize: 14
            font.weight: control.selected ? Font.DemiBold : Font.Normal
            anchors.verticalCenter: parent.verticalCenter
        }
    }

    background: Rectangle {
        radius: 10
        color: control.selected ? "#EAF6FC" : control.hovered ? "#F3F7FA" : "transparent"
        border.width: control.selected ? 1 : 0
        border.color: "#B7DDF0"
        Rectangle {
            visible: control.selected
            width: 3
            height: 22
            radius: 2
            color: "#1687C0"
            anchors.left: parent.left
            anchors.leftMargin: 5
            anchors.verticalCenter: parent.verticalCenter
        }
    }
}
