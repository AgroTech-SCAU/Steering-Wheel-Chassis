import QtQuick
import QtQuick.Layouts

Rectangle {
    id: root
    property string label: ""
    property string value: "0"
    property string unit: ""
    property color accent: "#1687C0"

    radius: 12
    color: "#F8FAFC"
    border.color: "#D9E2EC"
    implicitHeight: 82

    Rectangle {
        width: 3
        radius: 2
        color: root.accent
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.topMargin: 12
        anchors.bottomMargin: 12
    }

    Column {
        anchors.left: parent.left
        anchors.leftMargin: 16
        anchors.right: parent.right
        anchors.rightMargin: 12
        anchors.verticalCenter: parent.verticalCenter
        spacing: 5
        Text {
            text: root.label
            color: "#718092"
            font.pixelSize: 11
        }
        Row {
            spacing: 5
            Text {
                text: root.value
                color: "#182433"
                font.pixelSize: 22
                font.weight: Font.DemiBold
            }
            Text {
                text: root.unit
                color: "#7D8A96"
                font.pixelSize: 11
                anchors.baseline: parent.children[0].baseline
            }
        }
    }
}
