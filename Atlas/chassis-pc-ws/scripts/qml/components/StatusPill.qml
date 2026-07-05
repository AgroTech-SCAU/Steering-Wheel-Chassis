import QtQuick

Rectangle {
    id: root
    property string text: ""
    property string tone: "neutral" // success | warning | danger | neutral

    implicitHeight: 30
    implicitWidth: label.implicitWidth + 28
    radius: 15
    color: tone === "success" ? "#E8F7EF"
         : tone === "warning" ? "#FFF5DD"
         : tone === "danger" ? "#FDEBED"
         : "#EEF2F6"
    border.width: 1
    border.color: tone === "success" ? "#A8D9BA"
                : tone === "warning" ? "#E6CB83"
                : tone === "danger" ? "#EAB3BB"
                : "#CBD5DF"

    Row {
        spacing: 7
        anchors.centerIn: parent
        Rectangle {
            width: 7
            height: 7
            radius: 4
            color: root.tone === "success" ? "#2FA66D"
                 : root.tone === "warning" ? "#D99A22"
                 : root.tone === "danger" ? "#D84F60"
                 : "#7B8996"
            anchors.verticalCenter: parent.verticalCenter
        }
        Text {
            id: label
            text: root.text
            color: root.tone === "success" ? "#1F6C44"
                 : root.tone === "warning" ? "#765514"
                 : root.tone === "danger" ? "#A33745"
                 : "#526271"
            font.pixelSize: 12
            font.weight: Font.DemiBold
        }
    }
}
