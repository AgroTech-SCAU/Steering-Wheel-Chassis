import QtQuick
import QtQuick.Layouts

Rectangle {
    id: card
    property string title: ""
    property string subtitle: ""
    default property alias content: contentColumn.data

    color: "#FFFFFF"
    border.color: "#D9E2EC"
    border.width: 1
    radius: 14
    implicitHeight: outer.implicitHeight + 28

    ColumnLayout {
        id: outer
        anchors.fill: parent
        anchors.margins: 16
        spacing: 14

        ColumnLayout {
            spacing: 3
            Layout.fillWidth: true
            Text {
                text: card.title
                color: "#182433"
                font.pixelSize: 15
                font.weight: Font.DemiBold
                Layout.fillWidth: true
            }
            Text {
                visible: card.subtitle.length > 0
                text: card.subtitle
                color: "#718092"
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
        }

        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#E7EDF3"
        }

        ColumnLayout {
            id: contentColumn
            Layout.fillWidth: true
            spacing: 12
        }
    }
}
