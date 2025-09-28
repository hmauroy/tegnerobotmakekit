tegneRobot.startDrawing()
for (let index = 0; index < 30; index++) {
    tegneRobot.lowerPen()
    basic.pause(1000)
    tegneRobot.liftPen()
    basic.pause(1000)
}
tegneRobot.showOkIcon()
