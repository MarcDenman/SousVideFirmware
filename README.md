SousVideFirmware
================

Spark Core Sous Vide Machine

## To Do

- [ ] Remove SprintF ([https://community.spark.io/t/firmware-tips-and-tricks/3649/2](https://community.spark.io/t/firmware-tips-and-tricks/3649/2 "Details Here"))
- [ ] Remotely set Kp, Ki, Kd tuning parameters (Spark Function) 
- [ ] Start PID (Spark Function) (Convert TurnOff()? 1 = on, 0 = off) Refactor digitalWrite out of TurnOff
- [ ] Control Relay (Spark Variable (Bool)) (Directly Control Relay - Low, High)(Any point?)
- [ ] Accuracy (At least 1dp)
- [x] Set TargetTemp(SetPoint) - SetSetPoint(String Args) - Automatically turns on PID when set.
- [ ] Set Sous Vide to begin cooking at set-time.
- [ ] Log start time (Persistence)
- [ ] Post all other detail s(Persistence)
- [ ] Create a series of error codes etc. 1 = success, 2 = failure etc.
- [ ] ALL DigitalWrites to relay MUST go through RelayActive first.
