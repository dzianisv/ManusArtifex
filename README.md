# ManusArtifex — robohand

Control a Hiwonder LeArm 6-DOF arm (LSC-6 controller, USB HID) from a Mac.
The servos report nothing back. The camera is the only sensor.

## Start here

**Agents: read [AGENTS.md](AGENTS.md) first.** It says where the project state
lives and how to update it.

**People:** current state is in
[Notion → STATUS](https://app.notion.com/p/STATUS-3aeac25eb49f81b6923dea8de6b9fb73).
The [Robo Hand](https://app.notion.com/p/Robo-Hand-3abac25eb49f80ea93c6e29618f29456)
page is the index for goals, backlog, worklog and design.

```bash
pip install -e .
robohand info            # is the arm alive
robohand doctor          # device, camera, calibration, safety layers
robohand markers         # what the camera can see
```

Read [docs/safety.md](docs/safety.md) before commanding any motion. A servo was
already destroyed here by one bad command.

## Goals

- Pick up an object. Video proof. Main goal, not done.
- Then: mix cocktails (see the sibling `bender` project).

## Documentation

- [AGENTS.md](AGENTS.md) — how to use this project's Notion space. Start here.
- [docs/safety.md](docs/safety.md) — safety architecture, limits, procedures
- [docs/robot-brain.md](docs/robot-brain.md) — the design. What is AI, what is not, and why. Has a diagram and references.
- [docs/calibration.md](docs/calibration.md) — vision-assisted joint calibration
- [docs/camera-field-of-view.md](docs/camera-field-of-view.md) — camera aim; why a person in frame breaks pixel-differencing
- [docs/hardware-fault-servo5.md](docs/hardware-fault-servo5.md) — servo 5, intermittent, currently broken
- [docs/pick.md](docs/pick.md) — vision-guided pick, and why it servos instead of solving IK
- [docs/cocktail-robot.md](docs/cocktail-robot.md) — why no VLA can drive this arm
- [docs/research-hardware.md](docs/research-hardware.md) — what the controller can and cannot report

## The hardware

Robotic Arm Kit 6DOF, 5 servos — [Amazon](https://amzn.to/4jiTcto)

![](doc/img1.jpg)
![](doc/img2.jpg)
![](doc/img4.jpg)
![](doc/img5.jpg)
![](doc/img6.jpg)
![](doc/img7.jpg)

## Related

- https://github.com/ccourson/xArmServoController
- https://www.hiwonder.com.cn/store/learn/25.html — LSC-6 controller

Vendor docs: [Installation Manual](./doc/InstallationManual.pdf) ·
[User Manual](./doc/UserManual.pdf) ·
[Drive folder](https://drive.google.com/drive/folders/1B7h0Q0HydRYi348VFJnfbAmWay4W_1Sd)

