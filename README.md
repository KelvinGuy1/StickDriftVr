# StickDriftVr
A Free Open Source Steam VR Driver for NS Joycon support

<img src="img/vrxenia.svg" alt="StickDriftVr Logo" width="200" />

<sup>Xenia logo by QuestForTori - Modified by Mentalbox<sup/>

**StickDriftVr** is an **open-source Nintendo Switch Joy-Con driver for SteamVR**, created to give users a **free** (both in cost and use) way to use Joy-Con controllers in virtual reality.

This project embraces openness with no paywalls, no proprietary apps, and no restrictions on how you use or modify it.

##### ⚠️ Warning: Early Development Status - *This project is in a **very early and experimental state**. Calibration is currently **broken**, and you should expect **significant bugs, tracking issues, and drift** during use.  Nothing is guaranteed to work reliably yet. If you're concerned something happened to your joycons, attach them to your switch, recalibrate, and re-add them with your pc.*

## 🟥🟦 Features (Work In Progress)

- Full Nintendo Switch Joy-Con Support (Left, Right, paired mode) - **WIP**
- SteamVR-Compatible Motion Tracking using built-in IMU sensors (Gyroscope + Accelerometer Fusion) - **WIP**
- Low-Latency Input Handling
- Custom Button Mappings and fully configurable profiles
- Cross-Platform support - **WIP**
- Haptic Feedback Support - **Not started**

## 🛠️ Build

#### Linux
It's reccomended you have the [joycond](https://github.com/DanielOgorchock/joycond "joycond") daemon installed to pair the controlers together as one.

```bash
git clone https://github.com/KelvinGuy1/StickDriftVr/
cd StickDriftVr
sh build.sh
```
#### Windows
(coming soon)

## 🤝 Contribute
StickDriftVr is currently being developed by one guy. I am not taking donations at this time so for now, any help is welcome through the following:
- Pull requests
- Issue reports
- Documentation contributions
- Platform support testing
- Your contributions help keep this software free for everyone, forever.

## 📝 TODO
- Better integrate libjoycon into the repo and phase cloning out from CMakeLists.txt
- Fork internal patches of libjoycon to official repo
- Add Windows build support
- Create a Windows buildfile
- Reduce IMU drift
- Add reset/recenter controls
- Fix Joy-Con calibration retrieval on mount
- Phase out AI code, unused functions
- Fix commenting issues
