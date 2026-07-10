# qkrt-mcb

This repository contains the embedded programs running on the RoboMaster Type C Main Control Board (MCB). The project builds off of the [Taproot Template Project](https://gitlab.com/aruw/controls/taproot-template-project) developed by the University of Washington Advanced Robotics Team. All Taproot documentation can be found via the [Taproot Wiki](https://gitlab.com/aruw/controls/taproot/-/wikis/home).

## Getting Started
1. Follow the [MCB Programming Notion Page](https://app.notion.com/p/Main-Controller-Board-Programming-1211ea6d75cd81c2884eda20a9877951?source=copy_link) or [New User Guide](https://github.com/uw-advanced-robotics/taproot-template-project#new-user-guide) for setting up your development enviornment.
2. Clone the repository 
Run the following to clone this repository:
```bash
git clone --recursive https://github.com/qkrt-rm/qkrt-mcb.git
```
If you have already clones, to ensure you are using the right version of taproot run:
```bash
git submodule update --init --recursive
```
Now, cd into the project directory, activate the virtualenv, and run some builds:
```bash
cd qkrt-mcb/qkrt-mcb-project
pipenv shell
# Build for hardware
scons build
# Flash Code
scons run robot=TARGET_STANDARD   #TARGET_HERO, TARGET_SENTRY
```

4. Familiarize yourself with the build and flashing commands found in the [Building via Terminal](https://github.com/uw-advanced-robotics/taproot-template-project#building-and-running-via-the-terminal)
5. Read the [Taproot Command Subsystem Framework](https://gitlab.com/aruw/controls/taproot/-/wikis/Command-Subsystem-Framework)


Our work is mainly found in the ```text qkrt-mcb/qkrt-mcb-project-src``` directory. Review the subsystems and command code.

## Resources and Manuals
- [Taproot Wiki](https://gitlab.com/aruw/controls/taproot/-/wikis/home)
- [Software Resources Notion Page](https://app.notion.com/p/Software-Resources-27f1ea6d75cd8017b7dffa200f13e1b1)
- [Taproot Template Project Repo](https://github.com/uw-advanced-robotics/taproot-template-project)
- [RoboMaster Type C Board](https://www.robomaster.com/en-US/products/components/general/development-board-type-c#downloads)
- [FTDI USB-C Serial Converter](https://www.digikey.ca/en/products/detail/adafruit-industries-llc/4331/10446989?gclsrc=aw.ds&gad_source=1&gad_campaignid=17336435733&gclid=Cj0KCQjwsMLSBhD9ARIsAIpUTDonRJnxlEUuRG4xlYObJ7Lg5Ss0ET53Ct6WVG5c0M1f2NPqtHyyk2QaAsmaEALw_wcB)
