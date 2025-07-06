# SkyCats UAS4STEM Survey Mission
A computer vision algorithm that allows an aerial 'drone' vehicle to survey a field and identify ground targets for the [UAS4STEM Beginner Division competition](https://amablog.modelaircraft.org/uas4stem/). This repository is the collaborative space for SkyCats' project, a team localized at Neuqua Valley High School.
> Updated from Vick Ye's UAS4STEM [*"UA4STEM"*](https://github.com/Vick-Ye/UA4STEM) template image identification algorithm.

## Build
- **Holybro X500 V2** as our UAV base 
- **Raspberry Pi 5** for our dronekit utilization
  - Brushless gimbal equipped with a wide-lens camera for image recognition.
- 3D-printed parts for mounting

## Tests
Various test programs created to iteratively build upon our code to test functionality with the DroneKit Python Library.
- [x] <ins>video.py / stream.py</ins>: image recognition and feedback using OpenCV.
- [x] <ins>hoverTest.py</ins>: basic flight test to hover the drone and gain latency feedback.
- [ ] (WIP) <ins>movementTest.py</ins>: basic flight movement test.
- [ ] (WIP) <ins>imgRecognition.py</ins>: movement test including image recognition for obtaining POI coordinates.
- [ ] (WIP) <ins>searchMission.py</ins>: search flight to accurately determine GPS coordinates for field POIs from waypoint mission.
