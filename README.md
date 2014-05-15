# LCSR REPLAY

This describes a set of tools to make recordings of collaborative task demonstrations with the Barrett WAM arms.
It assumes that you have access to LCSR\_Barrett, LCSR\_Collab, and all the other CIRL simulation packages.

In short:

1) record data into rosbag files
2) record features
3) adjust data and replay adjusted trajectories

## Features

Essentially, this package lets you add information to a bag file, and process that bag file in different ways to produce new trajectories.

### Current

- I currently label segments based on arm/hand commands. You might want to use this to look at what's going on at a given time.
- Record with _rrec_
- Replay with _b2play_
- Terrible registered replay with _registered\_b2play_

### Planned

- I am integrating a Catkinized version of the DMP package.
- I will add launch files to handle different LCSR\_Collab simulation tasks.
- I'll also add launch files to support things like DMP-based LFD.
- I also am planning a RENAMER tool, which lets you assign names to the different segments in a bag file.
- Also, I want Python versions of all this stuff.


