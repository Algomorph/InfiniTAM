# InfiniTAM-Based Experiment With Dynamic-volume Reconstruction

[![Build status](https://ci.appveyor.com/api/projects/status/v5thuymftuu7i750/branch/master?svg=true)](https://ci.appveyor.com/project/Algomorph/infinitam/branch/master)

This is an ongoing project that aims to integrate voxel hashing techniques for real-time reconstruction of dynamic volume from a single, moving RGB-D camera. The main algorithmic insipirations are KillingFusion and SobolevFusion from the dynamic volume reconstruction perspective [see Mira Slavcheva's page at TUM](http://campar.in.tum.de/Main/MiraSlavcheva) , and work of [Matthias Neissner's group](https://niessnerlab.org/publications.html) on spatial voxel hashing in the context of voxel-based 3D reconstruction. The original code is forked from [InfiniTAM, Active Vision Group at Oxford University](http://www.robots.ox.ac.uk/~victor/infinitam/), but many parts have been massively modernized -- essentially, rewritten -- and extended with my own code (hence you'll find many files with a less-restrictive Apache V2 license insted of the original Oxford University Innovation Academic License.) Having said that, I haven't tested the older static-volume algorithms in awhile, so try those at your own risk at this point.

## What details are currently missing that I know of, and what are some known issues?

1. SDF-2-SDF rigid alignment is not yet implemented (InfiniTAM's camera trackers are used instead)
2. Capability to run the optimization in-reverse, in order to forward-animate the more-complete canonical mesh, is currently missing.
3. For the voxel-block-hash (super-fast) indexing, after about 70 frames of the Snoopy sequence, there aren't enough hash blocks any more as allowed by the current settings, and the program crashes. I'm currently investigating how to clean up the hash blocks after every N frames to combat this. For details and progress, check [issue #203](https://github.com/Algomorph/InfiniTAM/issues/203) and all the tasks in the project board referencing it.
4. For a full list of issues & tasks, see the [Dynamic-Scene Project Board](https://github.com/Algomorph/InfiniTAM/projects/1) and the [Maintenance/Reengineering Project Board]( https://github.com/Algomorph/InfiniTAM/projects/2). There are both algorithmic and software engineering issues (the code I've started with is far from perfect).

## How do I try this code out?

1. You need to be somewhat well-versed in using CMake. 3rd-party requirements are all open-source, and you can glean what you're missing by running the CMake generator. The one mandatory requirement that I've added is Boost, which should be [compiled with zlib](https://stackoverflow.com/questions/23107703/compiling-boost-with-zlib-on-windows) if you're building on windows.
2. Linux currently is the only officially supported OS, but I've recently fixed & tested the Windows build. All of the required CMake packages and this code in theory should work on any major platform, so you can try on MacOS at your own risk. Let me know if you'd like to fix things that are not working on your platform.
3. I recommend building with FFMPEG, since that will enable visual debugging/video recording.
4. To get some test data and try the code on it, download the [original Snoopy sequence](http://campar.in.tum.de/personal/slavcheva/deformable-dataset/index.html), modify Files/infinitam_snoopy_config.json with proper paths for input_and_output_settings_paths, and run like this (modify the path to point to infinitam_snoopy_config.json):

<build_folder>/Apps/InfiniTAM/InfiniTAM --config=Files/infinitam_snoopy_config.json

**Note**: If you build with FFMPEG, this will also record a video in the output folder specified in the config file.

## Is this code being worked on / maintained?

Yes, after a looong break, I'm officially switching to try to do something with it again, at least for awhile. Even if I'm not actively working on it, I do my best to respond to new issues or collaboration requests.

## Will I merge this back into InfiniTAM?

TLDR: Maybe.

Originally, that was the plan. However, at the time of writing I'm exhausted from trying to stick to the open(for extension)-closed(for modification) principle. The code wasn't originally designed for dynamic-volume fusion, so some things really do need to change to make the new things more maintainable. I've already changed a lot of code from the original InfiniTAM codebase, IMHO, for the better.

If this fork achieves reasonable success, I'll reach out to the InfiniTAM authors and ask whether they'd like to work on integrating my changes into their codebase. I (hope that I) didn't break anything so far, but the original code lacks any hint of continuous integration, which I plan to add to it, at least for my code, shortly.

Many portions of this code are still privy to the original Oxford University Innovation Academic License included within, consult file headers for which license is used for which part of the code.

Original InfiniTAM Repo & README: https://github.com/victorprad/InfiniTAM
