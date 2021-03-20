# InfiniTAM-Based Experiment With Dynamic-volume Reconstruction

[![Build status](https://ci.appveyor.com/api/projects/status/v5thuymftuu7i750/branch/master?svg=true)](https://ci.appveyor.com/project/Algomorph/infinitam/branch/master)

This is an ongoing project that aims to integrate voxel hashing techniques for real-time reconstruction of dynamic scene from a single, moving RGB-D camera. The main algorithmic insipirations are KillingFusion and SobolevFusion from the dynamic scene reconstruction perspective [see Mira Slavcheva's page at TUM](http://campar.in.tum.de/Main/MiraSlavcheva) , and work of [Matthias Neissner's group](https://niessnerlab.org/publications.html) on spatial voxel hashing in the context of voxel-based 3D reconstruction. The original code is forked from [InfiniTAM, Active Vision Group at Oxford University](http://www.robots.ox.ac.uk/~victor/infinitam/), but many parts have been massively modernized -- essentially, rewritten -- and extended with my own code (hence you'll find many files with a less-restrictive Apache V2 license instead of the original Oxford University Innovation Academic License.) Having said that, I haven't tested the older static-scene algorithms in awhile, so try those at your own risk at this point.

## What details are currently missing that I know of, and what are some known issues?

1. SDF-2-SDF rigid alignment is not yet implemented (InfiniTAM's camera trackers are used instead)
2. Capability to run the optimization in-reverse, in order to forward-animate the more-complete canonical mesh, is currently missing.
3. Runtime performance is not sufficiently optimized. The code still achieves ~4 fps on a GTX 1080 GPU on the [original Snoopy sequence](http://campar.in.tum.de/personal/slavcheva/deformable-dataset/index.html) (see below for link to improved mask images for that sequence). I have identified a few code sections that can be made significantly faster.
4. RGBD data from surfaces at large angles to the camera plane is typically unreliable from most off-the-shelf sensors. Noise from this data results in noise in the reconstructed scene, as well as increases the number of used voxel hash blocks (when those are used), thereby impacting runtime performance. A simple filter needs to be implemented to mitigate this.
5. *Biggest issue so far*: non-rigid alignment seems to be currently substandard on widely-used datasets. There might be some qualitative algorithmic problems or it might be a matter of tuning parameters, but certain parts of the "live" surface being aligned to the "canonical" seem like they are pulled too much into surrounding areas, whereas others seem like they are not pulled enough. 


For a full list of issues & tasks, see the [Dynamic-Scene Project Board](https://github.com/Algomorph/InfiniTAM/projects/1) and the [Maintenance/Reengineering Project Board]( https://github.com/Algomorph/InfiniTAM/projects/2). There are both algorithmic and software engineering issues (the code I've started with is far from perfect).

## How do I try this code out?

1. The primary mandatory 3rd-party requirement that I've added is Boost. The Boost modules the code currently relies on are iostreams, program_options, and property_tree. The Boost library might be a bit of a hassle to build on Windows if you haven't done it before. All other dependeincies are handled automatically via CMake and shouldn't pose a problem. If you're building on a system without internet, there are CMake options to configure the system to build 3rd-party requirements directly from sources included in the repository.
2. Linux currently is the only officially-supported OS, but I've recently fixed & tested the Windows build. All of the required CMake packages and this code in theory should work on any major platform, so you can try on MacOS at your own risk. Let me know if you'd like to fix things that are not working on your platform.
3. I recommend building with FFmpeg (```WITH_FFMPEG``` CMake flag), since that will enable visual debugging/video recording. Building with CUDA (```WITH_CUDA``` CMake flag) is also highly recommended if you have a good GPU, since CPU runtime has not yet been fully optimized. 
4. To get some test data and try the code on it, you can download the [original Snoopy sequence](http://campar.in.tum.de/personal/slavcheva/deformable-dataset/index.html) (better masks can be retrieved from http://algomorph.com/storage/reco/snoopy_masks.zip), modify Files/infinitam_snoopy_config.json with proper paths for input_and_output_settings_paths, and run like this (modify the path to point to infinitam_snoopy_config.json):

<build_folder>/Apps/InfiniTAM/InfiniTAM --config=Files/infinitam_snoopy_config.json

**Note**: If you build with FFmpeg, this will also record a video in the output folder (as specified in the config file).

## Is this code being worked on / maintained?

~Yes, after a looong break, I'm officially switching to try to do something with it again, at least for awhile. Even if I'm not actively working on it, I do my best to respond to new issues or collaboration requests.~

*[2021 UPDATE]* Having worked on this for over three years I have come to certain conclusions. Keep in mind: this comes after several corrections to the algorithm itself, some of them based on my correspondence with Mira Slavcheva herself. Also consider: the two other repositories I know of that even come remotely close to implementing her algorithms fully currently achieve much worse results (and in some cases, suffer from the same mistakes that I first made and then corrected here). These are: https://github.com/dgrzech/sobfu and https://github.com/al093/killingFusionCuda. *The bottom line* is that even though the code I have here still doesn't replicate Slavcheva's experiments one-to-one (the plain-voxel-array structure Slavcheva uses is populated densely with +1.0 and -1.0 values in the truncated region, not just the narrow band that InfiniTAM populates), it is clear to me that the algorithm doesn't work as described in the literature so far, i.e. either the tuning is completely off for the parameters or some details are omitted from the description. The former is unlikely since I've tried tuning the parameters with hyperopt (see python code) and so far haven't achieved any benefit.

Hence, I've decided to abandon this experimentation, since it seems like trying to figure out the missing details is going to take longer than coming up with something of my own that works better. I'm now looking into neural-network-driven / differentiable optimization solutions.

## Will I merge this back into InfiniTAM?

TLDR: ~Maybe.~ NO.

Originally, that was the plan. However, things have changed. A lot of things. The main and supporting libraries (right now still called, historically, ITMLib, ORUtils, and InputSource) contain at this point about 78000 lines of code. Only about 31000 of these come originally by the Oxford lab authors or people who derived from their code, or are not too heavily modified by yours truly. About 4000 of those are deprecated and are due for removal replacement by already-existing, newer and better code. Executables only contain about 500 or so lines of code that haven't been heavily modified or thrown out. In addition, the repository includes about 9000 lines of test code (at time of writing) and some Python-based tools for visual debugging, which are entirely new.

~Many~Some portions of this code are still privy to the original Oxford University Innovation Academic License included within, please consult file headers for which license is used for which part of the code when considering reuse.

The current plan is to launch a separate repository based on all these sources, restructure and rename all the libraries within, and release everything under the Apache v2 license (almost no usage restrictions, mainly for proper attributions) and under my own copyright.

Original InfiniTAM Repo & README: https://github.com/victorprad/InfiniTAM
