# solomon-reimp
Reimplementation of Solomon et al's Linear Reactive Control bipedal system. Featured in Chapters 4,6,7 and 8 of Neuroevolution of Bipedal Locomotion: algorithmic, balance penalty and morphological improvements for improved robustness and performance

The folder "Body Remaster" contains the reimplementations of Solomon et al's work in the ODE engine, without 2D engine bootstrapping used throughout the thesis.

By compiling the four different main files with 

x

you can then run the code with 

y

When run with PLAYBACK set to 0, the systems will run for the specified amount of generations with the given population size. 
After this, it will save a seeded genotype (X_Y.csv) and other results files to the same location as the main file. 

With PLAYBACK set to 1, the systems will then prompt you for a seed, which you can then enter (this is represented as X in the previous line).
The systems should find the file and run your seed, displaying the behaviour in the ODE engine.

MAIN FILES

1-main.cpp
The base reimplementation used in Chapter 4.

2.1-main_bigtuck.cpp
2.2-main_sqbase.cpp
2.3-main_bigtuck_i.cpp
2.4-main_sqbase_i.cpp

The albatross morphologies (and their morphological versions) used in Chapter 6.

3-main_turn.cpp
The turning task version of the reimplementation used in Chapter 7.

4-main_turninc.cpp
The incremental turning task version of the reimplemenation used in Chapter 8.
