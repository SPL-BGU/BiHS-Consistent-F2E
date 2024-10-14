<h1 align="center">Bridging Theory and Practice in Bidirectional Heuristic Search with Front-to-End Consistent Heuristics</h1>
<p align="center">
<a href="https://github.com/SPL-BGU/BiHS-Consistent-F2E/blob/main/LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>

The code for his repo is based on Prof. Nathan Sturtevant's HOG2 implementation which can be
found [here](https://github.com/nathansttt/hog2). <br/>
The code has been mainly run on the Ubuntu operating system using WSL and natively. <br/>
It was also run using Slurm Workload Manager operating on Rocky Linux. <br/>

# Compiling the Code
The code was compiled using GCC and g++ versions 9.4.0. <br/>
You first need to download the dependencies, so run the following command for the headless version of the code.

```sh
# apt install build-essential
```

If you want the non-headless version of HOG2 and the code, or the above does not work, try the following:

```sh
# apt install build-essential libglu1-mesa-dev freeglut3-dev libsfml-dev
```

Then use the compile script (you can run it with -h to view all options). You should run said script from inside the
scripts directory.
