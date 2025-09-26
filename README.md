<h1 align="center">Bridging Theory and Practice in Bidirectional Heuristic Search with Front-to-End Consistent Heuristics</h1>
<p align="center">
<a href="https://github.com/SPL-BGU/BiHS-Consistent-F2E/blob/main/LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>

You can find the paper in [https://doi.org/10.24963/ijcai.2023/625 ](https://doi.org/10.24963/ijcai.2023/625).

The code for his repo is based on Prof. Nathan Sturtevant's HOG2 implementation which can be
found [here](https://github.com/nathansttt/hog2). <br/>
The code has been mainly run on the Ubuntu operating system using WSL and natively. <br/>
It was also run using Slurm Workload Manager operating on Rocky Linux. <br/>

## Compiling the Code
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


## Running the Experiments
To run the experiments which appeared in the conference you need to run ex-conference.sh.
You can use the -h flag to view all options, but in general the flags correspond to a domain.<br/>
For example, the following will run the stp and road experiments of the conference code.

```sh
./scripts/exp-conference.sh --stp --road
```

This will run the journal experiments for pancake domain and linear combination experiment.

```sh
./scripts/exp-journal.sh --pancake --linear
```

## Generating Figures

Once you have all the results, you need to turn them into CSVs using the following, and then recreate the figures:

```sh
./scripts/csvs.sh
./scripts/figures.sh
```

Note that csvs.sh will run on all domains, so it might take a minute or two, depending on how many individual log files there
are. The more files, the longer it takes (you can simply concat files to speed this up).

The GMX and linear combination tables were done manually, and some tweaks may have been applied to the tables after their generation.

## Known Issues
The max policy had an issue in the conference code. This was rectified in the journal code and in the paper.

The interface to recreate the results was written after the fact, meaning it was not as heavily tested as the code for the papers.
This means that you might encounter issues with it. If you encounter any issue, find a bug, or need help, feel free to open an issue or contact Lior (the maintainer).

## Citation and Code Attribution
The new code for the journal paper can be found in
[journal/apps/aij](https://github.com/SPL-BGU/BiHS-Consistent-F2E/tree/main/journal/apps/aij).<br/>
The code corresponding with the previous conference paper can be found in
[conference/apps/bidirectional](https://github.com/SPL-BGU/BiHS-Consistent-F2E/tree/main/conference/apps/bidirectional)
which is a fork of Vidal Alcázar's code which can be found
[here](https://github.com/valcazar/hog2/tree/PDB-refactor/apps/bidirectional). <br/>

As said before, all of these rely on HOG2 which can be found [here](https://github.com/nathansttt/hog2). 

If you find our work interesting or the repo useful, please consider citing this paper:
```
@article{siag2025bridging,
	title = {Bridging theory and practice in bidirectional heuristic search with front-to-end consistent heuristics},
	author = {Lior Siag and Shahaf S. Shperberg},
	journal = {Artificial Intelligence},
	volume = {348},
	pages = {104420},
	year = {2025},
	issn = {0004-3702},
	doi = {https://doi.org/10.1016/j.artint.2025.104420},
}
```
