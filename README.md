# EleNa: Elevation-based Navigation

The usual Navigation System tries to get the shortest or the fastest path from one point to another. However, in some cases, having the ability to further customize the path is desirable. Such a case could arise if, while working out, following a path that maximizes or minimizes the elevation gain is more convenient than the usual shortest path in order to get better training results. Therefore, the EleNa project tries to find the best path between two points according to the user preferences regarding elevation gain and some distance constraint with respect to the shortest path.

## Getting Started

Simply clone the following repository to your local machine:

```
git clone https://github.com/clankster99/CS520.git
```

### Prerequisites

You need to have installed python 2.7. Also, the OSMnx library is needed:

```
pip install osmnx
```

If you are using Anaconda, then you may use the following command:
```
conda install -c conda-forge osmnx
```

### Installing

Once the repository is cloned, simply go to the directory's repository and run

```
python Main.py
```

The screen will show what to do next. It will ask for the following :
1. Origin's Latitude and Longitude
2. Destination's Latitude and Longitude
3. The percentage of distance, the user is willing to travel extra over shortest path length
4. User is trying to 'maximize' or 'minimize' the elevation.

We have implemented three different algorithms to solve the above problem. Please change the strategy variable to '1' / '2' / '3'  in the main.py to try our algorithms.

The output would the program would be a graph figure showing the route between origin and destination.

## Running the tests

Explain how to run tests (or the graphs maybe)

## Built With

* [OSMnx](https://osmnx.readthedocs.io/en/stable/) - API used

## Versioning

We use [Git](https://git-scm.com/) for versioning. 

## Authors

See also the list of [contributors](https://github.com/clankster99/CS520/settings/collaboration) who participated in this project.
