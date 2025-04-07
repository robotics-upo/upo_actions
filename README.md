# UPO Actions Package

This package is used mainly by the theta_star_2d (in future 2d and 3d) package and the arco_path_tracker package. The idea is to avoid dependency between planners and trackers, this way you can build your own tracker or planner and make it work with an old tracker or planner just using this common action package.