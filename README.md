# The CVRP

This repository offers a branch and cut method for solving the capacitated vehicule routing problem (CVRP).
The CVRP problem is strongly NP-complete and has been the subject of countless papers in the last 50 years.

This project relies on:
    - the open source solver pyomo (for solving linear optimization)
    - the open source GoogleOpt library (for computing a lower-bound of the solution)
    - the open source C++ library cvrpsep (comprises several complex seperation methods)
    
# How to use ?

The main file is cvrp.py; it is used to launch the solving of a CVRP problem specified in a .dat file (python cvrp.py cvrpfile.dat)
All meta parameters can be modified in the globals.py file 

    