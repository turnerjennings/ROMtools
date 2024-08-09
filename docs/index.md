# ROMtools

Rigid Body Dynamics tools for 2-D reduced order modelling of structural dynamics

## Installation

Package is not available on pip pending full testing.  Repository can be downloaded and installed locally:

    py -m pip install .


## Project Overview

The objective of this python package is to provide a set of tools for building simplified physics models of complex structural impact and dynamics problems.  The library includes flexible workflows for defining complex multibody dynamics problems and a object-oriented solver which allows for fast iteration and optimization of models.

### Parts of a model

A reduced-order model is comprised of several components:

1. Rigid bodies - objects of question in the analysis can be defined as a predetermined shape (disc, circle, semicircle) or as an arbitrary object by specifying mass and inertia properties.  Initial conditions for each body can also be specified.
2. Springs and Dampers - Rigid bodies are connected to each other and to fixed boundaries by linear and rotational spring/damper systems.
3. External loads - Additional external forces (gravity, impact loads) may be specified globally or applied to specific rigid bodies.
4. Configuration - A configuration object is defined which stores information and settings for a solution to the model defined above.
5. Solver - Model components and a configuration are passed to a solver object which performs the physics simulation and stores and saves the results.
6. (Optional) Animator - A solver can be passed to an animator after a solution is obtained to create plots of the resultant behavior

Each component is described in subsequent sections of the documentation.


