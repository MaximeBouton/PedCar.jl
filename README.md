# PedCar

[![Build Status](https://travis-ci.org/MaximeBouton/PedCar.jl.svg?branch=master)](https://travis-ci.org/MaximeBouton/PedCar.jl)

[![Coverage Status](https://coveralls.io/repos/MaximeBouton/PedCar.jl/badge.svg?branch=master&service=github)](https://coveralls.io/github/MaximeBouton/PedCar.jl?branch=master)

[![codecov.io](http://codecov.io/github/MaximeBouton/PedCar.jl/coverage.svg?branch=master)](http://codecov.io/github/MaximeBouton/PedCar.jl?branch=master)

This package provides a implementation of an urban intersection scenario as a discrete state MDP.
This model has been used in the following papers:

- M. Bouton, A. Nakhaei, K. Fujimura, and M. J. Kochenderfer, “Safe reinforcement learning with scene decomposition for navigating complex urban environments,” in *IEEE Intelligent Vehicles Symposium (IV)*, 2019.
- M. Bouton, J. Karlsson, A. Nakhaei, K. Fujimura, M. J. Kochenderfer, and J. Tumova, “Reinforcement learning with probabilistic guarantees for autonomous driving,” in *Workshop on Safety Risk and Uncertainty in Reinforcement Learning, Conference on Uncertainty in Artificial Intelligence (UAI)*, 2018.

## Installation

For julia > 1.1, you can add the SISL registry and the JuliaPOMDP registry (recommended): 
```julia
pkg> registry add https://github.com/sisl/Registry
pkg> registry add https://github.com/JuliaPOMDP/Registry
pkg> add https://github.com/MaximeBouton/PedCar.jl
```

otherwise you can install all the dependencies manually.

```julia
using Pkg
Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveDrivingModels.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoViz.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoUrban.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSensors.jl"))
Pkg.add("POMDPs")
using POMDPs
POMDPs.add_registry()
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotivePOMDPs.jl"))
Pkg.add(PackageSpec(url="https://github.com/MaximeBouton/PedCar.jl"))
```
