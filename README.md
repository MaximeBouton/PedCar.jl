# PedCar

[![Build Status](https://travis-ci.org/MaximeBouton/PedCar.jl.svg?branch=master)](https://travis-ci.org/MaximeBouton/PedCar.jl)

[![Coverage Status](https://coveralls.io/repos/MaximeBouton/PedCar.jl/badge.svg?branch=master&service=github)](https://coveralls.io/github/MaximeBouton/PedCar.jl?branch=master)

[![codecov.io](http://codecov.io/github/MaximeBouton/PedCar.jl/coverage.svg?branch=master)](http://codecov.io/github/MaximeBouton/PedCar.jl?branch=master)

## Installation

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