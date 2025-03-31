# DeconflictionSystem
Deconfliction system for drone trajectory implemented in Julia

## Installation
Install [Julia](https://julialang.org/downloads/) <br>
Install the following libraries in the Julia REPL:
```Julia
julia
]
activate . #Creating your own environment
add LinearAlgebra StaticArrays Plots PlotlyJS DataStructures Dates Random Distributions LightGraphs Metaheuristics ProgressMeter NearestNeighbors Revise Test
```
Once installed and sucessfully precompiled, type the following in the Julia REPL
```
julia Demonstration.jl
```
## Testing
You can see the testing via REPL:
```
test
```
## Demonstration

![Optimization Result](https://github.com/rainerrodrigues/DeconflictionSystem/blob/main/optimization_result.gif)
