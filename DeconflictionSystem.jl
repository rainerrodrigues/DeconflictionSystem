module UAVDeconfliction

using LinearAlgebra
using StaticArrays
using Plots
using PlotlyJS
using DataStructures
using Dates
using Random
using Distributions
using LightGraphs
using Metaheuristics
using ProgressMeter

export Waypoint, Trajectory, Mission, Conflict, DeconflictionSystem


# Data Structures
struct Waypoint
    x::Float64
    y::Float64
    z::Float64
    t::DateTime  # Time at waypoint
end

struct Trajectory
    waypoints::Vector{Waypoint}
    drone_id::String
    speed::Float64  # m/s
    safety_buffer::Float64  # meters
end

struct Mission
    primary::Trajectory
    others::Vector{Trajectory}
end

struct Conflict
    location::Tuple{Float64,Float64,Float64}
    time::DateTime
    drone1::String
    drone2::String
    distance::Float64
end

struct DeconflictionSystem
    missions::PriorityQueue{String,Mission}  # Priority based on mission urgency
    spatial_kdtree::KDTree  # For spatial indexing (simplified here)
    temporal_index::Dict{DateTime,Vector{String}}  # Time-based indexing
end