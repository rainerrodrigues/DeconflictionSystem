# tests/test_cases.jl
module TestCases
using ..DeconflictionSystem, Dates
export create_no_conflict_case, create_spatial_conflict_case, 
       create_temporal_conflict_case, create_edge_cases

# 1. Basic no-conflict scenario
function create_no_conflict_case()
    primary = Trajectory(
        [Waypoint(0,0,10,now()), Waypoint(100,0,10,now()+Minute(1))],
        "DroneA", 10.0, 5.0)
    other = Trajectory(
        [Waypoint(0,50,10,now()), Waypoint(100,50,10,now()+Minute(1))],
        "DroneB", 10.0, 5.0)
    return Mission(primary, [other])
end

# 2. Spatial conflict case
function create_spatial_conflict_case()
    t = now()
    primary = Trajectory(
        [Waypoint(0,0,10,t), Waypoint(100,0,10,t+Minute(1))],
        "DroneA", 10.0, 5.0)
    other = Trajectory(
        [Waypoint(50,-5,10,t+Second(30)), Waypoint(50,5,10,t+Second(90))],
        "DroneB", 5.0, 5.0)
    return Mission(primary, [other])
end

# 3. Temporal separation case
function create_temporal_conflict_case()
    primary = Trajectory(
        [Waypoint(0,0,10,now()), Waypoint(100,0,10,now()+Minute(1))],
        "DroneA", 10.0, 5.0)
    other = Trajectory(
        [Waypoint(0,0,10,now()+Hour(1)), Waypoint(100,0,10,now()+Hour(1)+Minute(1))],
        "DroneB", 10.0, 5.0)
    return Mission(primary, [other])
end

# 4. Edge cases collection
function create_edge_cases()
    cases = []
    
    # Case 1: Minimal separation
    push!(cases, (
        Trajectory([Waypoint(0,0,10,now()), Waypoint(10,0,10,now()+Second(1))], "DroneA", 1.0, 4.99),
        Trajectory([Waypoint(0,4.99,10,now()), Waypoint(10,4.99,10,now()+Second(1))], "DroneB", 1.0, 4.99)
    ))
    
    # Case 2: Vertical separation
    push!(cases, (
        Trajectory([Waypoint(0,0,10,now()), Waypoint(0,0,20,now()+Second(1))], "DroneA", 1.0, 5.0),
        Trajectory([Waypoint(0,0,30,now()), Waypoint(0,0,40,now()+Second(1))], "DroneB", 1.0, 5.0)
    ))
    
    return cases
end
end