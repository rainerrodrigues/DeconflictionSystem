module UAVDeconfliction
 
 using LinearAlgebra
 using StaticArrays
 using Plots
 using PlotlyJS: PlotlyJS, scatter3d, Layout, attr
 using DataStructures
 using Dates
 using Random
 using Distributions
 using LightGraphs
 using Metaheuristics
 using ProgressMeter
 using NearestNeighbors
 
 export Waypoint, Trajectory, Mission, Conflict, DeconflictionSystem
 export check_conflicts, optimize_trajectory, visualize_4d, run_test_cases
 
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
     spatial_kdtree::NearestNeighbors.KDTree  # For spatial indexing (simplified here)
     temporal_index::Dict{DateTime,Vector{String}}  # Time-based indexing
 end
 
 # Core Conflict Detection Functions
 function interpolate_position(traj::Trajectory, t::DateTime)
    isempty(traj.waypoints) && return nothing
    t < traj.waypoints[1].t && return traj.waypoints[1]
    t > traj.waypoints[end].t && return traj.waypoints[end]
     # Linear interpolation between waypoints
     for i in 1:length(traj.waypoints)-1
         wp1 = traj.waypoints[i]
         wp2 = traj.waypoints[i+1]
         if wp1.t <= t <= wp2.t
             Δt = (t - wp1.t).value / 1000  # Convert to seconds
             total_time = (wp2.t - wp1.t).value / 1000
             ratio = Δt / total_time
             
             x = wp1.x + ratio * (wp2.x - wp1.x)
             y = wp1.y + ratio * (wp2.y - wp1.y)
             z = wp1.z + ratio * (wp2.z - wp1.z)
             
             return (x, y, z)
         end
     end
     return nothing
 end
 
 # Building spatial index for comparison of trajectories
 function build_spatial_index(trajectories::Vector{Trajectory})
     points = Vector{SVector{3,Float64}}()
     for traj in trajectories
         for wp in traj.waypoints
             push!(points, SVector(wp.x, wp.y, wp.z))
         end
     end
     return KDTree(points)
 end
 
 function check_spatial_conflict(traj1::Trajectory, traj2::Trajectory, time_step::Period=Second(1))
     conflicts = Vector{Conflict}()
     
     # Get overlapping time window
     t_start = max(traj1.waypoints[1].t, traj2.waypoints[1].t)
     t_end = min(traj1.waypoints[end].t, traj2.waypoints[end].t)
     
     t_start <= t_end || return conflicts  # No temporal overlap
     
     # Check at regular intervals
     current_time = t_start
     while current_time <= t_end
         pos1 = interpolate_position(traj1, current_time)
         pos2 = interpolate_position(traj2, current_time)
         
         if pos1 !== nothing && pos2 !== nothing
             dist = norm([pos1[1]-pos2[1], pos1[2]-pos2[2], pos1[3]-pos2[3]])
             min_dist = traj1.safety_buffer + traj2.safety_buffer
             
             if dist < min_dist
                 push!(conflicts, Conflict(
                     ((pos1[1]+pos2[1])/2, (pos1[2]+pos2[2])/2, (pos1[3]+pos2[3])/2),
                     current_time,
                     traj1.drone_id,
                     traj2.drone_id,
                     dist
                 ))
             end
         end
         
         current_time += time_step
     end
     
     return conflicts
 end
 
 function check_conflicts(mission::Mission, time_step::Period=Second(1))
    # Input validation
    @assert time_step.value > 0 "Time step must be positive"
    @assert !isempty(mission.primary.waypoints) "Primary trajectory has no waypoints"
    try
        all_conflicts = Vector{Conflict}()
        @showprogress for other in mission.others
        conflicts = check_spatial_conflict(mission.primary, other, time_step)
        append!(all_conflicts, conflicts)
        return all_conflicts
        end
    catch e
        @error "Conflict detection failed" exception=(e, catch_backtrace())
        return Conflict[]  # Fail-safe return
    end

     
end
 
 # Optimization using Bio-inspired Algorithms
 function trajectory_cost(traj::Trajectory)
     # Simple cost function: total distance + time penalty
     distance = 0.0
     time = 0.0
     
     for i in 1:length(traj.waypoints)-1
         wp1 = traj.waypoints[i]
         wp2 = traj.waypoints[i+1]
         distance += norm([wp2.x-wp1.x, wp2.y-wp1.y, wp2.z-wp1.z])
         time += (wp2.t - wp1.t).value / 1000  # seconds
     end
     
     return distance + 0.1*time  # Weighted sum
 end
 
 function optimize_trajectory(initial_traj::Trajectory, other_trajs::Vector{Trajectory}, 
    bounds::Vector{Tuple{Float64,Float64}}, 
    max_iter::Int=50, population_size::Int=30)
     # Bee Swarm Optimization to find conflict-free trajectory
     
     # Objective function to minimize (conflicts + cost)
     function objective(x)
         # x is a vector representing waypoint adjustments
         new_waypoints = deepcopy(initial_traj.waypoints)
         for i in 1:length(new_waypoints)
             # Adjust x, y, z, t within bounds
             new_waypoints[i] = Waypoint(
                 clamp(new_waypoints[i].x + x[4i-3], bounds[1][1], bounds[1][2]),
                 clamp(new_waypoints[i].y + x[4i-2], bounds[2][1], bounds[2][2]),
                 clamp(new_waypoints[i].z + x[4i-1], bounds[3][1], bounds[3][2]),
                 new_waypoints[i].t + Millisecond(round(Int, x[4i]*1000))
             )
         end
         
         new_traj = Trajectory(new_waypoints, initial_traj.drone_id, initial_traj.speed, initial_traj.safety_buffer)
         conflicts = sum(length(check_spatial_conflict(new_traj, other)) for other in other_trajs)
         cost = trajectory_cost(new_traj)
         
         return conflicts * 1000 + cost  # Heavy penalty for conflicts
     end
     
     # Set up optimization problem
     D = 4 * length(initial_traj.waypoints)  # x,y,z,t for each waypoint
     lower = [-1.0 for _ in 1:D]  # Adjustments
     upper = [1.0 for _ in 1:D]

     # Create bounds matrix (required by current Metaheuristics API)
    bounds_matrix = Matrix{Float64}(undef, D, 2)
    for i in 1:D
        bounds_matrix[i,1] = lower[i]
        bounds_matrix[i,2] = upper[i]
    end
     
    # Set up ECA algorithm with current API
    options = Metaheuristics.Options(iterations=max_iter, show_results=true)
    algorithm = Metaheuristics.ECA(;N=population_size, options=options)

    # Run optimization (using current recommended approach)
    result = Metaheuristics.optimize(objective, bounds_matrix, algorithm)

     # Apply best solution
     best_x = minimizer(result)
     optimized_waypoints = deepcopy(initial_traj.waypoints)
     for i in 1:length(optimized_waypoints)
         optimized_waypoints[i] = Waypoint(
             clamp(optimized_waypoints[i].x + best_x[4i-3], bounds[1][1], bounds[1][2]),
             clamp(optimized_waypoints[i].y + best_x[4i-2], bounds[2][1], bounds[2][2]),
             clamp(optimized_waypoints[i].z + best_x[4i-1], bounds[3][1], bounds[3][2]),
             optimized_waypoints[i].t + Millisecond(round(Int, best_x[4i]*1000))
         )
     end
     
     return Trajectory(optimized_waypoints, initial_traj.drone_id, initial_traj.speed, initial_traj.safety_buffer)
 end
 
 # Visualization
 function visualize_4d(mission::Mission, conflicts::Vector{Conflict}=Conflict[])
     # Create 4D plot (3D space + color for time)
     trace_primary = PlotlyJS.scatter3d(
         x=[wp.x for wp in mission.primary.waypoints],
         y=[wp.y for wp in mission.primary.waypoints],
         z=[wp.z for wp in mission.primary.waypoints],
         mode="lines+markers",
         name="Primary Drone",
         line=attr(width=4, color="blue"),
         marker=attr(size=4, color=[Dates.value(wp.t) for wp in mission.primary.waypoints], 
                   colorscale="Viridis", showscale=true, colorbar=attr(title="Time"))
     )
     
     traces = [trace_primary]
     
     # Add other drones
     for (i, traj) in enumerate(mission.others)
         push!(traces, PlotlyJS.scatter3d(
             x=[wp.x for wp in traj.waypoints],
             y=[wp.y for wp in traj.waypoints],
             z=[wp.z for wp in traj.waypoints],
             mode="lines+markers",
             name="Drone $(traj.drone_id)",
             line=attr(width=2, color="red"),
             marker=attr(size=3, color=[Dates.value(wp.t) for wp in traj.waypoints], 
                       colorscale="Viridis", showscale=false)
         ))
     end
     
     # Add conflict points
     if !isempty(conflicts)
         push!(traces, scatter3d(
             x=[c.location[1] for c in conflicts],
             y=[c.location[2] for c in conflicts],
             z=[c.location[3] for c in conflicts],
             mode="markers",
             name="Conflicts",
             marker=attr(size=8, color="yellow", symbol="x")
         ))
     end
     
     layout = PlotlyJS.Layout(
         title="4D UAV Trajectories (Color Represents Time)",
         scene=attr(
             xaxis_title="X (m)",
             yaxis_title="Y (m)",
             zaxis_title="Altitude (m)",
             camera=attr(eye=attr(x=1.5, y=1.5, z=0.8))
         ),
         width=1000,
         height=800
     )
     
     return PlotlyJS.plot(traces, layout)
 end
 
 # Test Cases
 function run_test_cases()
     println("Running test cases...")
     
     # Test Case 1: No conflict (drones moving in perpendicular paths at different times)
     println("\nTest Case 1: No conflict")
     wp1 = [
         Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T00:00:00")),
         Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:01:00")),
         Waypoint(100.0, 100.0, 10.0, DateTime("2023-01-01T00:02:00"))
     ]
     primary = Trajectory(wp1, "Primary", 10.0, 5.0)
     
     wp2 = [
         Waypoint(0.0, 100.0, 10.0, DateTime("2023-01-01T00:10:00")),  # Different time window
         Waypoint(100.0, 100.0, 10.0, DateTime("2023-01-01T00:11:00")),
         Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:12:00"))
     ]
     other = Trajectory(wp2, "Other1", 10.0, 5.0)
     
     mission = Mission(primary, [other])
     conflicts = check_conflicts(mission)
     if !isempty(conflicts)
         println("Test Case 1 failed - Found conflicts:")
         for c in conflicts
             println("  Conflict at $(c.location) at time $(c.time) between $(c.drone1) and $(c.drone2)")
         end
     else
         println("✓ Passed")
     end
     
     # Test Case 2: Spatial conflict
     println("\nTest Case 2: Spatial conflict")
     wp3 = [
         Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T00:00:00")),
         Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:01:00")),
         Waypoint(100.0, 100.0, 10.0, DateTime("2023-01-01T00:02:00"))
     ]
     primary2 = Trajectory(wp3, "Primary2", 10.0, 5.0)
     
     wp4 = [
         Waypoint(50.0, -10.0, 10.0, DateTime("2023-01-01T00:00:30")),
         Waypoint(50.0, 10.0, 10.0, DateTime("2023-01-01T00:01:30"))
     ]
     other2 = Trajectory(wp4, "Other2", 5.0, 5.0)
     
     mission2 = Mission(primary2, [other2])
     conflicts2 = check_conflicts(mission2)
     @assert !isempty(conflicts2) "Test Case 2 failed: Expected conflicts"
     println("✓ Passed - Found $(length(conflicts2)) conflicts")
     
     # Test Case 3: Temporal conflict (same space, different times)
     println("\nTest Case 3: Temporal conflict check")
     wp5 = [
         Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T00:00:00")),
         Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:01:00"))
     ]
     primary3 = Trajectory(wp5, "Primary3", 10.0, 5.0)
     
     wp6 = [
         Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T01:00:00")),
         Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T01:01:00"))
     ]
     other3 = Trajectory(wp6, "Other3", 10.0, 5.0)
     
     mission3 = Mission(primary3, [other3])
     conflicts3 = check_conflicts(mission3)
     @assert isempty(conflicts3) "Test Case 3 failed: Expected no conflicts (different times)"
     println("✓ Passed")
     
     # Test Case 4: Optimization
     println("\nTest Case 4: Trajectory optimization")
     bounds = [(0.0, 200.0), (0.0, 200.0), (0.0, 50.0)]  # x, y, z bounds
     optimized = optimize_trajectory(primary2, [other2], bounds)
     conflicts_after = check_conflicts(Mission(optimized, [other2]))
     @assert length(conflicts_after) < length(conflicts2) "Test Case 4 failed: Optimization didn't reduce conflicts"
     println("✓ Passed - Reduced conflicts from $(length(conflicts2)) to $(length(conflicts_after))")
     
     return mission, mission2, mission3, optimized
 end
 
 end  # module