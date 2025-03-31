
include("src/DeconflictionSystem.jl")  
include("test/test_cases.jl")     

module Demonstration

# Load dependencies first
import DeconflictionSystem]
using  Main.TestCases
using Plots, Dates



function run_demonstration()
    println("Starting UAV Deconfliction Demonstration\n")

    # Test Case 1: No conflict
    println("Running Test Case 1: No conflict scenario")
    mission1 = TestCases.create_no_conflict_case()
    conflicts1 = DeconflictionSystem.check_conflicts(mission1)
    @assert isempty(conflicts1) "Test Case 1 failed - Found conflicts when none expected"
    println("✓ Passed - No conflicts detected\n")

    # Test Case 2: Spatial conflict
    println("Running Test Case 2: Spatial conflict scenario")
    mission2 = TestCases.create_spatial_conflict_case()
    conflicts2 = DeconflictionSystem.check_conflicts(mission2)
    @assert !isempty(conflicts2) "Test Case 2 failed - No conflicts detected when expected"
    println("✓ Passed - Found $(length(conflicts2)) conflicts")
    display(DeconflictionSystem.visualize_4d(mission2, conflicts2))
    println()

    # Test Case 3: Temporal separation
    println("Running Test Case 3: Temporal separation scenario")
    mission3 = TestCases.create_temporal_conflict_case()
    conflicts3 = DeconflictionSystem.check_conflicts(mission3)
    @assert isempty(conflicts3) "Test Case 3 failed - Found conflicts when none expected"
    println("✓ Passed - No conflicts detected despite spatial overlap\n")

    # Test Case 4: Optimization
    println("Running Test Case 4: Trajectory optimization")
    case4 = TestCases.create_edge_cases()
    
    # Show initial conflicts
    initial_conflicts = DeconflictionSystem.check_conflicts(case4.initial_mission)
    println("- Initial conflicts: $(length(initial_conflicts))")
    display(DeconflictionSystem.visualize_4d(case4.initial_mission, initial_conflicts))
    
    # Show optimized results
    optimized_conflicts = DeconflictionSystem.check_conflicts(case4.optimized_mission)
    println("- Optimized conflicts: $(length(optimized_conflicts))")
    display(DeconflictionSystem.visualize_4d(case4.optimized_mission, optimized_conflicts))
    
    @assert length(optimized_conflicts) < length(initial_conflicts) "Optimization failed to reduce conflicts"
    println("✓ Passed - Reduced conflicts from $(length(initial_conflicts)) to $(length(optimized_conflicts))\n")

    # Create animation of optimization results
    println("Creating animation of optimization results...")
    create_animation(case4)
    println("Animation saved as 'optimization_result.gif'")
end

function create_animation(case4)
    # Create time range covering both missions
    all_times = vcat(
        [wp.t for wp in case4.initial_mission.primary.waypoints],
        [wp.t for wp in case4.optimized_mission.primary.waypoints]
    )
    start_time = minimum(all_times)
    end_time = maximum(all_times)
    times = start_time:Second(5):end_time  # Frame every 5 seconds

    anim = @animate for t in times
        p = Plots.plot(layout=(1,2), size=(1200,600))
        
        # Initial mission frame
        Plots.scatter3d!(p[1], 
            title="Initial Trajectory at $(t)",
            xlabel="X", ylabel="Y", zlabel="Z",
            legend=:topright)
        
        # Optimized mission frame
        Plots.scatter3d!(p[2],
            title="Optimized Trajectory at $(t)",
            xlabel="X", ylabel="Y", zlabel="Z",
            legend=:topright)
        
        # Plot positions at current time
        plot_positions!(p[1], case4.initial_mission, t)
        plot_positions!(p[2], case4.optimized_mission, t)
        
        p
    end
    
    Plots.gif(anim, "optimization_result.gif", fps=5)
end

function plot_positions!(plot, mission, t)
    # Plot primary drone
    pos = DeconflictionSystem.interpolate_position(mission.primary, t)
    if pos !== nothing
        Plots.scatter3d!(plot, [pos[1]], [pos[2]], [pos[3]], 
            label="Primary", color=:blue, markersize=5)
    end
    
    # Plot other drones
    for traj in mission.others
        pos = DeconflictionSystem.interpolate_position(traj, t)
        if pos !== nothing
            Plots.scatter3d!(plot, [pos[1]], [pos[2]], [pos[3]], 
                label="Drone $(traj.drone_id)", color=:red, markersize=5)
        end
    end
end

# Export the main function
export run_demonstration

end  # end module

# Execution guard
if abspath(PROGRAM_FILE) == @__FILE__
    using .Demonstration
    Demonstration.run_demonstration()
end