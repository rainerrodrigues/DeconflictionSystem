using .UAVDeconfliction
using Plots
using PlotlyJS
using Dates: Second

# Run test cases
mission1, mission2, mission3, optimized_mission = UAVDeconfliction.run_test_cases()

# Visualize the conflict scenario
conflicts = UAVDeconfliction.check_conflicts(mission2)
display(UAVDeconfliction.visualize_4d(mission2, conflicts))

# Visualize the optimized trajectory
optimized_conflicts = UAVDeconfliction.check_conflicts(
    UAVDeconfliction.Mission(optimized_mission, mission2.others)
)
display(UAVDeconfliction.visualize_4d(
    UAVDeconfliction.Mission(optimized_mission, mission2.others), 
    optimized_conflicts
))

# Create an animation of the conflict scenario
println("Creating animation...")
times = range(
    mission2.primary.waypoints[1].t, 
    mission2.primary.waypoints[end].t, 
    step=Second(5)
)

anim = @animate for t in times
    # Initialize empty plot
    p = Plots.scatter3d([], [], [], 
        title="UAV Positions at $(t)",
        xlabel="X (m)",
        ylabel="Y (m)",
        zlabel="Altitude (m)",
        legend=:topright,
        xlim=(0, 500),  # Adjust based on your coordinate range
        ylim=(0, 500),
        zlim=(0, 100)
    )
    
    # Plot primary drone position
    pos1 = UAVDeconfliction.interpolate_position(mission2.primary, t)
    if pos1 !== nothing
        Plots.scatter3d!(p, [pos1[1]], [pos1[2]], [pos1[3]], 
            label="Primary", 
            color=:blue, 
            markersize=5)
    end
    # Add other drones using Plots.scatter3d
    for traj in mission2.others
        pos = UAVDeconfliction.interpolate_position(traj, t)
        if pos !== nothing
            Plots.scatter3d!(
                [pos[1]], [pos[2]], [pos[3]], 
                label="Drone $(traj.drone_id)", 
                color=:red, 
                markersize=5
            )
        end
    end
    
    # Highlight conflicts
    for c in conflicts
        if abs((c.time - t).value) < 5000
            Plots.scatter3d!(
                [c.location[1]], [c.location[2]], [c.location[3]], 
                label="Conflict", 
                color=:yellow, 
                markershape=:x, 
                markersize=8
            )
        end
    end
    p
end

gif(anim, "uav_conflicts.gif", fps=5)
println("Animation saved as uav_conflicts.gif")