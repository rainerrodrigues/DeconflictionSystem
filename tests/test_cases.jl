module TestCases
using ..UAVDeconfliction  # Parent module
using Dates

export create_test_case_1, create_test_case_2, create_test_case_3, create_test_case_4

function create_test_case_1()
    # Test Case 1: No conflict (perpendicular paths at different times)
    wp1 = [
        UAVDeconfliction.Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T00:00:00")),
        UAVDeconfliction.Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:01:00")),
        UAVDeconfliction.Waypoint(100.0, 100.0, 10.0, DateTime("2023-01-01T00:02:00"))
    ]
    primary = UAVDeconfliction.Trajectory(wp1, "Primary", 10.0, 5.0)
    
    wp2 = [
        UAVDeconfliction.Waypoint(0.0, 100.0, 10.0, DateTime("2023-01-01T00:10:00")),
        UAVDeconfliction.Waypoint(100.0, 100.0, 10.0, DateTime("2023-01-01T00:11:00")),
        UAVDeconfliction.Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:12:00"))
    ]
    other = UAVDeconfliction.Trajectory(wp2, "Other1", 10.0, 5.0)
    
    return UAVDeconfliction.Mission(primary, [other])
end

function create_test_case_2()
    # Test Case 2: Spatial conflict
    wp1 = [
        UAVDeconfliction.Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T00:00:00")),
        UAVDeconfliction.Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:01:00")),
        UAVDeconfliction.Waypoint(100.0, 100.0, 10.0, DateTime("2023-01-01T00:02:00"))
    ]
    primary = UAVDeconfliction.Trajectory(wp1, "Primary", 10.0, 5.0)
    
    wp2 = [
        UAVDeconfliction.Waypoint(50.0, -10.0, 10.0, DateTime("2023-01-01T00:00:30")),
        UAVDeconfliction.Waypoint(50.0, 10.0, 10.0, DateTime("2023-01-01T00:01:30"))
    ]
    other = UAVDeconfliction.Trajectory(wp2, "Other2", 5.0, 5.0)
    
    return UAVDeconfliction.Mission(primary, [other])
end

function create_test_case_3()
    # Test Case 3: Temporal conflict (same space, different times)
    wp1 = [
        UAVDeconfliction.Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T00:00:00")),
        UAVDeconfliction.Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:01:00"))
        ]
    primary = Trajectory(wp1, "Primary", 10.0, 5.0)
    wp2 = [
        UAVDeconfliction.Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T01:00:00")),
        UAVDeconfliction.Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T01:01:00"))
        ]
    other = Trajectory(wp2, "Other3", 10.0, 5.0)

    return UAVDeconfliction.Mission(primary, [other])
    
end

function create_test_case_4()
    # First create a conflict scenario (similar to test case 2)
    wp1 = [
        UAVDeconfliction.Waypoint(0.0, 0.0, 10.0, DateTime("2023-01-01T00:00:00")),
        UAVDeconfliction.Waypoint(100.0, 0.0, 10.0, DateTime("2023-01-01T00:01:00")),
        UAVDeconfliction.Waypoint(100.0, 100.0, 10.0, DateTime("2023-01-01T00:02:00"))
        ]
    primary = UAVDeconfliction.Trajectory(wp1, "Primary", 10.0, 5.0)
    
    # Create multiple interfering trajectories
    other1 = UAVDeconfliction.Trajectory(
        [
            UAVDeconfliction.Waypoint(50.0, -10.0, 10.0, DateTime("2023-01-01T00:00:30")),
            UAVDeconfliction.Waypoint(50.0, 10.0, 10.0, DateTime("2023-01-01T00:01:30"))
        ],
        "Interfering1", 5.0, 5.0
        )
    
    other2 = UAVDeconfliction.Trajectory(
        [
                UAVDeconfliction.Waypoint(80.0, 80.0, 10.0, DateTime("2023-01-01T00:00:45")),
                UAVDeconfliction.Waypoint(20.0, 80.0, 10.0, DateTime("2023-01-01T00:01:45"))
        ],
        "Interfering2", 7.0, 5.0
        )
    
    # Create the initial conflicting mission
    initial_mission = UAVDeconfliction.Mission(primary, [other1, other2])
    
    # Define optimization bounds (x, y, z in meters, time in seconds)
    bounds = [
        (0.0, 200.0),   # x bounds
        (0.0, 200.0),   # y bounds
        (5.0, 50.0),    # z bounds
        (-30.0, 30.0)   # time adjustment bounds (seconds)
    ]
    
    # Run optimization (with reduced parameters for testing)
    optimized_traj = UAVDeconfliction.optimize_trajectory(
        primary, 
        [other1, other2], 
        bounds,
        20,  # iterations
        15   # population size
        )
    
    # Return a named tuple with proper syntax
    return (
        initial_mission = initial_mission,
        optimized_mission = UAVDeconfliction.Mission(optimized_traj, [other1, other2]),
        bounds = bounds
        )
end
end