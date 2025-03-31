# tests/runtests.jl
include("../DeconflictionSystem.jl")
include("test_cases.jl")

using .UAVDeconfliction, .TestCases
using Test

@testset "Basic Conflict Detection" begin
    # No conflict case
    mission = create_no_conflict_case()
    @test isempty(check_conflicts(mission))
    
    # Spatial conflict case
    mission = create_spatial_conflict_case()
    @test !isempty(check_conflicts(mission))
    
    # Temporal separation case
    mission = create_temporal_conflict_case()
    @test isempty(check_conflicts(mission))
end

@testset "Edge Cases" begin
    for (i, (traj1, traj2)) in enumerate(create_edge_cases())
        @testset "Edge Case $i" begin
            mission = Mission(traj1, [traj2])
            result = check_conflicts(mission)
            # Expect conflict when buffers overlap
            expected = traj1.safety_buffer + traj2.safety_buffer > 
                      sqrt(sum((traj1.waypoints[1].x - traj2.waypoints[1].x)^2 +
                           (traj1.waypoints[1].y - traj2.waypoints[1].y)^2 +
                           (traj1.waypoints[1].z - traj2.waypoints[1].z)^2))
            @test !isempty(result) == expected
        end
    end
end