# tests/qa_validation.jl
function run_qa_suite()
    println("Running QA Validation Suite")
    failures = 0
    
    # Test 1: Verify empty input handling
    try
        check_conflicts(Mission(Trajectory([], "empty", 0,0), []))
        failures += 1
        @warn "Failed to catch empty trajectory"
    catch e
        println("✓ Empty input validation passed")
    end
    
    # Test 2: Minimum safe distance
    safe_case = create_no_conflict_case()
    safe_case.primary.waypoints[1] = Waypoint(0,0,10,now())
    safe_case.others[1].waypoints[1] = Waypoint(5.01,0,10,now())
    @assert isempty(check_conflicts(safe_case)) "Safe distance failed"
    
    # Add more validation checks...
    
    return failures == 0
end