# gate_probe.jl <cfg_dir> "x,y; x,y; ..."   — single-agent lattice polyline → seed-gate report
ENV["GKSwstype"] = "100"
const CFGDIR = ARGS[1]; const PATHSPEC = ARGS[2]
empty!(ARGS); push!(ARGS, CFGDIR)
using Plots, DataStructures, LinearAlgebra, Statistics, Random, Dates, Printf
Random.seed!(42)
const _ROOT = normpath(joinpath(@__DIR__, "..", "..", "..", "..", ".."))
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))
include(joinpath(_ROOT, "planners", "hexspline_cl.jl"))
pts = [Tuple(parse.(Float64, strip.(split(s, ',')))) for s in split(PATHSPEC, ';') if !isempty(strip(s))]
ctrls = [pts]
lms = ACTIVE_SCENARIO.landmarks
covs, _, _, _ = evaluate_joint_discrete(ctrls, lms, 1)
plans = build_obstacle_plan(ctrls, SPLINE_DEGREE)
padded = bspline_pad_controls(ctrls[1]); nctrl = length(ctrls[1])
println("path: $(length(pts)) pts, polyline len $(round(sum(hypot((pts[i+1] .- pts[i])...) for i in 1:length(pts)-1), digits=1))")
srcmap = pad_source_indices(nctrl)
for sf in plans[1]
    sl = seg_face_slacks(sf, padded, nctrl, covs[1])
    m = minimum(sl)
    m < 15.0 || continue
    obs = OBSTACLES[sf.obs]
    Qmv = seg_control_matrix(padded, sf.seg) * sf.Mseg
    ctrl_ids = ntuple(k -> srcmap[sf.seg + k - 1], 4)
    @printf("%s seg %2d (ctrl %s) obs %d face %d normal (%.2f,%.2f) min slack %7.2f   mv-x[%.0f,%.0f] mv-y[%.0f,%.0f]\n",
            m < 0 ? "BREACH" : "  ok  ", sf.seg, string(ctrl_ids), sf.obs, sf.face, obs.A[sf.face,1], obs.A[sf.face,2], m,
            minimum(Qmv[1,:]), maximum(Qmv[1,:]), minimum(Qmv[2,:]), maximum(Qmv[2,:]))
end
println("real curve collisions: ", verify_curve_collisions(ctrls))
println("goal unc: ", round(unc_radius(covs[1][end]), digits=3))
