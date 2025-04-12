using Agents, LinearAlgebra, BenchmarkTools, Random

@agent struct Sheep(ContinuousAgent{2,Float64})
    const speed::Float64
    const cohesion_weight::Float64
    const separation::Float64
    const separation_weight::Float64
    const alignment_weight::Float64
    const visual_distance::Float64
end

function flocking_model(rng, extent, n_birds, visual_distance;
    speed=1.5, cohesion_weight=0.1, separation=5.0, separation_weight=0.25,
    alignment_weight=0.1, spacing=visual_distance / 1.5,)
    space2d = ContinuousSpace(extent; spacing)
    model = StandardABM(Sheep, space2d; agent_step!, rng, container=Vector)
    for n in 1:n_birds
        vel = SVector{2}(rand(abmrng(model)) * 2 - 1 for _ in 1:2)
        add_agent!(model, vel, speed, cohesion_weight, separation,
            separation_weight, alignment_weight, visual_distance)
    end
    return model
end

function agent_step!(sheep, model)
    neighbor_agents = nearby_agents(sheep, model, sheep.visual_distance)
    N = 0
    match = separate = cohere = SVector{2}(0.0, 0.0)
    for neighbor in neighbor_agents
        N += 1
        heading = get_direction(sheep.pos, neighbor.pos, model)
        cohere += heading
        match += neighbor.vel
        if sum(heading .^ 2) < sheep.separation^2
            separate -= heading
        end
    end
    cohere *= sheep.cohesion_weight
    separate *= sheep.separation_weight
    match *= sheep.alignment_weight
    sheep.vel += (cohere + separate + match) / max(N, 1)
    sheep.vel /= norm(sheep.vel)
    move_agent!(sheep, model, sheep.speed)
end

# Initialize the model
model = flocking_model(
    Random.MersenneTwister(42),
    (100, 100),
    10000,
    5.0,
    speed=1.5,
    cohesion_weight=0.1,
    separation=5.0,
    separation_weight=0.25,
    alignment_weight=0.1,
)

# Measure performance of the model
bench = @benchmarkable run!($model, 100)
tune!(bench)
results = run(bench)
