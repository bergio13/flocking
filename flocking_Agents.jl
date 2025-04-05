using Agents
using Random, LinearAlgebra
using CairoMakie
using BenchmarkTools


@agent struct Sheep(ContinuousAgent{2,Float64})
    speed::Float64
    cohesion_weight::Float64
    separation::Float64
    separation_weight::Float64
    alignment_wieght::Float64
    visual_distance::Float64
end

const EXTENT = (98, 98)

function initialize_model(;
    n_sheeps=1000,
    speed=1.5,
    cohesion_weight=0.1,
    separation=2.0,
    separation_weight=0.25,
    alignment_wieght=0.04,
    visual_distance=5.0,
    extent=(100, 100),
    seed=42,
)
    space2d = ContinuousSpace(extent; spacing=visual_distance)
    rng = Random.MersenneTwister(seed)

    model = StandardABM(Sheep, space2d; rng, agent_step!)
    for _ in 1:n_sheeps
        vel = rand(abmrng(model), SVector{2}) * 2 .- 1
        add_agent!(
            model,
            vel,
            speed,
            cohesion_weight,
            separation,
            separation_weight,
            alignment_wieght,
            visual_distance,
        )
    end
    return model
end

function reflect!(sheep, extent)
    x_max, y_max = extent
    x, y = sheep.pos
    vx, vy = sheep.vel

    if x < 2 || x > x_max
        vx = -vx
        x = clamp(x, 2, x_max)
    end
    if y < 2 || y > y_max
        vy = -vy
        y = clamp(y, 2, y_max)
    end

    sheep.pos = SVector(x, y)
    sheep.vel = SVector(vx, vy)
end

function agent_step!(sheep, model)
    # Obtain the ids of neighbors within the sheep's visual distance
    neighbor_ids = nearby_ids(sheep, model, sheep.visual_distance)
    N = 0
    match = separate = cohere = (0.0, 0.0)
    # Calculate behaviour properties based on neighbors
    for id in neighbor_ids
        N += 1
        neighbor = model[id].pos
        heading = get_direction(sheep.pos, neighbor, model)

        # `cohere` computes the average position of neighboring birds
        cohere = cohere .+ heading
        if euclidean_distance(sheep.pos, neighbor, model) < sheep.separation
            # `separate` repels the sheep away from neighboring birds
            separate = separate .- heading
        end
        # `match` computes the average trajectory of neighboring birds
        match = match .+ model[id].vel
    end
    N = max(N, 1)
    # Normalise results based on model input and neighbor count
    cohere = cohere ./ N .* sheep.cohesion_weight
    separate = separate ./ N .* sheep.separation_weight
    match = match ./ N .* sheep.alignment_wieght
    # Compute velocity based on rules defined above
    sheep.vel = (sheep.vel .+ cohere .+ separate .+ match) ./ 2
    sheep.vel = sheep.vel ./ norm(sheep.vel)
    # Move sheep according to new velocity and speed
    move_agent!(sheep, model, sheep.speed)
    # Apply reflective boundary conditions
    #reflect!(sheep, EXTENT)
end

model = initialize_model()
@btime run!($model, 100)

#const sheep_polygon = Makie.Polygon(Point2f[(-1, -1), (2, 0), (-1, 1)])
#
#function sheep_maker(s::Sheep)
#    φ = atan(s.vel[2], s.vel[1]) #+ π/2 + π
#    rotate_polygon(sheep_polygon, φ)
#end
#
#model = initialize_model()
#abmvideo(
#    "images/flocking.mp4", model;
#    agent_marker=sheep_maker,
#    agent_size=0.5,
#    framerate=15, frames=100,
#    title="Flocking"
#)



