using StaticArrays, Random, LinearAlgebra, Plots
using BenchmarkTools

mutable struct Sheep
    position::SVector{2,Float64}
    velocity::SVector{2,Float64}
end

# Parameters for the flocking simulation
@kwdef mutable struct FlockParams
    separation_weight::Float64 = 0.15
    cohesion_weight::Float64 = 0.4
    alignment_weight::Float64 = 0.3
    max_speed::Float64 = 0.4
    radius::Float64 = 2.0
    world_size::Float64 = 20.0
    noise_intensity::Float64 = 0.2
    delta_t::Float64 = 1.0
end


# Update the position of a sheep based on its velocity
function update_position!(agent, delta_t::Float64=1.0, noise_intensity::Float64=0.2)
    agent.position += agent.velocity * delta_t
    # Add bounded randomness
    agent.velocity += SVector(randn(), randn()) * noise_intensity
end

# Separation rule: steer to avoid crowding local flockmates
function separation(sheep::Sheep, flock::Vector{Sheep}, radius::Float64)
    force = SVector(0.0, 0.0)
    for other in flock
        if sheep === other
            continue
        end
        delta = sheep.position - other.position
        distance = norm(delta)
        if distance < radius
            if distance > 0
                force += delta / distance
            else
                force += SVector(randn(), randn())  # Random direction if overlapping
            end
        end
    end
    return force
end

# Cohesion rule: steer towards the center of mass of neighbors
function cohesion(sheep::Sheep, flock::Vector{Sheep}, radius::Float64)
    center = zero(sheep.position)
    count = 0
    for other in flock
        if sheep === other
            continue
        end
        delta = other.position - sheep.position
        distance = norm(delta)
        if distance < radius
            center += other.position
            count += 1
        end
    end
    count == 0 && return zero(center)
    center /= count
    direction = center - sheep.position
    norm_dir = norm(direction)
    norm_dir > 0 ? direction / norm_dir : zero(direction)
end

# Alignment rule: steer towards the average heading of neighbors
function alignment(sheep::Sheep, flock::Vector{Sheep}, radius::Float64)
    avg_vel = zero(sheep.velocity)
    count = 0
    for other in flock
        if sheep === other
            continue
        end
        delta = other.position - sheep.position
        distance = norm(delta)
        if distance < radius
            avg_vel += other.velocity
            count += 1
        end
    end
    count == 0 && return zero(avg_vel)
    avg_vel /= count
    desired = avg_vel - sheep.velocity
    norm_desired = norm(desired)
    norm_desired > 0 ? desired / norm_desired : zero(desired)
end


# Update the velocity of a sheep based on the flocking rules
function update_velocity!(sheep::Sheep, flock::Vector{Sheep}, p::FlockParams)
    sep = separation(sheep, flock, p.radius)
    coh = cohesion(sheep, flock, p.radius)
    ali = alignment(sheep, flock, p.radius)
    sheep.velocity += sep * p.separation_weight + coh * p.cohesion_weight + ali * p.alignment_weight

    # Speed limiting
    speed = norm(sheep.velocity)
    if speed > p.max_speed
        sheep.velocity = (sheep.velocity / speed) * p.max_speed
    end
end


# Toroidal boundary conditions
#function apply_boundary!(sheep::Sheep, size::Float64)
#    sheep.position = mod.(sheep.position, size)
#end

function apply_boundary!(agent, world_size)
    for i in 1:2
        if agent.position[i] < 0 || agent.position[i] > world_size
            agent.velocity = setindex(agent.velocity, -agent.velocity[i], i)
            agent.position = setindex(agent.position, clamp(agent.position[i], 0.0, world_size), i)
        end
    end
end

# Create a flock of n sheep at random positions
function create_flock(n::Int, world_size=1.0)
    [Sheep(SVector(rand() * world_size, rand() * world_size),
        SVector(randn() * 0.1, randn() * 0.1)) for _ in 1:n]
end


# Update the flock positions
function update_flock!(flock::Vector{Sheep}, p::FlockParams)
    for sheep in flock
        update_velocity!(sheep, flock, p)
        update_position!(sheep, p.delta_t, p.noise_intensity)
        apply_boundary!(sheep, p.world_size)
    end
end

# Plotting function for the flock
function plot_flock(flock::Vector{Sheep}, step::Int, p::FlockParams)
    # Extract positions
    x = [sheep.position[1] for sheep in flock]
    y = [sheep.position[2] for sheep in flock]

    # Extract velocities for arrows
    u = [sheep.velocity[1] for sheep in flock]
    v = [sheep.velocity[2] for sheep in flock]

    # Adjust arrow lengths based on world size
    u = u .* p.world_size / 10
    v = v .* p.world_size / 10

    # Plot sheep positions
    plt = scatter(x, y, xlim=(0, p.world_size), ylim=(0, p.world_size),
        title="Flock Step $step", legend=false, aspect_ratio=:equal)

    speeds = [norm(sheep.velocity) for sheep in flock]
    quiver!(x, y, quiver=(u, v), color=speeds, colormap=:coolwarm, linewidth=0.5)
    # Plot boundaries
    plot!([0, p.world_size, p.world_size, 0, 0], [0, 0, p.world_size, p.world_size, 0],
        color=:black, linewidth=2, linestyle=:dash, legend=false)

    display(plt)
end

# Simulation function with plotting
function simulate_and_plot!(flock::Vector{Sheep}, steps::Int, p::FlockParams)
    anim = @animate for step in 1:steps
        update_flock!(flock, p)
        plot_flock(flock, step, p)
    end
    gif(anim,
        #"images/flock.gif", 
        fps=15)
end


# Usage
function run()
    params = FlockParams(world_size=20.0)
    flock = create_flock(100, params.world_size)
    simulate_and_plot!(flock, 100, params)
end

# Comment the follwing line when running herd.jl
#run()

function benchmark_simulation(steps::Int=100, n_agents::Int=10000)
    params = FlockParams(world_size=1000.0)
    flock = create_flock(n_agents, params.world_size)

    @btime begin
        local f = deepcopy($flock)
        for _ in 1:$steps
            update_flock!(f, $params)
        end
    end
end

benchmark_simulation()