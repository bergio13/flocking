using StaticArrays, Random, LinearAlgebra, Plots

mutable struct Sheep
    position::SVector{2,Float64}
    velocity::SVector{2,Float64}
end

# Parameters for the flocking simulation
@kwdef struct FlockParams
    separation_weight::Float64 = 0.15
    cohesion_weight::Float64 = 0.4
    alignment_weight::Float64 = 0.3
    max_speed::Float64 = 0.5
    radius::Float64 = 1.5
    world_size::Float64 = 20.0
    noise_intensity::Float64 = 0.5
end


# Update the position of a sheep based on its velocity
function update_position!(sheep::Sheep, delta_t::Float64=1.0)
    sheep.position += sheep.velocity * delta_t
    # Add bounded randomness
    sheep.velocity += SVector(randn(), randn()) * 0.1
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

# Soft boundary conditions
function apply_soft_boundary!(sheep::Sheep, world_size::Float64, boundary_force::Float64=0.5)
    # Repulsive force near boundaries
    if sheep.position[1] < 0.1
        sheep.velocity += SVector(boundary_force, 0.0)
    elseif sheep.position[1] > world_size - 0.1
        sheep.velocity += SVector(-boundary_force, 0.0)
    end
    if sheep.position[2] < 0.1
        sheep.velocity += SVector(0.0, boundary_force)
    elseif sheep.position[2] > world_size - 0.1
        sheep.velocity += SVector(0.0, -boundary_force)
    end
end

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
        update_position!(sheep)
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

    # Plot sheep positions
    plt = scatter(x, y, xlim=(0, p.world_size), ylim=(0, p.world_size),
        title="Flock Step $step", legend=false, aspect_ratio=:equal)

    speeds = [norm(sheep.velocity) for sheep in flock]
    colors = [speed / maximum(speeds) for speed in speeds]  # Normalize speeds for coloring
    quiver!(x, y, quiver=(u, v), color=colors, colormap=:coolwarm, linewidth=0.5)
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
    gif(anim, fps=15)
end



# Usage
params = FlockParams()
flock = create_flock(15, params.world_size)
simulate_and_plot!(flock, 100, params)
