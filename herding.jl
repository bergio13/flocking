using StaticArrays, Random, LinearAlgebra, Plots
using Base: time
include("flocking.jl")

# ========================
# Dog Structure
# ========================
mutable struct Dog
    position::SVector{2,Float64}
    velocity::SVector{2,Float64}
    target::Union{Nothing,SVector{2,Float64}}
    mode::Symbol  # :chase, :guide, :patrol
end

@kwdef struct HerdingParams
    # Sheep parameters
    separation_weight::Float64 = 0.15
    cohesion_weight::Float64 = 0.4
    alignment_weight::Float64 = 0.3
    max_sheep_speed::Float64 = 0.4
    sheep_radius::Float64 = 1.5

    # Dog parameters
    dog_speed::Float64 = 0.9
    dog_influence_radius::Float64 = 2.0
    dog_repulsion_strength::Float64 = 0.8
    patrol_margin::Float64 = 1.0

    # World parameters
    world_size::Float64 = 20.0
    pen_center::SVector{2,Float64} = SVector(17.0, 17.0)
end

# ========================
# Update Velocity Functions
# ========================
function update_sheep!(sheep::Sheep, flock::Vector{Sheep}, dogs::Vector{Dog}, p::HerdingParams)
    sep = separation(sheep, flock, p.sheep_radius)
    coh = cohesion(sheep, flock, p.sheep_radius)
    ali = alignment(sheep, flock, p.sheep_radius)
    dog_force = dog_repulsion(sheep, dogs, p)

    sheep.velocity += sep * p.separation_weight + coh * p.cohesion_weight + ali * p.alignment_weight + dog_force
    speed = norm(sheep.velocity)
    speed > p.max_sheep_speed && (sheep.velocity = (sheep.velocity / speed) * p.max_sheep_speed)
end

# Dog modes:
# :herd - Guide sheep toward pen
# :fetch - Chase and collect scattered sheep
# :patrol - Monitor boundaries
# :block - Position between sheep and boundaries

# Dog update function
function update_dog!(dog::Dog, flock::Vector{Sheep}, dogs::Vector{Dog}, p::HerdingParams)
    # Calculate flock center for strategic positioning
    flock_center = calculate_flock_center(flock)

    # Update dog target and behavior based on mode
    if dog.mode == :fetch
        dog_fetch_behavior!(dog, flock, p)
    elseif dog.mode == :herd
        dog_herd_behavior!(dog, flock, flock_center, p)
    elseif dog.mode == :patrol
        dog_patrol_behavior!(dog, flock, p)
    elseif dog.mode == :block
        dog_block_behavior!(dog, flock, flock_center, p)
    end

    # Ensure dog doesn't exceed maximum speed
    speed = norm(dog.velocity)
    if speed > p.dog_speed
        dog.velocity = normalize(dog.velocity) * p.dog_speed
    end

    # Avoid collisions with other dogs
    avoid_other_dogs!(dog, dogs, p)
end

# ========================
# Dog Behavior Functions
# ========================

# Calculate the center of the flock
function calculate_flock_center(flock::Vector{Sheep})
    if isempty(flock)
        return SVector(0.0, 0.0)
    end

    sum_pos = SVector(0.0, 0.0)
    for sheep in flock
        sum_pos += sheep.position
    end

    return sum_pos / length(flock)
end

# Fetch behavior: Find outlier sheep and bring them back
function dog_fetch_behavior!(dog::Dog, flock::Vector{Sheep}, p::HerdingParams)
    if isempty(flock)
        return
    end

    # Find the sheep furthest from the flock center
    flock_center = calculate_flock_center(flock)
    furthest_sheep = nothing
    max_dist = -1.0

    for sheep in flock
        dist_to_center = norm(sheep.position - flock_center)
        dist_to_pen = norm(sheep.position - p.pen_center)

        # Prioritize sheep far from center and far from pen
        combined_dist = dist_to_center + 0.2 * dist_to_pen

        if combined_dist > max_dist
            max_dist = combined_dist
            furthest_sheep = sheep
        end
    end

    if !isnothing(furthest_sheep)
        # Position behind the sheep relative to the flock center
        direction_to_center = normalize(flock_center - furthest_sheep.position)
        target_position = furthest_sheep.position - direction_to_center * p.dog_influence_radius

        # Move toward the target position
        direction = target_position - dog.position
        dog.velocity = normalize(direction) * p.dog_speed
        dog.target = target_position
    end
end

# Herding behavior: Guide sheep toward pen
function dog_herd_behavior!(dog::Dog, flock::Vector{Sheep}, flock_center::SVector{2,Float64}, p::HerdingParams)
    if isempty(flock)
        return
    end

    # Direction from flock center to pen
    direction_to_pen = normalize(p.pen_center - flock_center)

    # Position behind the flock opposite to the pen direction
    ideal_position = flock_center - direction_to_pen * p.dog_influence_radius * 1.2

    # Calculate distance to ideal position
    dist_to_ideal = norm(dog.position - ideal_position)

    # If dog is far from ideal position, move toward it
    if dist_to_ideal > p.dog_influence_radius * 0.3
        direction = ideal_position - dog.position
        dog.velocity = normalize(direction) * p.dog_speed
        dog.target = ideal_position
    else
        # When in position, make small movements to encourage sheep movement
        # Create a zigzag pattern perpendicular to herding direction
        perp_dir = SVector(-direction_to_pen[2], direction_to_pen[1])
        zigzag = sin(time()) * perp_dir * p.dog_influence_radius * 0.3

        target_position = ideal_position + zigzag
        direction = target_position - dog.position
        dog.velocity = normalize(direction) * p.dog_speed
        dog.target = target_position
    end
end

# Patrol behavior: Guard boundaries to prevent sheep escape
function dog_patrol_behavior!(dog::Dog, flock::Vector{Sheep}, p::HerdingParams)
    # If no patrol target or close to current target, select a new one
    if isnothing(dog.target) || norm(dog.position - dog.target) < p.dog_influence_radius * 0.5
        # Create patrol points along the boundaries with some variation
        margin = p.patrol_margin

        # Choose a random point along one of the four boundaries
        side = rand(1:4)
        if side == 1  # Left boundary
            x = margin
            y = rand() * p.world_size
        elseif side == 2  # Top boundary
            x = rand() * p.world_size
            y = p.world_size - margin
        elseif side == 3  # Right boundary
            x = p.world_size - margin
            y = rand() * p.world_size
        else  # Bottom boundary
            x = rand() * p.world_size
            y = margin
        end

        sheep_near_boundary = find_sheep_near_boundary(flock, p)
        if !isnothing(sheep_near_boundary)
            # Move toward the sheep near the boundary
            dog.target = sheep_near_boundary.position
        else
            dog.target = SVector(x, y) # Set new patrol target
        end
    end


    direction = dog.target - dog.position
    dog.velocity = normalize(direction)
end

# Blocking behavior: Position between sheep and escape routes
function dog_block_behavior!(dog::Dog, flock::Vector{Sheep}, flock_center::SVector{2,Float64}, p::HerdingParams)
    if isempty(flock)
        return
    end

    # Find direction where most sheep are heading
    avg_direction = SVector(0.0, 0.0)
    for sheep in flock
        avg_direction += normalize(sheep.velocity)
    end
    avg_direction = normalize(avg_direction)

    # Calculate potential escape point based on flock movement direction
    escape_point = flock_center + avg_direction * p.world_size * 0.25

    # Adjust escape point to be at the nearest boundary if needed
    escape_point = SVector(
        clamp(escape_point[1], 0.0, p.world_size),
        clamp(escape_point[2], 0.0, p.world_size)
    )

    # Position between flock center and escape point
    ideal_position = flock_center + normalize(escape_point - flock_center) * p.dog_influence_radius * 1.5

    # Move to ideal position
    direction = ideal_position - dog.position
    dog.velocity = normalize(direction) * p.dog_speed
    dog.target = ideal_position
end

# Find sheep that are dangerously close to boundaries
function find_sheep_near_boundary(flock::Vector{Sheep}, p::HerdingParams)
    danger_margin = p.patrol_margin * 2
    closest_sheep = nothing
    min_boundary_dist = p.world_size

    for sheep in flock
        # Calculate distance to nearest boundary
        boundary_dist = min(
            sheep.position[1],           # Distance to left boundary
            sheep.position[2],           # Distance to top boundary
            p.world_size - sheep.position[1],  # Distance to right boundary
            p.world_size - sheep.position[2]   # Distance to bottom boundary
        )

        if boundary_dist < danger_margin && boundary_dist < min_boundary_dist
            min_boundary_dist = boundary_dist
            closest_sheep = sheep
        end
    end

    return closest_sheep
end


# Avoid collisions with other dogs
function avoid_other_dogs!(dog::Dog, dogs::Vector{Dog}, p::HerdingParams)
    avoidance = SVector(0.0, 0.0)
    avoidance_radius = p.dog_influence_radius * 0.9

    for other_dog in dogs
        dog === other_dog && continue

        delta = dog.position - other_dog.position
        distance = norm(delta)

        if distance < avoidance_radius
            # Stronger avoidance for closer dogs
            avoidance += normalize(delta) * (avoidance_radius - distance) / avoidance_radius
        end
    end

    # Only apply significant avoidance if needed
    if norm(avoidance) > 0.2
        dog.velocity += normalize(avoidance) * p.dog_speed
    end
end

# ========================
# Sheep Behavior Rules
# ========================
function dog_repulsion(sheep::Sheep, dogs::Vector{Dog}, p::HerdingParams)
    force = SVector(0.0, 0.0)
    for dog in dogs
        delta = sheep.position - dog.position
        distance = norm(delta)
        distance < p.dog_influence_radius && (force += normalize(delta) * p.dog_repulsion_strength / (distance + 1e-5))
    end
    return force
end

# ========================
# Initialization
# ========================
function create_dogs(n::Int, p::HerdingParams; modes=[:herd, :fetch, :patrol, :block])
    [Dog(SVector(rand() * p.world_size, rand() * p.world_size),
        SVector(randn(), randn()),
        nothing,
        modes[rand(1:end)]
    ) for _ in 1:n]
end

# ========================
# Simulation Loop
# ========================
function simulate!(flock::Vector{Sheep}, dogs::Vector{Dog}, p::HerdingParams, steps::Int)
    anim = @animate for step in 1:steps
        # Update dogs
        for dog in dogs
            update_dog!(dog, flock, dogs, p)
            update_position!(dog)
            apply_boundary!(dog, p.world_size)
        end

        # Update sheep
        for sheep in flock
            update_sheep!(sheep, flock, dogs, p)
            update_position!(sheep)
            apply_boundary!(sheep, p.world_size)
        end

        # Plot
        plot_flock(flock, dogs, step, p)
    end
    gif(anim,
        #"images/herding.gif",
        fps=15)
end

# ========================
# Visualization
# ========================
function plot_flock(flock::Vector{Sheep}, dogs::Vector{Dog}, step::Int, p::HerdingParams)
    plt = plot(xlim=(0, p.world_size), ylim=(0, p.world_size),
        title="Herding Simulation (Step $step)", aspect_ratio=:equal,
        legend=:topleft)

    # Plot sheep
    sheep_x = [s.position[1] for s in flock]
    sheep_y = [s.position[2] for s in flock]
    scatter!(sheep_x, sheep_y, color=:skyblue, label="Sheep", alpha=0.8)
    quiver!(sheep_x, sheep_y, quiver=([s.velocity[1] for s in flock], [s.velocity[2] for s in flock]),
        color=:skyblue, alpha=0.6)

    # Plot dogs with different colors based on mode
    for dog in dogs
        color = if dog.mode == :herd
            :orangered
        elseif dog.mode == :fetch
            :purple
        elseif dog.mode == :patrol
            :darkgreen
        else # :block
            :gold
        end

        scatter!([dog.position[1]], [dog.position[2]], color=color,
            markershape=:utriangle, label="$(dog.mode)")

        quiver!([dog.position[1]], [dog.position[2]],
            quiver=([dog.velocity[1]], [dog.velocity[2]]),
            color=color, linewidth=2)

        # Show target if available
        if !isnothing(dog.target)
            scatter!([dog.target[1]], [dog.target[2]], color=color, markershape=:xcross,
                markersize=4, alpha=0.5, label=false)
            plot!([dog.position[1], dog.target[1]], [dog.position[2], dog.target[2]],
                color=color, linestyle=:dash, alpha=0.3, label=false)
        end
    end

    # Plot pen 
    circle_points = [(p.pen_center[1] + 2 * cos(θ), p.pen_center[2] + 2 * sin(θ)) for θ in 0:0.1:2π]
    x_circle = [p[1] for p in circle_points]
    y_circle = [p[2] for p in circle_points]
    plot!(x_circle, y_circle, label="Pen")

    plt
end

# ========================
# Run Simulation
# ========================
params = HerdingParams()
flock = create_flock(50, params.world_size)
dogs = create_dogs(1, params, modes=[:fetch])
simulate!(flock, dogs, params, 200)