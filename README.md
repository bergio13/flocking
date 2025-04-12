# Flocking ABM Simulation in Julia

This repository contains a simple implementation of a ABM flocking simulation in Julia. The simulation is based on the Boids algorithm, which was introduced by Craig Reynolds in 1986. The algorithm simulates the flocking behavior of birds by defining three simple rules that each boid follows:

1. **Separation**: Boids avoid collisions with their neighbors.
2. **Cohesion**: Boids move towards the center of mass of their neighbors
3. **Alignment**: Boids steer towards the average heading of their neighbors

The simulation is implemented in Julia using the `Plots` package for visualization. The code is organized into the following files:

- `flocking.jl`: Contains the main simulation code, with the implementation of the Boids algorithm
- `herding.jl`: Another type of agent is introduced, which is a herder. The herder interacts with the boids in a different way than the boids interact with each other and has a certain set of possible behaviors it can adopt to influence the boids.

Moreover, there is a folder named `with_Agents.jl` that contains the same simulation but using the `Agents.jl` package. The code is organized into the following files:

- `flocking_Agents.jl`: Contains the main simulation code, with the implementation of the Boids algorithm using the `Agents.jl` package
- `flocking_Agents_perf.jl`: Contains the main simulation code, with the implementation of the Boids algorithm using the `Agents.jl` package, but with significant performance improvements

## Benchmarking

The performance of the simulation is benchmarked using the `BenchmarkTools` package.
The benchmarking results are as follows:

- Flocking Simulation **without** Agents.jl:

  - 1.000 agents

  ```
  BenchmarkTools.Trial: 7 samples with 1 evaluation per sample.
   Range (min … max):  729.264 ms … 881.870 ms  ┊ GC (min … max): 0.00% … 0.00%
   Time  (median):     751.048 ms               ┊ GC (median):    0.00%
   Time  (mean ± σ):   765.534 ms ±  52.560 ms  ┊ GC (mean ± σ):  0.00% ± 0.00%

    █  ██   █ █  █                                              █
    █▁▁██▁▁▁█▁█▁▁█▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁█ ▁
    729 ms           Histogram: frequency by time          882 ms <

   Memory estimate: 283.17 KiB, allocs estimate: 5997.
  ```

  - 10.000 agents

  ```
  BenchmarkTools.Trial: 1 sample with 1 evaluation per sample.
  Single result which took 87.227 s (0.00% GC) to evaluate,
  with a memory estimate of 2.38 MiB, over 68999 allocations.
  ```

- Flocking Simulation **with basic** Agents.jl implementation:

  - 1.000 agents

  ```
  BenchmarkTools.Trial: 2 samples with 1 evaluation per sample.
   Range (min … max):  4.783 s …   4.861 s  ┊ GC (min … max): 8.77% … 7.62%
   Time  (median):     4.822 s              ┊ GC (median):    8.19%
   Time  (mean ± σ):   4.822 s ± 55.278 ms  ┊ GC (mean ± σ):  8.19% ± 0.81%

    █                                                       █
    █▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁█ ▁
    4.78 s         Histogram: frequency by time        4.86 s <

   Memory estimate: 2.99 GiB, allocs estimate: 99390487.
  ```

  - 10.000 agents

  ```
  BenchmarkTools.Trial: 1 sample with 1 evaluation per sample.
  Single result which took 372.205 s (6.15% GC) to evaluate,
  with a memory estimate of 163.84 GiB, over 5487300709 allocations.
  ```

- Flocking Simulation **with improved** Agents.jl implementation:

  - 1.000 agents

  ```
  BenchmarkTools.Trial: 3 samples with 1 evaluation per sample.
   Range (min … max):  1.736 s …    2.062 s  ┊ GC (min … max): 14.87% … 12.91%
   Time  (median):     1.775 s               ┊ GC (median):    15.00%
   Time  (mean ± σ):   1.858 s ± 177.808 ms  ┊ GC (mean ± σ):  14.45% ±  1.49%

    █     █                                                  █
    █▁▁▁▁▁█▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁█ ▁
    1.74 s         Histogram: frequency by time         2.06 s <

   Memory estimate: 2.14 GiB, allocs estimate: 71947087.
  ```

  - 10.000 agents

  ```
  BenchmarkTools.Trial: 1 sample with 1 evaluation per sample.
  Single result which took 37.377 s (12.49% GC) to evaluate,
  with a memory estimate of 37.84 GiB, over 1269617067 allocations.
  ```

## Visualization

### Flocking Simulation

![Flock Simulation](images/flock.gif)

### Herding Simulation

![Herding Simulation](images/herding.gif)
