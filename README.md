# Tracking controller for Autonomous Car
This repository presents the implementation for autonomous car tracking with pre-drive path.

## Structure
```mermaid
flowchart LR
    A[Path] --> B(Trajectory generation)
    B --> C{Local trajectory generation}
    C --> D[Controller]
    D --> E[Autonomous Car]
    E --> C
```

## Prequisites

Install `clang-format`
```
sudo apt install ninja-build
sudo apt install build-essential
sudo apt install clang-format-11
sudo ln -s /usr/bin/clang-format-11 /usr/local/bin/clang-format
```

Install `acados`, please follow the following [instructions](https://docs.acados.org/installation/index.html)

`Pre-commit`: Check code format before push
```
pip install pre-commit
pre-commit install
pre-commit autoupdate
```

## Build and run

Build
```
./scripts/build.sh
```

Run (build step already included in run script)
```
./scripts/run_example_<name>.sh
```
Visualize results
```
python3 python/visual_path.py       # Plot tracking results
python3 python/visual_animation.py  # Plot animation
```
## Results
<img src="data/video.gif" alt="" width="60%"/><img src="data/tracking.png" alt="" width="40%"/>