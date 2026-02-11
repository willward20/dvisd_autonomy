# Contributing to DVISD Autonomy
Thank you for contributing to the DVISD Autonomy repository! This document provides guidelines for contributions.

## Creating a New Branch
When developing a new feature, always create a new branch off of `main`. Push changes directly to your new branch (never to `main`). Contribute to `main` by creating pull requests. 

```
# First, update your local main branch.
git checkout main
git pull origin main

# Create your feature branch off of main
git checkout -b your-feature-name
```

## Creating a Pull Request

Before creating a pull request into `main`, checkout the latest changes to `main` and resolve any conflicts.

```
# Pull recent changes from main.
git checkout main
git pull origin main

# Merge main into your branch.
git checkout your-feature-name
git merge main

# Push your branch to origin.
git push origin your-feature-name
```

On GitHub, create a new pull request from `your-feature-name` into `main`. Include a description of what changed., and add `willward20` as a reviewer (optional). 


## Code Structure
```
/dvisd_autonomy      # Core Python source code 
    control.py       # Encapsulates RC car motor control
    ...
    utils.py         # Utility functions (e.g., YAML loading)

/config              # Vehicle-specific configuration parameters
    cardinal1.yaml   # Large vehicle
    cardinal2.yaml   # Small vehicle

/scripts             # Top-level scripts designed for students
    control_example.py
    ...
```

## Key Principles

### Expose only high-level interfaces

Source code should provide simple, meaningful methods for students. For example, `control.py` exposes only `forward()`, `turn()`, and `stop()`, hiding low-level hardware details.

### Separate configuration from code

Load vehicle parameters via `load_yaml()` so the same code works across multiple vehicles (see `control_example.py`).

### Use scripts for top-level orchestration

Scripts should call only high-level interfaces. Provide a template for students with a clear section for their code. Organize source code in `dvisd_autonomy` so the top-level API is simple.

Top-level script template:

```
def main():

    # Initialize system
    print("Setting things up...")

    # ---- Students write their code here ----


    # ---------------------------------------

    # Shutdown system
    print("Shutting things down")

if __name__ == "__main__":
    main()
```

