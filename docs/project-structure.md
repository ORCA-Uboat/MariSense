# MariSense Project Structure

This document provides a comprehensive overview of the MariSense project structure.

## Directory Tree

```
MariSense/
├── README.md                      # Main project documentation
├── LICENSE                        # Project license
├── CONTRIBUTING.md                # Contribution guidelines
├── requirements.txt               # Python dependencies
├── setup.py                       # Package installation script
├── .gitignore                     # Git ignore rules
│
├── docs/                          # Documentation
│   ├── object-detection.md        # Object detection documentation
│   ├── object-tracking.md         # Object tracking documentation
│   ├── odometry.md                # Odometry documentation
│   ├── development-tools.md       # Tools and utilities documentation
│   └── project-structure.md       # This file - project structure reference
│
├── src/                           # Source code
│   └── marisense/                 # Main package
│       ├── __init__.py            # Package initialization
│       ├── loaders/               # Data loaders
│       │   └── __init__.py
│       ├── visualization/         # Visualization tools
│       │   └── __init__.py
│       ├── evaluation/            # Evaluation metrics
│       │   └── __init__.py
│       ├── converters/            # Format converters
│       │   └── __init__.py
│       └── augmentation/          # Data augmentation
│           └── __init__.py
│
├── configs/                       # Configuration files
│   └── README.md                  # Configuration documentation
│
└── assets/                        # Media assets
    └── images/                    # Images for documentation
        ├── platform_real_pic.png  # Platform photograph
        └── platform_details.png   # Sensor configuration diagram
```

## Key Components

### Documentation (`docs/`)

Contains comprehensive documentation for the dataset:
- **object-detection.md**: Object detection task details, annotations, and evaluation
- **object-tracking.md**: Object tracking task details, sequences, and metrics
- **odometry.md**: Odometry task details, sensor data, and ground truth
- **development-tools.md**: Tools and utilities for working with the dataset
- **project-structure.md**: This file - complete project structure reference

### Source Code (`src/marisense/`)

Python package for working with the dataset:
- **loaders/**: Data loading utilities for different tasks
- **visualization/**: Visualization tools for data and results
- **evaluation/**: Evaluation metrics implementation
- **converters/**: Format conversion utilities
- **augmentation/**: Data augmentation tools

### Configuration (`configs/`)

Configuration files directory:
- Contains README with configuration documentation
- Placeholder for task-specific configuration files

### Assets (`assets/`)

Media files for documentation:
- **images/**: Platform images and diagrams
  - `platform_real_pic.png`: Photograph of the data acquisition platform
  - `platform_details.png`: Sensor configuration diagram

## File Descriptions

### Root Level Files

- **README.md**: Main project documentation with quick start guide
- **LICENSE**: MIT License for the project
- **CONTRIBUTING.md**: Guidelines for contributing to the project
- **requirements.txt**: Python package dependencies
- **setup.py**: Package installation and distribution setup
- **.gitignore**: Files and directories to ignore in git

## Usage Patterns

### For Dataset Users

1. Start with [`README.md`](../README.md) for overview
2. Check task-specific documentation in `docs/`
3. Review sensor specifications and data formats
4. Download dataset (instructions in documentation)
5. Use provided Python package for data loading

### For Developers

1. Review [`CONTRIBUTING.md`](../CONTRIBUTING.md) for guidelines
2. Set up development environment with `requirements.txt`
3. Explore `src/marisense/` for package structure
4. Implement custom loaders or evaluation metrics
5. Use `configs/` for experiment configuration
### For Researchers

1. Read the dataset paper (citation in README)
2. Review evaluation protocols in `docs/`
3. Download and explore the dataset
4. Implement and evaluate your methods
5. Cite the dataset in your publications

## Navigation Tips

- Each directory contains a `README.md` with specific documentation
- Documentation files are cross-linked for easy navigation
- Use the main [`README.md`](../README.md) as the entry point
- Refer to [`docs/development-tools.md`](development-tools.md) for tool usage
- Check individual task documentation for detailed specifications

## Maintenance

This structure is designed to be:
- **Modular**: Each component is self-contained
- **Scalable**: Easy to add new features and tasks
- **Documented**: Comprehensive documentation at all levels
- **Testable**: Complete test coverage
- **Extensible**: Clear patterns for contributions

## Questions?

For questions about the project structure:
- Check the relevant README files in each directory
- Review the documentation in `docs/`
- See [`CONTRIBUTING.md`](../CONTRIBUTING.md) for contribution guidelines
- Open an issue on GitHub for clarification
