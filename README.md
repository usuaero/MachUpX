# MachUpX
A new implementation of Warren F. Phillips' numerical lifting-line algorithm, combining the best of MachUp Pro and MachUp_Py.

Written by Cory Goates (graduate student, USU)

NOTE: THIS CODE IS STILL IN DEVELOPMENT. ANY LEVEL OF FUNCTIONALITY IS NOT GUARANTEED AT THIS POINT.

Discussion of the numerical lifting-line algorithm can be found in the following sources:

W. F. Phillips and D. O. Snyder. "Modern Adaptation of Prandtl's
Classic Lifting-Line Theory", Journal of Aircraft, Vol. 37, No. 4
(2000), pp. 662-670.

W. F. Phillips, "Flow over Multiple Lifting Surfaces," Mechanics of
Flight, 2nd ed., Wiley, New Jersey, 2010, pp. 94 -107.

## Features

MachUpX can be run from the command line or imported as a module in a python script.

### Command Line

#### Run Analyses from input file

Simply execute:

`python -m machupX <YOUR_INPUT_FILE>`

### Python API

Here is an example of using the API:

```python
import machupX as MX

input_file = "test_input.json"

# Initialize MachUpX
scene = MX.Scene(input_file)

# Display a wireframe of the aircraft
scene.display_wireframe()

# Determine the forces and moments at the given state
FM = scene.solve_forces()
print(FM)
```

## Documentation

In depth documentation, can be found at [ReadTheDocs](machupx.readthedocs.io)

Specific help with API functions can be found in the docstrings.

## Installation

Navigate to the root directory and execute:

`python setup.py install`.

### Prerequisites

The following Python packages are required:

numpy
scipy

### Getting the Source Code

You can either download the source as a ZIP file and extract the contents, or 
clone the MachUp repository using Git. If your system does not already have a 
version of Git installed, you will not be able to use this second option unless 
you first download and install Git. If you are unsure, you can check by typing 
`git --version` into a command prompt.

#### Downloading source as a ZIP file

1. Open a web browser and navigate to [https://github.com/usuaero/MachUpX](https://github.com/usuaero/MachUpX)
2. Make sure the Branch is set to `Master`
3. Click the `Clone or download` button
4. Select `Download ZIP`
5. Extract the downloaded ZIP file to a local directory on your machine

#### Cloning the Github repository

1. From the command prompt navigate to the directory where MachUp will be installed
2. `git clone https://github.com/usuaero/MachUpX`

## Testing

Testing is accomplished using the pytest module. Run tests using the command:

'python -m pytest test/'

## Support

Contact doug.hunsaker@usu.edu or cory.goates@aggiemail.usu.edu with any questions.

## License

This project is licensed under the MIT license. See LICENSE file for more information. 
