# User Interface
MachUpX is a Python module and so can be used in one of two ways, either through the command line or the Python interpreter. Examples of the interface can be found in the examples/ directory of the source code.

## Command Line
MachUpX is run from the command line using the "-m" option. For example

```python
python -m machupX example_input.json
```

will run the analyses listed under "run" in the file `example_input.json`. The various results are saved as files to be accessed by the user. For creating the input file, see [Input Files](creating_input_files).

## Python Interpreter
MachUpX can also be imported through the Python interpreter and its functionality can then be accessed through the Scene class. For example

```python
import machupX as MX

input_file = "traditional_input.json"

# Initialize scene
my_scene = MX.Scene(input_file)

# Display wireframe of the scene
my_scene.display_wireframe(show_legend=True)

# Solve aerodynamic forces and moments
FM_results = my_scene.solve_forces(dimensional=True, non_dimensional=False, verbose=True)
print(json.dumps(FM_results["traditional_airplane"]["total"], indent=4))

# Trim the aircraft in pitch
trim_state = my_scene.aircraft_pitch_trim(set_trim_state=True, verbose=True)
print(json.dumps(trim_state["traditional_airplane"]))

# Calculate aerodynamic derivatives
derivs = my_scene.aerodynamic_derivatives()
print(json.dumps(derivs["traditional_plane"]))
```

For more information on using the Scene class, see [Scene Class](scene_object).
