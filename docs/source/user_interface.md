# User Interface
MachUpX can be used in one of two ways, either through the command line or though the Python API.

## Command Line
MachUpX is run from the command line using the "-m" option to the Python interpreter. For example:

```python
python -m machupX example_input.json
```

will run the analyses listed in example_input.json. The various results are saved as files to be accessed by the user.

## Python API
MachUpX can also be imported as a module through the Python interpreter and its functionality can then be accessed through the Scene class. For example:

```python
input_file = "traditional_input.json"

my_scene = MX.Scene(input_file)

my_scene.display_wireframe(show_legend=True)

FM_results = my_scene.solve_forces(dimensional=True, non_dimensional=False, verbose=True)
print(json.dumps(FM_results["traditional_airplane"]["total"], indent=4))

trim_state = my_scene.aircraft_pitch_trim(set_trim_state=True, verbose=True)
print(json.dumps(trim_state["traditional_airplane"]))

derivs = my_scene.aerodynamic_derivatives()
print(json.dumps(derivs["traditional_plane"]))
```
