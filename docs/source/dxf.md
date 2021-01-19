# DXF Export Guide
There are at two types of files for each wing segment created in MachUpX. How each file is used is described as follows. Alternately, a video tutorial for the whole craft can be found [here](https://youtu.be/nUwv6XcJdsI). The files used were generated for a traditional aircraft with a main wing, horizontal, and vertical stabilizers.

## Import 3D .dxf Airfoils
The first step for each wing is to import airfoils for the wing segment you will work on. This file will be titled after the wing segment to be modeled, with the file name terminating in "_AF". This .dxf file can be inserted on any plane. Note the plane must be selected prior to selecting Insert > DXF/DWG...

![](dxf_figs/00_insert_on_plane.png)

In this case, the file name is traditional_airplane_h_stab_left_AF.dxf.

![](dxf_figs/01_select_file.png)

This file must be imported as a 3D file. You can select "Finish" to complete importing the airfoils. However, if you need to change the import units (if the default is not desired), you can select "Next".

![](dxf_figs/02_DXF_Import.png)

### Changing Units in SolidWorks

Here a dropdown menu can be used to select a different import unit.

![](dxf_figs/03_units_change.png)

### Create 3D Sketch of Airfoils

After selecting all imported airfoil splines, open a new 3D sketch, and select "Convert Entities". This will make closed shapes for lofting.

![](dxf_figs/04_splines_to_3d_sketch.png)

You can now exit the 3D sketch.

![](dxf_figs/05_exit_3d_sketch.png)

Hiding the imported airfoil curves may help with later visibility and lofting.

![](dxf_figs/06_hide_af_curves.png)

## Import 3D .dxf Guide Curves
Next, you must import the guide curves dxf file exactly as you imported the airfoils file. The guide curves file will have the same wing segment name, with the file name terminating in "_GC". This .dxf file can also be inserted on any plane. 

## Loft
With the airfoils 3D sketch and the guide curves imported, you are ready for lofting.

![](dxf_figs/07_open_loft.png)

Select the airfoils in order. If a selection menu pops up, use the closed shape option.

![](dxf_figs/08_loft_select_airfoils.png)

Now, select the guide curves.

![](dxf_figs/09_loft_select_guide_curves.png)

Now, finish the loft.

![](dxf_figs/10_tada.png)

Success! Good Luck!

## Common Errors

### Problems with Guide Curves
Often, with a complex shape, two guide curves are not enough to constrain the wing geometry through the loft. For more complex lofts, we recommend several guide curves. The number of guide curves can be increased in the .dxf export function. It should be noted that SolidWorks begins to slow down significantly after around forty guide curves. We recommend between four and twenty if two is insufficient. If the loft does not complete, sometimes unselecting certain guide curves may help.

We also recommend an even number of guide curves. This ensures a guide curve is placed at the trailing edge and at the leading edge. Otherwise, the nose of the wing may not loft as desired.

### Zero Chord Locale

If there is at some point along the span your chord is zero, you may notice some errors. When you import the airfoils dxf file for this wing segment you will get an error stating N entities could not be imported (It will be the same as the number of guide curves you have for the dxf export).

![](dxf_figs/11_import_error.png)

Import the guide curves prior to creating the airfoils 3D sketch. with the 3D sketch open after adding the airfoils, add a point where the chord is zero along the guide curves.

![](dxf_figs/12_3D_point.png)

You can use this point similar to the airfoils during the loft. You may notice the absence of an example loft after selecting the point. Once you select all the guide curves, the example loft will appear.

![](dxf_figs/13_tada_pointy.png)

Success! Good Luck!