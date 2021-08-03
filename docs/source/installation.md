# Installation

## Prerequisites

Version 2 of MachUpX is dependent upon the AirfoilDatabase package, also distributed by the USU AeroLab. The source code for AirfoilDatabase can be found on our [Github page](https://github.com/usuaero/AirfoilDatabase). Before installing MachUpX, you will need to download and install AirfoilDatabase manually.

## Getting Python

If you do not have Python installed on your machine, it can be downloaded from a number of locations. We use [https://www.anaconda.com/distribution/](https://www.anaconda.com/distribution/). Please be sure you have Python 3.6 or later.

## Getting the Source Code

You can either download the source as a ZIP file and extract the contents, or clone the MachUpX repository using Git. If your system does not already have a version of Git installed, you will not be able to use this second option unless you first download and install Git. If you are unsure, you can check by typing `git --version` into a command prompt.

### Downloading source as a ZIP file (Not recommended)

1. Open a web browser and navigate to [https://github.com/usuaero/MachUpX](https://github.com/usuaero/MachUpX)
2. Make sure the Branch is set to `Master`
3. Click the `Clone or download` button
4. Select `Download ZIP`
5. Extract the downloaded ZIP file to a local directory on your machine

### Cloning the Github repository (Recommended)

1. From the command prompt navigate to the directory where MachUpX will be installed. Note: git will automatically create a folder within this directory called MachUpX. Keep this in mind if you do not want multiple nested folders called MachUpX.
2. Execute

    $ git clone https://github.com/usuaero/MachUpX

We recommend cloning from the repository, as this will allow you to most easily download and install periodic updates (because we will always be updating MachUpX!). This can be done using the following command

    $ git pull

Please reinstall after pulling from the repository.

## Installing

Once you have the source code downloaded, navigate to the root (MachUpX/) directory and execute

    $ pip install .

Please note that any time you update the source code (e.g. after executing a git pull), MachUpX will need to be reinstalled by executing the above command.

## Testing the Installation

Once the installation is complete, run

    $ py.test

on Unix or

    > py.test-script.py

on Windows to verify MachUpX is working properly on your machine. Some warnings and depreciation notices are normal.

## FreeCAD for Exporting STEP Files

MachUpX is able to generate CAD models of the airframe data provided using the STEP exchange format (.stp). These can be opened by any CAD package. In order to do this, MachUpX uses the FreeCAD Python API. As such, FreeCAD needs to be installed on your machine for this to work. FreeCAD can be downloaded from [https://www.freecadweb.org/](https://www.freecadweb.org/) or on Linux systems using

    $ sudo apt-get install freecad

You will then need to add the FreeCAD modules to the Python path. This can be done using Conda (comes with Anaconda python). On Linux, you will need to add the following directories using the command `conda-develop`, i.e.

    $ conda-develop /usr/lib/freecad/lib
    $ conda-develop /usr/lib/freecad/bin
    $ conda-develop /usr/lib/freecad/bin/lib
    $ conda-develop /usr/lib/freecad/Ext
    $ conda-develop /usr/lib/freecad/Mod
    $ conda-develop /usr/lib/freecad/Gui

The location of these directories will vary depending on your machine's configuration. When you call the aircraft_export_step() function, you may see `No modules found in /usr/lib/freecad-python3/Mod` or a similar error message in yellow or red text. This means the libraries were found. Any other error message or a STEP file not being generated indicate that you have not properly configured FreeCAD for Python to recognize the FreeCAD modules.