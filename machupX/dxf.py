"""

This file writes a .dxf file from given 2d xyy value-splines.
These lines are received as three 2D arrays. one is for x coordinates, one is for y coordinates, one is for z coordinates.
Each x array index is an array of x-coordinates for a spline, to be reported in conjunction with the same index of the y array and the z array.
"""

import numpy as np
from datetime import datetime as dt
import os

def dxf_line(filename,x,y,z,file_info):

    # create file
    f = open(filename + ".dxf", "w+")

    # run through file info and input it into the file
    for info in file_info:
        f.write("999\n" + info + "\n")

    # write file creation time
    time = str(dt.now())
    f.write("999\nfile made " + time + "\n")
    
    # write dxf file opening statement for line
    f.write("0\nSECTION\n2\nENTITIES\n")

    # determine the number of compound lines to create
    num_complines = x.shape[0]

    # if the y and x compound line counts are not equal in number, throw error
    if y.shape[0] != num_complines:
        raise ValueError('y array must have same number of compound lines as x array')
    
    # if the z and x compound line counts are not equal in number, throw error
    if z.shape[0] != num_complines:
        raise ValueError('z array must have same number of compound lines as x array')
    
    # cycle through each compound line
    for i in range(num_complines):

        # determine the number of lines in the current compound line
        num_lines = x[i].shape[0] - 1

        # if the y and x line points are not equal in number, throw error
        if y[i].shape[0] != num_lines + 1:
            raise ValueError('y array compound line %d must have same number of points as x array' % i)

        # if the z and x line points are not equal in number, throw error
        if z[i].shape[0] != num_lines + 1:
            raise ValueError('z array compound line %d must have same number of points as x array' % i)

        # cycle through line in each compound line
        for j in range(num_lines):

            # write dxf file line opening statement
            f.write("0\nLINE\n8\nLayer\n")

            # write line start x coordinate
            f.write("10\n%.16f\n" % x[i][j])

            # write line start y coordinate
            f.write("20\n%.16f\n" % y[i][j])

            # write line start y coordinate
            f.write("30\n%.16f\n" % z[i][j])

            # write line end x coordinate
            f.write("11\n%.16f\n" % x[i][j+1])

            # write line end y coordinate
            f.write("21\n%.16f\n" % y[i][j+1])

            # write line end y coordinate
            f.write("31\n%.16f\n" % z[i][j+1])

            # write line color (always black=0) # blue=5
            f.write("62\n0\n")

    # write dxf file closing statement
    f.write("0\nENDSEC\n0\nEOF\n")
    
    # close file
    f.close()

    # # move file to desktop
    # pm.move_file_to_Desktop(filename + ".dxf")

    return

def dxf_point(filename,x,y,z,file_info):

    # create file
    f = open(filename + ".dxf", "w+")

    # run through file info and input it into the file
    for info in file_info:
        f.write("999\n" + info + "\n")

    # write file creation time
    time = str(dt.now())
    f.write("999\nfile made " + time + "\n")
    
    # write dxf file opening statement for line
    f.write("0\nSECTION\n2\nENTITIES\n")

    # determine the number of compound lines to create
    num_pointsarr = x.shape[0]

    # if the y and x compound line counts are not equal in number, throw error
    if y.shape[0] != num_pointsarr:
        raise ValueError('y array must have same number of points arrays as x array')
    
    # if the z and x compound line counts are not equal in number, throw error
    if z.shape[0] != num_pointsarr:
        raise ValueError('z array must have same number of points arrays as x array')
    
    # cycle through each compound line
    for i in range(num_pointsarr):

        # determine the number of lines in the current compound line
        num_points = x[i].shape[0]

        # if the y and x line points are not equal in number, throw error
        if y[i].shape[0] != num_points:
            raise ValueError('y points array %d must have same number of points as x array' % i)

        # if the z and x line points are not equal in number, throw error
        if z[i].shape[0] != num_points:
            raise ValueError('z points array %d must have same number of points as x array' % i)

        # cycle through line in each compound line
        for j in range(num_points):

            # write dxf file line opening statement
            f.write("0\nPOINT\n8\nLayer\n")

            # write line start x coordinate
            f.write("10\n%.16f\n" % x[i][j])

            # write line start y coordinate
            f.write("20\n%.16f\n" % y[i][j])

            # write line start y coordinate
            f.write("30\n%.16f\n" % z[i][j])

    # write dxf file closing statement
    f.write("0\nENDSEC\n0\nEOF\n")
    
    # close file
    f.close()

    # # move file to desktop
    # pm.move_file_to_Desktop(filename + ".dxf")

    return

def dxf_lwpolyline(filename,x,y,z,file_info):

    # create file
    f = open(filename + ".dxf", "w+")

    # run through file info and input it into the file
    for info in file_info:
        f.write("999\n" + info + "\n")

    # write file creation time
    time = str(dt.now())
    f.write("999\nfile made " + time + "\n")
    
    # write dxf file opening statement for line
    f.write("0\nSECTION\n2\nENTITIES\n")

    # determine the number of compound lines to create
    num_complines = x.shape[0]

    # if the y and x compound line counts are not equal in number, throw error
    if y.shape[0] != num_complines:
        raise ValueError('y array must have same number of compound lines as x array')
    
    # cycle through each compound line
    for i in range(num_complines):

        # determine the number of lines in the current compound line
        num_lines = x[i].shape[0] - 1

        # if the y and x line points are not equal in number, throw error
        if y[i].shape[0] != num_lines + 1:
            raise ValueError('y array compound line %d must have same number of points as x array' % i)

        # cycle through line in each compound line
        for j in range(num_lines):

            # write dxf file line opening statement
            f.write("0\nLWPOLYLINE\n8\nLayer\n")

            # write line start x coordinate
            f.write("10\n%.16f\n" % x[i][j])

            # write line start y coordinate
            f.write("20\n%.16f\n" % y[i][j])

            # write line start z coordinate
            f.write("30\n%.16f\n" % z[i][j])

            # write line end x coordinate
            f.write("11\n%.16f\n" % x[i][j+1])

            # write line end y coordinate
            f.write("21\n%.16f\n" % y[i][j+1])

            # write line end z coordinate
            f.write("31\n%.16f\n" % z[i][j+1])

            # write line color (always black=0) # blue=5
            f.write("62\n0\n")

    # write dxf file closing statement
    f.write("0\nENDSEC\n0\nEOF\n")
    
    # close file
    f.close()

    # # move file to desktop
    # pm.move_file_to_Desktop(filename + ".dxf")

    return

def dxf_spline(filename,x,y,z,file_info):

    # create file
    f = open(filename + ".dxf", "w+")

    # run through file info and input it into the file
    for info in file_info:
        f.write("999\n" + info + "\n")

    # write file creation time
    time = str(dt.now())
    f.write("999\nfile made " + time + "\n")
    
    # write dxf file opening statement for line
    f.write("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1015\n0\n")
    f.write("ENDSEC\n0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n")
    f.write("0\nLAYER\n100\nAcDbLayerTableRecord\n2\n0\n0\nENDTAB\n")
    f.write("0\nTABLE\n2\nBLOCK_RECORD\n0\nENDTAB\n0\nENDSEC\n0\n")
    f.write("SECTION\n2\nENTITIES\n")

    # determine the number of splines to create
    num_complines = x.shape[0]

    # if the y and x spline counts are not equal in number, throw error
    if y.shape[0] != num_complines:
        raise ValueError('y array must have same number of splines as x array')

    # if the z and x spline counts are not equal in number, throw error
    if z.shape[0] != num_complines:
        raise ValueError('z array must have same number of splines as x array')
    
    # cycle through each compound line
    for i in range(num_complines):

        # determine the number of points in the current spline
        num_lines = x[i].shape[0]

        # write dxf file spline opening statement
        f.write("0\nSPLINE\n8\nLayer\n100\nAcDbSpline\n71\n3\n74\n4\n")
        # try fixing this. 74 is the number of fit points. fix the number of 
        # fit points...
        # f.write("0\nSPLINE\n8\nLayer\n100\nAcDbSpline\n71\n3\n74\n4\n")

        # if the y and x spline points are not equal in number, throw error
        if y[i].shape[0] != num_lines:
            raise ValueError('y array spline %d must have same number of points as x array' % i)

        # if the z and x spline points are not equal in number, throw error
        if z[i].shape[0] != num_lines:
            raise ValueError('z array spline %d must have same number of points as x array' % i)

        # determine start and end tangents (in components form)
        sx = x[i][1] - x[i][0]
        sy = y[i][1] - y[i][0]
        sz = z[i][1] - z[i][0]

        ex = x[i][-1] - x[i][-2]
        ey = y[i][-1] - y[i][-2]
        ez = z[i][-1] - z[i][-2]

        # cycle through line in each compound line
        for j in range(num_lines):

            # write next spline x coordinate
            f.write("11\n%.16f\n" % x[i][j])

            # write next spline y coordinate
            f.write("21\n%.16f\n" % y[i][j])

            # write next spline z coordinate
            f.write("31\n%.16f\n" % z[i][j])

            # # write line color (always black=0) # blue=5
            # f.write("62\n0\n")
        
        # add in start and end tangents
        # write start spline tangent x-component
        f.write("12\n%.16f\n" % sx)
        # write start spline tangent y-component
        f.write("22\n%.16f\n" % sy)
        # write start spline tangent z-component
        f.write("32\n%.16f\n" % sz)

        # write end spline tangent x-component
        f.write("13\n%.16f\n" % ex)
        # write end spline tangent y-component
        f.write("23\n%.16f\n" % ey)
        # write end spline tangent z-component
        f.write("33\n%.16f\n" % ez)

    # write dxf file closing statement
    f.write("0\nENDSEC\n0\nEOF\n")
    
    # close file
    f.close()

    # # move file to desktop
    # pm.move_file_to_Desktop(filename + ".dxf")

    return

def dxf(filename,x,y,z,file_info = [],geometry = "spline"):

    # create directory if not existant yet
    directory = filename.split("/")[1:]
    path = "/"
    for i in range(len(directory)-1):
        path += directory[i] + "/"
        if not os.path.isdir(path[:-1]):
            os.mkdir(path[:-1])
    
    # if isonelayer:
    if geometry == "spline":
        dxf_spline(filename,x,y,z,file_info)
    elif geometry == "polyline":
        dxf_lwpolyline(filename,x,y,z,file_info)
    elif geometry == "line":
        dxf_line(filename,x,y,z,file_info)
    elif geometry == "point":
        dxf_point(filename,x,y,z,file_info)
    else:
        raise ValueError("geometry must be 'spline', 'polyline', "+\
            "'line', or 'point'" )
    return
