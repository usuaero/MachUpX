"""
Generic Multivariable Polynomial Fit Using Least Squares Regression

With
Full Control Of Interaction Terms And Constraint Capabilities

This module contains functions to calculate the polynomial coefficients for 
a an arbitrary order polynomial curve fit to a dataset with an arbitrary
number of independent variables.

Routine Listings
-----------------

multivariablePolynomialFit : function for calculating a curve fit to data
    with an arbitrary number of independent variables

multivariablePolynomialFunction : function for calculating a polynomial with
    an arbitrary number of independent variables

multivariableR2 : function calculating the coefficient of determination
    value, or R^2 value, of a polynomial fit of an arbitrary number of
    independent variables

multivariableRMS : function calculating an RMS (root, mean, squared) error
    and a custom RMSN (root, mean, squared, normalized) error where
    normalized means the error is divided by the mean of the absolute value
    of the dependent variables, for a multidimensional polynomial function

compose_j : function used by the multivariable series of functions that
    composes the n values of the independent variables into the counter j,
    this function cal also be used for the nhat values and i

decompose_j : function used by the multivariable seris of functions that
    decomposes the counter j into the different n values for the independent
    variables, this function can also be used for i and the nhat values.
"""
import numpy as np

def multivariablePolynomialFit(Nvec, xx, yy ,interaction=False, sym=[], sym_same=[], sym_diff=[], zeroConstraints=[], constraints=[], percent=False, weighting=None, display=True):
    """
    inputs
    
        Nvec = list with a length V equal to the number of independent
            variables. The ith element values are integers of polynomial
            order of the ith independent variable
        x = numpy matrix of size k by V, where k is the total number of
            points in the dataset. The ith column represents the ith
            independent variable values with the rows representing the
            different data points
        y = list with a length of k with the dependent variable values
        interaction = boolean value with default set to False. This variable
            determines whether or not interaction terms are included in the
            fit function. If set to True, interaction terms up the max order
            for each independent variable are included, i.e. if Nvec = [3,2]
            then the highest interaction term included is x_1^3*x_2^2.
            Specific interaction terms can be omitted using the constraints
            input
        sym = optional list that defaults as an empty list. If used, the
            length should be V and each element should contain a boolean,
            True or False. The ith element determines if the ith independent
            variable is symmetric either even or odd, which is determined by
            the order given in Nvec. This will also remove the cooresponding
            interaction terms if they are enabled.
        sym_same = optional list that defaults as an empty list. If used,
            the entries in the list should be tuples with two integers. The
            integers represent the independent variables that the "same"
            symmetry condition will be applied. The "same" symmetry forces
            all interaction terms
        sym_diff = optional list that defaults as an empty list. 
        zeroConstraints = an optional list that defaults as an empty list.
            Entries in the list contain integer tuples of length V. The
            integer values represent the powers of the independent variables
            whose coefficient will be forced to 0 before the best fit
            calculations are performed, allowing the user to omit specific
            interaction terms or regular polynomial terms
        constraints = an optional list that defaults to an empty list.
            Entries in the list contain tuples of length 2. The first entry 
            is a list of integers that represent the powers of the
            independent variables whose coefficient will then be forced to
            be equal to the second entry in the tuple, which should be a
            float.
        percent = boolean value with default set to False. When set to True
            the least squares is performed on the percent error squared.
            This option should not be used if y contains any zero or near
            zero values, as this might cause a divide by zero error.
        weighting = optional callable function that defaults to None. If
            given, weighting should be a function that takes as arguments:
            x, y, and p where x and y are the independent and dependent
            variables defined above and p is the index representing a
            certain data point. weighting should return a 'weighting factor'
            that determines how important that datapoint is. Returning a '1'
            weights the datapoint normally.
    
    returns
    
        a = list of the polynomial coefficients. 'a' has a length equal to
            the products of the n_vec elements plus one.
            i.e.: (n_vec0+1)*(n_vec1+1)*...
        r2 = the coefficient of determination, also referred to as the R^2
            value. A value of 1 means the polynomial given by 'a' fits the
            data perfectly
    """
    # input variables and setup calculations
    ########################################################################
    x = np.copy(xx)
    y = np.copy(yy)
    # calculate number of dimensions used
    if type(Nvec) != list:
        Nvec = [Nvec]
    V = len(Nvec)
    # calculate the number of points in dataset
    k = len(y)
    # check for inconsistencies in dimensions from input variables
    if len(x.shape) == 1:
        x = np.transpose([x])
    if x.shape[1] != V: raise ValueError('Dimensions for V don\'t match between n_vec and x. Lenght of n_vec and number of columns of x should equal the number of independent variables used, V.')
    if x.shape[0] != k: raise ValueError('Number of rows between x and y don\'t agree! The number of rows should be the total number of points, k, of the dataset.')
    # calculate the length of the coefficient list to be returned
    J = 1
    for n in Nvec:
        J *= n + 1
    # if sym wasn't given, initialize it to False values
    if type(sym) != list: sym = [sym]
    if sym == []:
        sym = [False] * V
    elif len(sym) != V:
        raise ValueError('Length of sym doesn\'t match the number of dimensions, V.')
    # create active list
    ########################################################################
    # set active to empty list
    active = []
    # loop through j values
    for j in range(J):
        # calculate the n values
        n = decompose_j(j, Nvec)
        # check if n is a constraint then continue on to the next j
        if tuple(n) in zeroConstraints: continue
        # check if j is an interaction term and if interactions aren't allowed then continue on to the next j
        if sum(n) != max(n) and not interaction: continue
        # initialize flag variable to false
        flag = False
        # loop through the sym list to find the symmetry constraints
        for count,symm in enumerate(sym):
            # check if flag has been tripped, then continue to the next j if it has
            if flag: break
            # check for a symmetry constraint
            if symm:
                # check if the order of the count-th independent variable is even
                if Nvec[count]%2 == 0:
                    # check if the n value of the count-th independent variable is odd
                    if n[count]%2 == 1:
                        flag = True
                # this else block means the order of the count-th independent variable is odd
                else:
                    # check if the n value of the count-th independent variable is even
                    if n[count]%2 == 0:
                        flag = True
        # if the flag has been tripped, skip to the next j value
        if flag: continue
        # loop through sym_same constraints
        for val in sym_same:
            # check if the n values from both variables given in val are even, then trip flag
            if n[val[0]]%2 == 0 and n[val[1]]%2 == 0:
                flag = True
            # check if the n values from both variables given in val are odd, then trip flap
            if n[val[0]]%2 == 1 and n[val[1]]%2 == 1:
                flag = True
        # loop through sym_diff constraints
        for val in sym_diff:
            # check if the n values from both variables given in val are even and odd, then trip flag
            if n[val[0]]%2 == 0 and n[val[1]]%2 == 1:
                flag = True
            # check if the n values from both variables given in val are odd and even, then trip flap
            if n[val[0]]%2 == 1 and n[val[1]]%2 == 0:
                flag = True
        # if flag hasn't been tripped, append j value onto the active list
        if not flag: active.append(j)
    #create constraints list
    ########################################################################
    con = {}
    for n,val in constraints:
        j = compose_j(n, Nvec)
        con['{}'.format(j)] = val
    #create A matrix and b vector
    ########################################################################
    # initialize A matrix
    A = np.zeros( ( len(active),len(active) ) )
    # initialize b vector
    b = np.zeros( len(active) )
    # loop through i values
    for ii,i in enumerate(active):
        if display: print('calculating A{}j and b{} values, {:.2f}% done'.format(i,i,ii/len(active)*100))
        # calculate the nhat values
        nhat = decompose_j(i, Nvec)
        # loop through the j values
        for jj,j in enumerate(active):
            # calculate the n values
            n = decompose_j(j, Nvec)
            # calcualte Aij entry
            #####################
            
            if str(i) in con:
                if i == j:
                    A[ii,jj] = 1.
                else:
                    A[ii,jj] = 0.
            else:
                # initialize summ to 0
                summ = 0.
                # loop through points in dataset
                for p in range(1,k+1):
                    # initialize product series variable to 1
                    if y[p-1] != None:
                        prod = 1.
                    else:
                        prod = 0.
                    # loop through dimensions
                    for v in range(1,V+1):
                        # multiply the term onto the product series
                        prod *= x[p-1,v-1] ** (n[v-1] + nhat[v-1])
                    #==================================================
                    # add weighting factor
                    if callable(weighting):
                        prod *= weighting(x, y, p-1)
                    #==================================================
                    # add the product series variable to the summation
                    if percent:
                        if y[p-1] != None: summ += prod/abs(y[p-1])
                    else:
                        summ += prod
                # set Aij to the finalized summation
                A[ii,jj] = summ
        # calculate bi entry
        ####################
        if str(i) in con:
            b[ii] = con[str(i)]
        else:
            # initialize summation variable to 0
            summ = 0.
            # loop through points in the dataset
            for p in range(1,k+1):
                # initialize the product series variable to 1
                if y[p-1] != None:
                    prod = 1.
                else:
                    prod = 0.
                # loop through the dimensions
                for v in range(1,V+1):
                    # multiply the term onto the product series
                    prod *= x[p-1,v-1] ** nhat[v-1]
                #==================================================
                # add weighting factor
                if callable(weighting) and y[p-1] != None:
                    prod *= weighting(x, y, p-1)
                #==================================================
                # add the term onto the summation
                if percent:
                    summ += prod
                else:
                    if y[p-1] != None: summ += y[p-1] * prod
            # set bi to the finalized summation
            b[ii] = summ
    #solve Aa=b equation
    ########################################################################
    if display: print('solving the Aa=b equation')
    a = np.linalg.solve(A,b)
    #input the missing 0 coefficients into 'a' so that it can be used with the multidimensional_poly_func
    ########################################################################
    for i in range(J):
        if not i in active:
            a = np.insert(a,i,0.)
            active = np.insert(active,i,0)
    #calculate R^2 value
    ########################################################################
    r = multivariableR2(a, Nvec, x, y)       #r2_2D(a,M,x,y,z)
    #return values
    return a, r

def multivariablePolynomialFunction(a, Nvec, x):
    """
    Multivariable Polynomial Function
    
    inputs:
    
        a = list of the polynomial coefficients. 'a' has a length equal to
            the products of the Nvec elements plus one.
            i.e.: (Nvec0+1)*(Nvec1+1)*...
        Nvec = list with a length V equal to the number of independent
            variables. The ith element values are integers of polynomial
            order of the ith independent variable
        x = numpy matrix of size k by V, where k is the total number of
            points in the dataset. The ith column represents the ith
            independent variable values with the rows representing the
            different data points
    
    returns:
    
        f = value of the multivariable polynomial function for the given
            independent variables
    """
    # initialize summation to 0
    f = 0.
    # calculate total number of datapoints
    k = len(a)
    # calculate total number of dimensions
    V = len(x)
    # loop through the datapoints
    for j in range(k):
        # calculate the n values
        n = decompose_j(j, Nvec)
        # initialize the product series variable to 1
        prod = 1.
        # loop through the dimensions
        for v in range(V):
            # multiply onto the product series the term
            prod *= x[v] ** n[v]
        # add onto the summation the proper term
        f += a[j] * prod
    # return the finalized summation value
    return f

def multivariableR2(a, Nvec, xx, yy):
    """
    Routine to calculate the R^2 value of a multivariable polynomial fit to
    a dataset
    
    inputs:
    
        a = array of polynomial coefficients
        Nvec = list of integers representing the polynomial order of the
            independent variables
        xx = numpy matrix of size k by V, where k is the total number of
            points in the dataset. The ith column represents the ith
            independent variable values with the rows representing the
            different data points
        yy = list with a length of k with the dependent variable values
    
    returns:
    
        R2 = R^2 value, or the coefficient of determination
    """
    # ensure x and y are in the proper format
    x = np.copy(xx)
    y = np.copy(yy)
    ynew = np.copy([temp for temp in y if temp != None])
    # calculate k
    k = len(ynew)
    # calculate mean y value
    y_ = sum(ynew) / float(k)
    # calculate the SSt value
    SSt = sum( (ynew - y_) ** 2. )
    # initialize the f array
    f = []
    # loop through the datapoints
    for i in range(len(y)):
        if y[i] == None: continue
        # calculate the f value from the polynomial function
        f.append( multivariablePolynomialFunction(a,Nvec,x[i,:]) )
    f = np.copy(f)
    # calculate the SSr term
    SSr = sum( (ynew - f) ** 2. )
    # calculate and return the R^2 value
    return 1. - SSr / SSt

def multivariableRMS(raw_x, raw_y, a, Nvec):
    """
    Routine to calculate the RMS and RMSN errors of a multivariable
    polynomial fit to a dataset
    
    inputs:
    
        raw_x = array of independent variables values from the dataset of
            shape (k,V) where k is the total number of points in the dataset
            and V is the number of independent variables
        raw_y = array of dependent variable values from the dataset
        a = array of polynomial coefficients
        Nvec = list of integers representing the polynomial order of the
            independent variables
    
    returns:
    
        RMS = root, mean, squared error
        RMSN = root, mean, squared, normalized error
    """
    x = np.copy( raw_x )
    y = np.copy( raw_y )
    avg = np.mean(abs(y))
    k = len(x[:,0])
    func = np.zeros(k)
    e = np.zeros(k)
    e_per = np.zeros(k)
    for i in range(k):
        func[i] = multivariablePolynomialFunction(a, Nvec, x[i])
        e[i] = (y[i] - func[i]) ** 2.
        e_per[i] = ((y[i] - func[i])/avg) ** 2.
    return np.sqrt(np.mean(e)), np.sqrt(np.mean(e_per))

def compose_j(n, Nvec):
    """
    Eq. 4 in Poly Fits Derivation. Routine to compose the j counter from
    the n values. Can also be used as Eq. 10 with the i counter and the nhat
    values.
    
    inputs:
    
        n = list of integer values representing the independent variables'
            exponents for the jth term in the multidimensional polynomial
            function (Eq. 2)
        Nvec = list of integers representing the polynomial order of the
            independent variables
    
    returns:
    
        j = integer representing the column of the A matrix or the jth
            polynomial coefficient
    """
    # calculate V
    V = len(Nvec)
    # initialize j to 0
    j = 0
    # loop through independent variables
    for v in range(1,V+1):
        # initialize product series to 1
        prod = 1
        # loop through w values for product series
        for w in range(v+1,V+1):
            # multiply on the term to the product series
            prod *= Nvec[w-1] + 1
        # add on term onto j
        j += n[v-1] * prod
    return j

def decompose_j(j, Nvec):
    """
    Eq. 5 in Poly Fits Derivation. Routine to decompose the j counter into
    the n values. Can also be used as Eq. 1 with the i counter and the nhat
    values.
    
    inputs:
    
        j = integer representing the column of the A matrix or the jth
            polynomial coefficient
        Nvec = list of integers representing the polynomial order of the
            independent variables
    
    returns:
    
        n = list of integer values representing the independent variables'
            exponents for the jth term in the multidimensional polynomial
            function (Eq. 2)
    """
    # calculate V
    V = len(Nvec)
    # initialize n values to nothing
    n = [[]]*V
    # loop through the n values that need to be solved, starting at the highest and working down
    for v in range(V,0,-1):
        # initialize the denomenator product series to 1
        denom = 1
        # loop through the w values needed for the product series
        for w in range(v+1,V+1):
            # multiply on the terms for the denomenator product series
            denom *= Nvec[w-1] + 1
        # initialize the summation variable to 0
        summ = 0
        # loop through the u values necessary for the summation
        for u in range(v+1,V+1):
            # initialize the product series variable inside the summation to 1
            prod = 1
            # loop through the s values needed for the product series that is inside the summation
            for s in range(u+1,V+1):
                # multiply on the term for the product series that is inside of the summation
                prod *= Nvec[s-1] + 1
            # add on the needed term to the summation series
            summ += n[u-1] * prod
        # finally calculate the n value cooresponding to this v
        n[v-1] = int(round( ((j-summ)/denom)%(Nvec[v-1]+1) ))
    return n


