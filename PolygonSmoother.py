from geopy import distance
from trianglesolver import solve
from RDP import rdp
from math import degrees
from scipy.spatial import ConvexHull



def remove_duplicates(coords = []):
    '''Steps through the list of coordinates to find and remove coordinates that appear more than once

    Parameters
    ----------
    coords : list
        List of coordinates to remove duplicates from

    Returns
    -------
    list
        the original list of coordinates without duplicates
    '''
    new_list = []
    for point in coords:
        if point not in new_list:
            new_list.append(point)
    return new_list

def nearest_neighbor_smoother(coords=[]):
    ''' A nearest neighbor smoother steps through the list of coordinates and adjust each coordinate
        based on the values of the previoys and next values, wrapping around when necessary.

        This approach affects the overall area of the polygon since each coordinate gets adjusted to a new coordinate 

    Parameters
    ----------
    coords : list
        List of coordinates to smooth

    Returns
    -------
    list
        A new list of smoothed out coordinates
    '''
    for n in range(len(coords)):
        prevIndex = n-1
        nextIndex = n+1
        if prevIndex == -1:
            prevIndex = len(coords)-1
        if nextIndex >= len(coords):
            nextIndex = 0
        
        coords[n][0] = coords[prevIndex][0] * 0.3 + coords[n][0] * .4 + coords[nextIndex][0] * .3
        coords[n][1] = coords[prevIndex][1] * 0.3 + coords[n][1] * .4 + coords[nextIndex][1] * .3

    return coords

def remove_sharp_angle(coords = [], deg_threshold = 30.0):
    ''' Detect and remove sharp angles from a polygon
        Uses harversine formaular or spherical law of cosine to calculate distance between 2 coordinates
        Uses the law of cosines to detect and remove angles sharper than the specified angles

    Parameters
    ----------
    coords : list
       List of coordinates to detect angles for
    deg_threshold: float
        the maximum angle to allow. Coordinates that that form angles below this value will be removed

    Returns
    -------
    list
        the original list of coordinates with sharp angles trimmed
    '''
    list_to_modify = [x for x in coords]
    coords_count = len(coords)
    if(coords_count <= 3):
        return coords

    for i in range(coords_count):
        if(i+1 >= coords_count or i+2 >= coords_count):
            return list_to_modify

        coords_1 = coords[i]
        coords_2 = coords[i+1]
        coords_3 = coords[i+2]
        d1 = distance.distance(coords_1, coords_2)
        d2 = distance.distance(coords_2, coords_3)
        d3 = distance.distance(coords_3, coords_1)
        d1 = float(str(d1)[:-3])*1000
        d2 = float(str(d2)[:-3])*1000
        d3 = float(str(d3)[:-3])*1000
        if d1 > 0.01 and d2 > 0.01 and d3 > 0.01: # if they are 0, there will be an error
            a,b,c,A,B,C = solve(a=d1, b=d2, c=d3) # Calculate the angles from the sides
            A,B,C = degrees(A), degrees(B), degrees(C) # Convert to math.degrees
            if (360.0 - C) < deg_threshold or C < deg_threshold:
                list_to_modify.remove(coords[i+1])

def remove_lakes_islands(coords = [], deg_threshold = 180.0):
    list_to_modify = [x for x in coords]
    coords_count = len(coords)
    if(coords_count <= 3):
        return coords

    for i in range(coords_count):
        if(i+1 >= coords_count or i+2 >= coords_count):
            return list_to_modify

        coords_1 = coords[i]
        coords_2 = coords[i+1]
        coords_3 = coords[i+2]
        d1 = distance.distance(coords_1, coords_2)
        d2 = distance.distance(coords_2, coords_3)
        d3 = distance.distance(coords_3, coords_1)
        d1 = float(str(d1)[:-3])*1000
        d2 = float(str(d2)[:-3])*1000
        d3 = float(str(d3)[:-3])*1000

        if d1 > 0.01 and d2 > 0.01 and d3 > 0.01: # if they are 0, there will be an error
            a,b,c,A,B,C = solve(a=d1, b=d2, c=d3) # Calculate the angles from the sides
            A,B,C = degrees(A), degrees(B), degrees(C) # Convert to math.degrees
            if C > deg_threshold:
                #spike= True
                list_to_modify.remove(coords[i+1])

def run_angle_remover(iterations=3, coords = [], deg_threshold=30.0):
    for i in range(3):
        coords = remove_sharp_angle(coords=coords, deg_threshold=deg_threshold)

    return coords

def k_sample_smoother(k=3, coords=[], strategy='delete'):
    for i in range(0,len(coords),3):
        index1 = i
        index2 = i+1
        index3 = i+2
        index4 = i+3
        index5 = i+4
        if index2 >= len(coords):
            index2 = 0
        if index3 >= len(coords):
            index3 = 1
        if index4 >= len(coords):
            index4 = 2
        if index5 >= len(coords):
            index5 = 3

        pA = coords[i]
        pB = coords[index2]
        pC = coords[index3]
        pD = coords[index4]
        pE = coords[index5]

        AB = distance.distance(pA, pB)
        AC = distance.distance(pA, pC)
        AD = distance.distance(pA, pD)
        AE = distance.distance(pA, pE)
        BD = distance.distance(pB, pD)
        BE = distance.distance(pB, pE)
        BC = distance.distance(pB, pC)
        CD = distance.distance(pC, pD)
        CE = distance.distance(pC, pE)
        DE = distance.distance(pD, pE)
        
        ATOTAL = sum([float(str(AB)[:-3])*1000, float(str(AC)[:-3])*1000, float(str(AD)[:-3])*1000, float(str(AE)[:-3])*1000])
        BTOTAL = sum([float(str(AB)[:-3])*1000, float(str(BC)[:-3])*1000, float(str(BD)[:-3])*1000, float(str(BE)[:-3])*1000])
        CTOTAL = sum([float(str(AC)[:-3])*1000, float(str(BC)[:-3])*1000, float(str(CD)[:-3])*1000, float(str(CE)[:-3])*1000])
        DTOTAL = sum([float(str(AD)[:-3])*1000, float(str(BD)[:-3])*1000, float(str(CD)[:-3])*1000, float(str(DE)[:-3])*1000])
        ETOTAL = sum([float(str(AE)[:-3])*1000, float(str(BE)[:-3])*1000, float(str(CE)[:-3])*1000, float(str(DE)[:-3])*1000])

        # If BTOTAL is largest you would replace point B with D if BD = min { AB AC AD AE }
        if ATOTAL > BTOTAL and ATOTAL > CTOTAL and ATOTAL > DTOTAL and ATOTAL > ETOTAL: #AB AC AD AE
            if AB < AC and AB < AD and AB < AE:
                coords[i] = pB
            elif AC < AB and AC < AD and AC < AE:
                coords[i] = pC
            elif AD < AB and AD < AC and AD < AE:
                coords[i] = pD
            elif AE < AB and AE < AC and AE < AD:
                coords[i] = pE
        elif BTOTAL > ATOTAL  and BTOTAL > CTOTAL and BTOTAL > DTOTAL and BTOTAL > ETOTAL: #AB BC BD BE
            if AB < BC and AB < BD and AB < BE:
                coords[index2] = pB
            elif BC < AB and BC < BD and BC < BE:
                coords[index2] = pC
            elif BD < AB and BD < BC and BD < BE:
                coords[index2] = pD
            elif BE < AB and BE < BC and BE < BD:
                coords[index2] = pE
        elif BTOTAL > ATOTAL  and BTOTAL > CTOTAL and BTOTAL > DTOTAL and BTOTAL > ETOTAL: #AC BC CD CE
            if AC < BC and AC < CD and AC < CE:
                coords[index3] = pC
            elif BC < AC and BC < CD and BC < CE:
                coords[index3] = pC
            elif CD < AC and CD < BC and CD < CE:
                coords[index3] = pD
            elif CE < AC and CE < BC and CE < CD:
                coords[index3] = pE
        elif BTOTAL > ATOTAL  and BTOTAL > CTOTAL and BTOTAL > DTOTAL and BTOTAL > ETOTAL: #DA DB DC DE
            if DA < DB and DA < DC and DA < DE:
                coords[index3] = pA
            elif DB < DA and DB < DC and DB < DE:
                coords[index3] = pB
            elif DC < DA and DC < DB and DC < DE:
                coords[index3] = pC
            elif DE < DA and DE < DB and DE < DC:
                coords[index3] = pE
        elif BTOTAL > ATOTAL  and BTOTAL > CTOTAL and BTOTAL > DTOTAL and BTOTAL > ETOTAL: # EA EB EC DE
            if EA < EB and EA < EC and EA < DE:
                coords[index3] = pA
            elif EB < EA and EB < EC and EB < DE:
                coords[index3] = pB
            elif EC < EA and EC < EB and EC < DE:
                coords[index3] = pC
            elif DE < EA and DE < EB and DE < EC:
                coords[index3] = pE

    return coords

'''
The Ramer-Douglas-Peucker algorithm is an algorithm for reducing the number 
of points in a curve that is approximated by a series of points.
'''
def compute_significant_points(epsilon = 4, coords = []):
    return rdp(coords, 1.0)

def reduce_point(coords=[], epsilon = 4):
    ''' A function to smartly reduced the number of coordinates in a polygon to a desired number
        by ranking cordinates based on their distances from each other. The idea is to find the most
        useful n (epsilon) points that can still represent the original polygon
        Coordinates that are farther away from each other will be ranked higher
        While coordinates that are closer to each other will be ranked lower
        The function returns the first n (epsilon) highest ranked coordinates
        If you wish to maintain the shape of the original polygon then do not make this value too low, 
        otherwise, this function is capable of repres
     
        NOTE: Only call this function with an already smoothed polygon (i.e. coords should be the result of a call to
        either a convex_hull() or map_cleaner() function)
        In addition, the function assumes that the polygon is smoothed and that epsilon >= 4

    Parameters
    ----------
    coords : list
         Coordinates of the polygin to be reduced
    epsilon:
        The number of coordinates to keep. If you wish to maintain the shape
        of the original polygon then do not make this value too low.
        epsilon must be an even number

    Returns
    -------
    list
        the polygon with reduced coordinates
    '''
    if epsilon % 2 != 0:
        epsilon -= 1

    coordinate_count = len(coords)
    if coordinate_count == 0 or coordinate_count == 4:
        return coords

    coords = remove_duplicates(coords)
    coordinate_count = len(coords)
    dist_counter = 1
    for h in range(coordinate_count):
        max_distance = 0.0
        x = 0 
        y = 0
        for i in range(coordinate_count):
            for j in range (i+1, coordinate_count):
                if len(coords[i]) < 3 and len(coords[j]) < 3:
                    d1 = distance.distance(coords[i], coords[j])
                    d1 = float(str(d1)[:-3])*1000
                    if d1 > max_distance:
                        max_distance = d1
                        x = i
                        y = j
        if x == 0 and y == 0:
            break

        if len(coords[x]) < 3:
            coords[x].append(dist_counter)
        if y < coordinate_count and len(coords[y]) < 3:
            coords[y].append(dist_counter)
        dist_counter += 1


    contracted_coords = []
    for i in range(coordinate_count):
        if len(coords[i]) == 3 and coords[i][2] <= epsilon:
            coords[i].pop(2)
            contracted_coords.append(coords[i])

    contracted_coords.append(contracted_coords[0])
    return contracted_coords

def remove_outliers(coords = [], dist_threshold = 1000):
    ''' Detects outliers by computing distance between every two cordinates
        Any point that's more than dist_threshold(Km) away from any other point is an outlier
        This approach can be problematic because this action can create fake outliers in the resulting polygon
    
    
    Parameters
    ----------
    coords : list
        List of coordinates to smooth

    dist_threshold: int
        the minimum distance to use to classify a point as an outlier

    Returns
    -------
    list
        A new list without outliers
    '''
    smooth_coodinates = list()
    for i in range(len(coords)):
        nextIndex = i+1
        if nextIndex >= len(coords):
            nextIndex = 0
        d1 = distance.distance(coords[i], coords[nextIndex])
        if d1 < dist_threshold:
            smooth_coodinates.append(coords[i])

    return smooth_coodinates

def convex_hull(coords=[]):
    ''' Compute the convex hull of given set of coordinate points
    
    Parameters
    ----------
    coords : list
        List of coordinates to smooth

    Returns
    -------
    list
        A new list of smoothed out coordinates
    '''
    hull = ConvexHull(coords)
    conv_hull = list()
    for v in sorted(hull.vertices):
        conv_hull.append(coords[v])
    
    return conv_hull