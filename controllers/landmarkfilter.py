""" Provides thin_scan which reduces contiguous spans of the active_mask to a single central element. """

from math import pi
from copy import copy
from sensors import RangeScan
from common.angles import get_smallest_angular_difference

def thin_scan(this_robot, in_scan, active_mask):

    n = len(in_scan.ranges)

    out_scan = copy(in_scan)

    label = 1
    i = 0
    while i < n:
        span_indices = []
        while i < n and in_scan.masks[i] & active_mask != 0:
            span_indices.append(i)
            i += 1
        i += 1
        
        if len(span_indices) > 0:
            # We find the index of the closest value, then erase from the span
            # all other entries.
            closest_range = float('inf')
            closest_k = None
            for k in span_indices:
                if out_scan.ranges[k] < closest_range:
                    closest_range = out_scan.ranges[k]
                    closest_k = k
            for k in span_indices:
                if k != closest_k:
                    out_scan.ranges[k] = out_scan.RANGE_MAX
                    out_scan.masks[k] = 0

    # If the scan is panoramic then we could get active entries on either end
    # of the scan that should have been combined.  So we scan forwards and
    # backwards to find the first and last active entries and if they are too
    # close to each other we delete the closest one. 
    if (out_scan.ANGLE_MAX - out_scan.ANGLE_MIN > (0.9 * 2 * pi)):
        first_active_index = 0
        while out_scan.masks[first_active_index] & active_mask == 0:
            first_active_index += 1
        last_active_index = n-1
        while out_scan.masks[last_active_index] & active_mask == 0:
            last_active_index -= 1
        if first_active_index != last_active_index: # true for single active
            diff = get_smallest_angular_difference(
                out_scan.angles[last_active_index],
                out_scan.angles[first_active_index])
            if diff < pi/8:
                # Now delete the closest one.
                if (out_scan.ranges[first_active_index] < 
                   out_scan.ranges[last_active_index]):
                    out_scan.ranges[last_active_index] = out_scan.RANGE_MAX
                    out_scan.masks[last_active_index] = 0
                else:
                    out_scan.ranges[first_active_index] = out_scan.RANGE_MAX
                    out_scan.masks[first_active_index] = 0
            
    return out_scan
