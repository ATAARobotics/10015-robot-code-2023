

x, y, actual_y = [16.08, 1759.63, 1080]
x, y, actual_y = [22.5, 1747.00, 1080]


avg = (1759.63 + 1747.0) / 2.0
factor = 1080.0 / avg
print(factor)
print(1182.253978233065 * factor)
print(1188.6867631904156 * factor)


## emprical push-test
## pushed forward 30cm
## encoder-left 278
## encoder-right -277

# (couple different tests found these values instead)
# encoder-left 260
# encoder-right -262

# caliper wheel: 87.5 mm is diameter


## based on an idea from adrian:
## "what if we just point in the direction of one of the wheels, and use all of its encoder/distance value?"
##
## then thinking of generalizing that: we can take _any_ wheel,
## compute its angle offset to "actual direction driven" and "do trig"
## on it
##
## (the "averaging" in the other article was about meccanum .. so
## e.g. we could do the same during straight-forward motion, and
## average the _result_ of above Trig on the two front wheels.

import math
print("---")

circ = math.pi * 87.5 # mm
ticks_per_rev = 294 # ticks
mm_per_tick = circ / ticks_per_rev

dist = mm_per_tick * -277.0
print(dist)
actual = dist / math.sin(math.radians(-60))
print(actual)
actual = -dist / math.sin(math.radians(-300))
print(actual)

"""
so, trying to generalize.

if 0 degrees == straight right
then 90 is "straight forward"

 (confirm: does our robot do this? or is 0 straight forward??)

so at 90 degrees, our 60 right wheel is 90-60 = 30 degrees off
...and our 120 wheel is 90 - 120 = -30 degrees off

...no, 0 is forward in our setup:

60, 300, 180 are wheels.

0 == forward, 60 == right, 180 == slide, 300 == left
"""
