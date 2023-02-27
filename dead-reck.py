

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

import math
print("---")

circ = math.pi * 87.5 # mm
ticks_per_rev = 294 # ticks
mm_per_tick = circ / ticks_per_rev

dist = mm_per_tick * 277.0
print(dist)
actual = dist / math.cos(math.radians(30))
print(actual)
