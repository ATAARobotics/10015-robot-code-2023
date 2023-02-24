
# test our math on dead-reckoning code

import math

# 90mm wheels measured at 86mm circumference -- double-check!
# 294 ticks per revolution
mm_per_tick = (math.pi * 86) / 294.0

# pretend our encoders say we went this far:
left, right, slide = 294, -294, 0
time = 0.3

if True:
    l = (left * mm_per_tick) / time
    r = (right * mm_per_tick) / time
    s = (slide * mm_per_tick) / time
    # mm / s
else:
    l = left * mm_per_tick
    r = right * mm_per_tick
    s = slide * mm_per_tick


print(l, r, s)

thirty = math.radians(30)
print("thirty", thirty, math.cos(thirty), math.sin(thirty))

y = (-l * math.cos(thirty)) + (r * math.cos(thirty))
x = (l * math.sin(thirty)) + (r * math.sin(thirty)) - s

print("mm / s", x, y)
print(x * time, y * time)
