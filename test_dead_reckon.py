
# test our math on dead-reckoning code

import math

# 90mm wheels measured at 86mm circumference -- double-check!
# 294 ticks per revolution
mm_per_tick = (math.pi * 86) / 294.0

# pretend our encoders say we went this far:
left, right, slide = 294, -294, 0

l = left * mm_per_tick
r = right * mm_per_tick
s = slide * mm_per_tick

print(l, r, s)


# first test: drove forward
# 109cm
# encoder_right = -1249
# encoder_left = 1307
# pos_x = -8.42
# pos_y = -620.49

# test two: driving "strafe right"
# 111cm
# encoder_right = 583
# encoder_left = 522
# slide_encoder = -1240

print("DINGDING", math.cos(math.radians(60)), math.sin(math.radians(60)))
print("ZINGA   ", math.cos(math.radians(120)), math.sin(math.radians(120)))

lx = left * math.cos(math.radians(60))
ly = left * math.sin(math.radians(60))

rx = right * math.cos(math.radians(120))
ry = right * math.sin(math.radians(120))

print(lx, ly, rx, ry)

moved_x = lx + rx + slide
moved_y = ly + ry + slide

print("ZZZZ", 1240 * mm_per_tick)

print(mm_per_tick)
print(moved_x, moved_y)
print(int(moved_x), int(moved_y))

# hmm .. okay so lx + rx ~= 0 .. good
# ly ~= lx ~= 270mm .. good, ish?
# ...but if we _add_ them, we get too much (i.e. 270mm is about the right answer)
# so do we take the _average_?

print("XX", (ry + ly + 0) / 3.0)
print("ZZ", (ry + ly + 0))
# that's 170mm .. so we rotated a wheel that's 270mm 1x
# ... intuitively, it's about 2/3rds "forward"?
# ... so like 180? ... hmm, maybe the average is right?
# ... and other stuff goes into rotation?
# ... or can we use the IMU to _remove_ rotation?

# hrrrrmmmm
