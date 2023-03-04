import math

notes = """
Telemetry

status: initialized
avg_head: 1.2103282187037683
d: 904.7056364561706
dead_actual: 0.0
dead_diff: 58.53596818447113
dead_head: 1.4640318155288696
dead_ticks: 0.0
dead_total: 917.2522526739408
enc_left: -929
enc_right: 899
enc_slide: -1
moved_x: 969.468341138227
moved_y: 20.48230191060073
pos_x: 916.9359272249383
pos_y: 21.123277698248966
stick_y: 0.0
travel_left: 969.6846854317452
travel_right: 904.7056364561706
travel_slide: 0.0


877mm actual
"""

mm_per_tick = (math.pi * 87.5) / 294.0;

travel_left = 969.6846854317452
travel_right = 904.7056364561706


# contribution of each wheel to the total distance is:
# _it's_ difference from the desired heading
# ...we _want_ to just average left + right
# ...but not in all cases; in some we want to average all 3?
# ... or, I guess: we can always average "the closest two"!


avg = (travel_left + travel_right) / 2.0

print(mm_per_tick * travel_left)
print(mm_per_tick * travel_right)
print(mm_per_tick * avg)

avg_head = 1.2103282187037683

print(math.cos(math.radians(avg_head)))
print(math.sin(math.radians(avg_head)))

x = math.cos(math.radians(avg_head)) * mm_per_tick * avg
print("x", x)

# measure: 102cm
# detect: travel_avg: 1186.7055957997459

#print("ZZZ", ((1173 * mm_per_tick) / math.sin(math.radians(60))))
print("ZZZ", ((1473.0 * mm_per_tick) / math.sin(math.radians(60))))
print("ZZZ", (((1483.0 + 1347) / 2.0 * mm_per_tick) / math.sin(math.radians(60))))
print("ZZZ", (1483.0 * mm_per_tick) / math.sin(math.radians(60)))
