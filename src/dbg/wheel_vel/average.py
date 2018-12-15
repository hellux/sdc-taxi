import sys

offsets = [-0.03, 0.28, -0.23, -0.02, 0.03, 0.03, 0.03, -0.04, -0.06, 0.00]

t = []
v = []
w = []

period = 10

for line in sys.stdin:
    _t, _v, _w = map(float, line.split())
    if _v > 0:
        non_zero = True
        t.append(_t)
        v.append(_v)
        w.append(_w)

p_sum = 0
avg = []
i = 0
while i < len(t):
    p = i % period
    p_sum += v[i]
    if p == period-1:
        avg.append(p_sum/period)
        p_sum = 0
    i += 1

p_offs = []
offs = []
i = 0
while i < len(t)-(len(t)%period):
    a = avg[int(i/period)]
    p_offs.append(v[i]-a)
    if i % period == period-1:
        offs.append(p_offs)
        p_offs = []
    i += 1

