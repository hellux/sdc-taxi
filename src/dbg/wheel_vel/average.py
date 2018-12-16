import sys

offsets = [-0.03, 0.28, -0.23, -0.02, 0.03, 0.03, 0.03, -0.04, -0.06, 0.00]

t = []
v = []
w = []

period = 10

for line in sys.stdin:
    values = list(map(float, line.split()))
    t.append(values[0])
    v.append(values[1])
    w.append(values[2])

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

period = 5
va = [0 for _ in range(period)]
vi = 0
vs = 0
v5 = []
v10 = []
for _v in v:
    if _v == 0:
        vs = 0
        v5.append(0)
        vi = 0
    else:
        vs -= va[vi]
        vs += _v
        va[vi] = _v
        vi = (vi + 1) % period
        v5.append(vs/period)

period = 10
va = [0 for _ in range(period)]
vi = 0
vs = 0
v10 = []
for _v in v:
    if _v == 0:
        vs = 0
        v10.append(0)
        vi = 0
    else:
        vs -= va[vi]
        vs += _v
        va[vi] = _v
        vi = (vi + 1) % period
        v10.append(vs/period)

with open('avg.dat', 'w') as f:
    for _t, _v, _w, _v5, _v10 in zip(t, v, w, v5, v10):
        f.write('{} {} {} {} {}\n'.format(_t, _v, _w, _v5, _v10))
