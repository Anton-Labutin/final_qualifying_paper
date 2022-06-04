results = cell(4, 5)
braking = struct('ok', true)
speedup = struct('ok', false)
change1 = struct('ok', true, 'curvature_range', [1, 2])
change2 = struct('ok', false)

results{1, 2} = {braking, speedup, change1}
results{2, 3} = {braking, speedup, change2}
results(1, 2)
results{1, 2}{1, 1}.ok
