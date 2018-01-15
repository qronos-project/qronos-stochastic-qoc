% assertAlmostEqualOrBothLarge(a, b, threshold_large, relative_tolerance):
% Assert that (a,b) are equal(*) or both larger than(*) threshold_large.
% If not, an error is raised.
% (*) Comparisons are done with a relative_tolerance.
function assertAlmostEqualOrBothLarge(a, b, threshold_large, relative_tolerance)
assert(~isnan(relative_tolerance))

if a > threshold_large
    assert(b > threshold_large * (1 - relative_tolerance))
else
    assert(a == b || abs((a - b) / b) < relative_tolerance, sprintf('Values %e and %e differ more than the rel. tolerance %e', a, b, relative_tolerance))
end
end