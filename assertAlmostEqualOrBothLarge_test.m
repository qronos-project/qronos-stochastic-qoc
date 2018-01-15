% unit-tests for assertAlmostEqualOrBothLarge
function assertAlmostEqualOrBothLarge_test
% true:
assertAlmostEqualOrBothLarge(42, Inf, 40, 1e-42)
assertAlmostEqualOrBothLarge(42, 1337, 40, 1e-42)
reltol = 1e-6;
assertAlmostEqualOrBothLarge(42, 40 * (1 - reltol + eps), 40, reltol)

% corner cases:
assertAlmostEqualOrBothLarge(0, 0, 40, reltol)

% false:
assertExceptionThrown(@() assertAlmostEqualOrBothLarge(1, 1 + 2 * reltol, 40, reltol), '');
assertExceptionThrown(@() assertAlmostEqualOrBothLarge(10 * 1, 10 * (1 + 2 * reltol), 40, reltol), '');
assertExceptionThrown(@() assertAlmostEqualOrBothLarge(1, 42, 40, reltol), '');
assertExceptionThrown(@() assertAlmostEqualOrBothLarge(1, 0, 40, reltol), '');
end