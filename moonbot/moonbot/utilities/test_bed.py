import params

import math

L1 = params.L2
L2 = params.L3

dz = 0.0005
some = L1 + L2 - dz
cos1 = (L1**2 + some**2 - L2**2)/(2 * L1 * some)

cos2 = (L1**2 + L2**2 - some**2)/(2 * L1 * L2)

print(-90.0 + math.degrees(math.acos(cos1)), 180 - math.degrees(math.acos(cos2)))