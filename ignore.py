import math
import cmath

a = float(input("coefficient of x^2: "))
if a == 0:
    print("Calculation exited with Error")
b = float(input("coefficient of x: "))
c = float(input("constant term: "))

discriminant = b**2 - 4*a*c

mx = "y max"
mn = "y min"

def disting(a):
    global s
    if a > 0:
        s = mn
    else:
        s = mx
    return s

if discriminant < 0:
    disting(a)
    print("No real root")
    b = -b
    x = (b + cmath.sqrt(discriminant)) / 2.0
    x_1 = (b - cmath.sqrt(discriminant)) / 2.0
    print(f"x1 = {x}, x2 = {x_1}")
    los = (x + x_1) / 2
    y = a*los**2 + b*los + c

    print(f"Value of {s} = {y.real}")
    print(f"Value of Line of symmetry x = {los.real}")

elif discriminant == 0:
    disting(a)
    x = -1*b/2*a
    print(f"x={x}")
    los = x
    y = a * los ** 2 + b * los + c
    print(f"Value of {s} = {y}")
    print(f"Value of Line of symmetry x = {los}")

else:
    disting(a)
    b = b*-1
    x = (b + math.sqrt(discriminant))/2
    x_1 = (b - math.sqrt(discriminant))/2
    print(f"x1 = {x}, x2 = {x_1}")
    los = (x + x_1) / 2
    y = a * los ** 2 + b * los + c
    print(f"Value of {s} = {y}")
    print(f"Value of Line of symmetry x = {los}")