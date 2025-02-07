lx = 100
lz = 100

u = 3
r = 1

# Get the Sphere.obj file
with open("Sphere.obj", "w") as f:
    f.write("mtllib Sphere.mtl\n\n")

    for x in range(lx):
        for z in range(lz):
            f.write(f"v {u * (x - lx / 2)} {r} {u * (z - lz / 2)}\n")

    f.write("\no Sphere\n")
    
    for i in range(lx * lz):
        f.write(f"sph {i + 1} 1\n")